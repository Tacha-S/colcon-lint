# Copyright 2023 Tatsuro Sakaguchi
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import argparse
import ast
import pathlib
import re
import shutil
import subprocess
from xml.etree import ElementTree

from colcon_core.command import CommandContext
from colcon_core.command import add_log_level_argument
from colcon_core.logging import colcon_logger
from colcon_core.package_selection import add_arguments as add_packages_arguments
from colcon_core.package_selection import get_package_descriptors
from colcon_core.package_selection import select_package_decorators
from colcon_core.topological_order import topological_order_packages
from colcon_core.verb import VerbExtensionPoint
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources.python_launch_description_source import \
    get_launch_description_from_python_launch_file
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackagePrefix
from launch_ros.substitutions import FindPackageShare
from rosdep2.lookup import RosdepLookup

logger = colcon_logger.getChild(__name__)


class LintVerb(VerbExtensionPoint):
    def __init__(self) -> None:
        super().__init__()

    def add_arguments(self, *, parser: argparse.ArgumentParser) -> None:
        add_packages_arguments(parser)
        add_log_level_argument(parser)

    def main(self, *, context: CommandContext) -> int:
        descriptors = get_package_descriptors(context.args, additional_argument_names=['*'])
        decorators = topological_order_packages(descriptors, recursive_categories=('run', ))
        select_package_decorators(context.args, decorators)

        rc = 0
        for decorator in decorators:
            if not decorator.selected:
                continue
            pkg = decorator.descriptor
            pkg_path = pathlib.Path(FindPackageShare(pkg.name).find(pkg.name)) / 'launch'
            launch_depends = set()
            for file in pkg_path.glob('**/*.py'):
                launch_depends |= self.resolve_launch_depends(file)

            pkg_lib_path = pathlib.Path(FindPackagePrefix(pkg.name).find(pkg.name)) / 'lib'
            egg_link = list(pkg_lib_path.glob('**/site-packages/*egg-link'))
            import_depends = set()
            setup_py_depends = set()
            build_depends = set()
            build_export_depends = set()
            buildtool_depends = set()
            test_depends = set()
            pkg_build_path = pkg_path.parents[4] / 'build' / pkg.name
            if egg_link:
                with open(egg_link[0]) as f:
                    python_sources = pathlib.Path(f.read().split('\n')[0]) / pkg.name
                for file in python_sources.glob('**/*.py'):
                    import_depends |= self.resolve_import_depends(file)

                setup_py = python_sources.parent / 'setup.py'
                if setup_py.exists():
                    setup_py_depends = self.resolve_setup_py_depends(setup_py)
            elif (pkg_build_path / 'Makefile').exists():
                trace_file = pkg_build_path / 'trace.log'
                if trace_file.exists():
                    with open(trace_file) as f:
                        trace_log = f.readlines()
                else:
                    with open(pkg_build_path / 'Makefile') as f:
                        for line in f.readlines():
                            if line.startswith('CMAKE_SOURCE_DIR'):
                                src_dir = pathlib.Path(line.split('=')[1].strip())
                                break
                    tmp_dir = pathlib.Path('/tmp/colcon-lint')
                    tmp_dir.mkdir(exist_ok=True)
                    subprocess.Popen(['cmake',
                                      '-Wno-dev',
                                      '--trace-expand',
                                      '--trace-redirect=trace.log',
                                      src_dir],
                                     cwd=tmp_dir,
                                     stdout=subprocess.PIPE,
                                     stderr=None).wait()
                    trace_file = tmp_dir / 'trace.log'
                    with open(trace_file) as f:
                        trace_log = f.readlines()
                    shutil.rmtree(tmp_dir)
                cmake_depends = self.resolve_cmake_depends(trace_log)
                build_depends |= cmake_depends[0]
                build_export_depends |= cmake_depends[1]
                buildtool_depends |= cmake_depends[2]
                test_depends |= cmake_depends[3]

            exec_depends = launch_depends | import_depends | setup_py_depends
            depends = (build_depends & exec_depends) | (test_depends & exec_depends)
            exec_depends -= depends
            build_depends -= depends
            build_export_depends -= depends
            test_depends -= depends

            tree = ElementTree.parse(pkg_path.parent / 'package.xml')
            root = tree.getroot()
            described_buildtool_depends = set([dep.text for dep in root.iter('buildtool_depend')])
            described_build_depends = set([dep.text for dep in root.iter('build_depend')])
            described_build_export_depends = set([dep.text for dep in root.iter('build_export_depend')])
            described_test_depends = set([dep.text for dep in root.iter('test_depend')])
            described_exec_depends = set([dep.text for dep in root.iter('exec_depend')])
            described_depends = set([dep.text for dep in root.iter('depend')])
            for dep in buildtool_depends - described_depends - described_buildtool_depends - set([pkg.name]):
                logger.warn(f'[{pkg.name}] {dep} should add to buildtool_depend.')
                rc = 1
            for dep in build_depends - described_depends - described_build_depends - set([pkg.name]):
                logger.warn(f'[{pkg.name}] {dep} should add to build_depend.')
                rc = 1
            for dep in build_export_depends - described_depends - described_build_export_depends - set([pkg.name]):
                logger.warn(f'[{pkg.name}] {dep} should add to build_export_depend.')
                rc = 1
            for dep in test_depends - described_depends - described_test_depends - set([pkg.name]):
                logger.warn(f'[{pkg.name}] {dep} should add to test_depend.')
                rc = 1
            for dep in exec_depends - described_depends - described_exec_depends - set([pkg.name]):
                logger.warn(f'[{pkg.name}] {dep} should add to exec_depend.')
                rc = 1
            for dep in depends - described_depends - \
                    (described_build_depends & described_exec_depends) - set([pkg.name]):
                logger.warn(f'[{pkg.name}] {dep} should add to depend.')
                rc = 1
            described = described_depends | described_buildtool_depends | described_build_depends | \
                described_build_export_depends | described_test_depends | described_exec_depends
            for dep in described - depends - buildtool_depends - \
                    build_depends - build_export_depends - test_depends - exec_depends - set([pkg.name]):
                logger.warn(f'[{pkg.name}] {dep} cannot be resolved.')
        return rc

    def resolve_python_package(self, package: str) -> bool:
        lookup = RosdepLookup.create_from_rospkg()
        return len(lookup.get_views_that_define(package)) > 0

    def resolve_launch_depends(self, path: pathlib.Path) -> set:
        depends = set()
        description = get_launch_description_from_python_launch_file(str(path))
        context = LaunchContext()
        for entity in description.entities:
            depends |= self.parse_entity(entity, context)
        return depends

    def resolve_import_depends(self, path: pathlib.Path) -> set:
        depends = set()
        with open(path) as f:
            text = f.read()
            a = ast.parse(text)
            for line in a.body:
                if isinstance(line, (ast.Import, ast.ImportFrom)):
                    if isinstance(line, ast.Import):
                        package = line.names[0].name
                    else:
                        package = line.module.split('.')[0]
                    try:
                        FindPackageShare(package).find(package)
                        depends.add(package)
                    except Exception:
                        if self.resolve_python_package('python3-' + package):
                            depends.add('python3-' + package)
                        if self.resolve_python_package(package + '-pip'):
                            depends.add(package + '-pip')
        return depends

    def resolve_setup_py_depends(self, path: pathlib.Path) -> set:
        depends = set()
        with open(path) as f:
            text = f.read()
            a = ast.parse(text)
            for line in a.body:
                if hasattr(line, 'value') and isinstance(line.value, ast.Call) and line.value.func.id == 'setup':
                    for keyword in line.value.keywords:
                        if keyword.arg == 'install_requires':
                            for value in keyword.value.elts:
                                if self.resolve_python_package('python3-' + value.s):
                                    depends.add('python3-' + value.s)
                                if self.resolve_python_package(value.s + '-pip'):
                                    depends.add(value.s + '-pip')
        return depends

    def resolve_cmake_depends(self, trace_log: list[str]) -> tuple[set, set, set, set]:
        build_depends = set()
        build_export_depends = set()
        buildtool_depends = set()
        test_depends = set()
        is_test = False
        for line in trace_log:
            if 'CMakeLists.txt' in line and 'if(BUILD_TESTING )' in line:
                is_test = True
            if ('CMakeLists.txt' in line or 'ament_auto_package' in line) and 'ament_export_dependencies' in line:
                match = re.match(r'.*ament\_export\_dependencies\((.+)\)', line)
                if match:
                    for dep in match.group(1).split(';'):
                        dep = dep.strip()
                        try:
                            FindPackageShare(dep).find(dep)
                            build_export_depends.add(dep)
                        except Exception:
                            pass
                continue
            if 'find_package' not in line:
                continue
            if 'CMakeLists.txt' in line or 'ament_auto_find_build_dependencies' in line or \
                    'ament_lint_auto_find_test_dependencies' in line:
                match = re.match(r'.*find\_package\((.+)\)', line)
                if not match:
                    continue
                dep = match.group(1).split()[0]
                if dep in ['ament_cmake', 'ament_cmake_auto']:
                    buildtool_depends.add(dep)
                    continue
                try:
                    FindPackageShare(dep).find(dep)
                    if is_test:
                        test_depends.add(dep)
                    else:
                        build_depends.add(dep)
                except Exception:
                    pass
        return build_depends, build_export_depends, buildtool_depends, test_depends

    def parse_entity(self, entity: LaunchDescriptionEntity, context: LaunchContext) -> set:
        depends = set()
        if issubclass(type(entity), Node):
            depends.add(entity.node_package)
        elif issubclass(type(entity), ComposableNode):
            if isinstance(entity.package, list):
                depends |= set([self.parse_substitutions(entity.package)])
            else:
                depends.add(entity.package)
        elif issubclass(type(entity), LoadComposableNodes):
            for e in entity._LoadComposableNodes__composable_node_descriptions:
                depends |= self.parse_entity(e, context)
        elif isinstance(entity, GroupAction):
            for e in entity.get_sub_entities():
                depends |= self.parse_entity(e, context)
        elif isinstance(entity, IncludeLaunchDescription):
            paths = entity._IncludeLaunchDescription__launch_description_source._LaunchDescriptionSource__location
            depends |= self.parse_path(self.parse_substitutions(paths))
        elif isinstance(entity, DeclareLaunchArgument):
            subst = entity.default_value
            if subst is not None:
                text = self.parse_substitutions(subst)
                context._LaunchContext__launch_configurations.update({entity.name: text})
                depends |= self.parse_path(text)
            else:
                context._LaunchContext__launch_configurations.update({entity.name: entity.default_value})
        elif isinstance(entity, OpaqueFunction):
            entities = entity.execute(context)
            for e in entities:
                depends |= self.parse_entity(e, context)
        elif isinstance(entity, RegisterEventHandler):
            for tup in entity.describe_conditional_sub_entities():
                for e in tup[1]:
                    depends |= self.parse_entity(e, context)
        return depends

    def parse_substitutions(self, substitutions: list[Substitution]) -> str:
        ret = ''
        for substitution in substitutions:
            if isinstance(substitution, TextSubstitution):
                ret += substitution.text
            elif isinstance(substitution, str):
                ret += substitution
        return ret

    def parse_path(self, path: str) -> set:
        depends = set()
        seg = path.split('/')
        if 'share' in seg:
            package_index = seg.index('share') + 1
            depends.add(seg[package_index])
        return depends
