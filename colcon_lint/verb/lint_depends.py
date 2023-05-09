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
from launch_ros.substitutions import FindPackageShare

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
            exec_depends = set()
            for file in pkg_path.glob('**/*.py'):
                exec_depends |= self.resolve_depends(file)
            tree = ElementTree.parse(pkg_path.parent / 'package.xml')
            root = tree.getroot()
            described_exec_depends = set([dep.text for dep in root.iter('exec_depend')])
            described_depends = set([dep.text for dep in root.iter('depend')])

            python_sources = pkg_path.parents[4] / 'build' / pkg.name / pkg.name
            import_depends = set()
            for file in python_sources.glob('**/*.py'):
                with open(file) as f:
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
                                import_depends.add(package)
                            except Exception:
                                pass

            detected = exec_depends | import_depends
            missing = detected - described_exec_depends - described_depends - set([pkg.name])
            unnecessary = described_exec_depends - detected
            if missing:
                logger.warn(f'[{pkg.name}] missing packages: {missing}')
                rc = 1
            if unnecessary:
                logger.warn(f'[{pkg.name}] unnecessary packages: {unnecessary}')
        return rc

    def resolve_depends(self, path: pathlib.Path) -> set:
        depends = set()
        description = get_launch_description_from_python_launch_file(str(path))
        context = LaunchContext()
        for entity in description.entities:
            depends |= self.parse_entity(entity, context)
        return depends

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
