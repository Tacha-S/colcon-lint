#!/usr/bin/env python
# -*- coding:utf-8 -*-

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

import pathlib

from colcon_lint.verb.lint_depends import LintVerb


def test_node() -> None:
    linter = LintVerb()
    deps = linter.resolve_launch_depends(pathlib.Path(__file__).parent / 'node.launch.py')
    assert deps == set(['test_node'])


def test_composable_node() -> None:
    linter = LintVerb()
    deps = linter.resolve_launch_depends(pathlib.Path(__file__).parent / 'composable_node.launch.py')
    assert deps == set(['test_composable_node1', 'test_composable_node2'])


def test_group_action() -> None:
    linter = LintVerb()
    deps = linter.resolve_launch_depends(pathlib.Path(__file__).parent / 'group_action.launch.py')
    assert deps == set(['test_group1', 'test_group2'])


def test_include() -> None:
    linter = LintVerb()
    deps = linter.resolve_launch_depends(pathlib.Path(__file__).parent / 'include.launch.py')
    assert deps == set(['test_include'])


def test_arg() -> None:
    linter = LintVerb()
    deps = linter.resolve_launch_depends(pathlib.Path(__file__).parent / 'arg.launch.py')
    assert deps == set(['test_arg'])


def test_opaque() -> None:
    linter = LintVerb()
    deps = linter.resolve_launch_depends(pathlib.Path(__file__).parent / 'opaque.launch.py')
    assert deps == set(['test_opaque'])


def test_event_handler() -> None:
    linter = LintVerb()
    deps = linter.resolve_launch_depends(pathlib.Path(__file__).parent / 'event_handler.launch.py')
    assert deps == set(['test_node', 'test_on_exit'])
