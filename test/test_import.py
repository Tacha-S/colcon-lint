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


def test_python_package() -> None:
    linter = LintVerb()
    assert not linter.resolve_python_package('numpy')
    assert linter.resolve_python_package('python3-numpy')


def test_import() -> None:
    linter = LintVerb()
    deps = linter.resolve_import_depends(pathlib.Path(__file__).parent / 'import.py')
    assert deps == set(['rclpy', 'python3-numpy', 'std_msgs'])
