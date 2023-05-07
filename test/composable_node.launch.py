#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Copyright 2023 Tatsuro Sakaguchi
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:

    return LaunchDescription([
        LoadComposableNodes(
            target_container='container',
            composable_node_descriptions=[
                ComposableNode(package='test_composable_node1',
                               plugin='test1'),
                ComposableNode(package='test_composable_node2',
                               plugin='test2')
            ],
        )])
