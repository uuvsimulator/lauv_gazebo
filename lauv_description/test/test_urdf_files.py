#!/usr/bin/env python
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
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
from __future__ import print_function
import unittest
import subprocess
import os

import launch_testing.actions
import launch_testing.markers
import pytest


from launch import LaunchDescription


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([
            launch_testing.actions.ReadyToTest(),
        ])


def call_xacro(xml_file):
    assert os.path.isfile(xml_file), 'Invalid XML xacro file'
    return subprocess.check_output(['xacro', xml_file])


class TestURDFFiles(unittest.TestCase):
    def test_xacro(self):
        # Retrieve the root folder for the tests
        test_dir = os.path.abspath(os.path.dirname(__file__))
        robots_dir = os.path.join(test_dir, '..', 'robots')

        for item in os.listdir(robots_dir):
            if not os.path.isfile(os.path.join(robots_dir, item)):
                continue
            output = call_xacro(os.path.join(robots_dir, item)).decode("utf-8")
            self.assertNotIn(
                output, 
                'XML parsing error',
                'Parsing error found for file {}'.format(item))
            self.assertNotIn(
                output, 
                'No such file or directory', 
                'Some file not found in {}'.format(item))
