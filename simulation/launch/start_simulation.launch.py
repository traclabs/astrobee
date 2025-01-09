# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

from utilities.utilities import *
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    pkg_astrobee_gazebo = get_package_share_directory('astrobee_gazebo')
    world_filename = PythonExpression(["'", LaunchConfiguration("world"), "' + '.sdf'"])
    world_file = PathJoinSubstitution([pkg_astrobee_gazebo, 'worlds', world_filename])
    config_file = PathJoinSubstitution([pkg_astrobee_gazebo, 'config', "params.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("speed",   default_value="1"),
        DeclareLaunchArgument("debug",   default_value="false"),
        DeclareLaunchArgument("physics",   default_value="ode"),


        SetEnvironmentVariable(name='GAZEBO_RESOURCE_PATH', value="/usr/share/gazebo-11"),

#   <param name="/simulation_speed" value="$(arg speed)" />
# TODO(@mgouveia): Not sure what to do about the speed, I think I'll have to pass it to
# the plugin through sdf robot description since I can't set the parameter here

# ANA FIX THIS
#            get_launch_file( 'launch/gzserver.launch.py', 'gazebo_ros'),
#            launch_arguments = {
#                                'verbose': LaunchConfiguration('debug'),   # Debug a node set
#                                'physics': LaunchConfiguration('physics'), # SIM IP address
#                                'params_file': config_file,

        IncludeLaunchDescription(
            get_launch_file( 'launch/gz_sim.launch.py', 'ros_gz_sim'),
            launch_arguments = [
               ('gz_args', [
                   world_file,
                   ' -r',
                   ' -v 4', 
                   ' -s'
               ])
            ]   
        ),
      
        IncludeLaunchDescription(
            get_launch_file( 'launch/gz_sim.launch.py', 'ros_gz_sim'),
            launch_arguments = [
               ('gz_args', [
                   ' -g',
                   ' -v 4'
               ])
            ],
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
      
        # Publish the Clock -- Bridge ROS topics and Gazebo messages for establishing communication
        Node(
          name="bridge_clock",
          package='ros_gz_bridge',
          executable='parameter_bridge',
          parameters=[{
              'config_file': os.path.join(pkg_astrobee_gazebo, 'config', 'ros_gz_astrobee_bridge.yaml'),
              'qos_overrides./tf_static.publisher.durability': 'transient_local',
          }],
          output='screen'
        )                 

    ])
