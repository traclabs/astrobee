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


def generate_launch_description():

    return LaunchDescription([
        # Update the environment variables relating to absolute paths
        SetEnvironmentVariable(name="ASTROBEE_ROBOT",         condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration("mlp"), "local")),
                               value=os.getenv("ASTROBEE_ROBOT", LaunchConfiguration("robot"))),
        SetEnvironmentVariable(name="ASTROBEE_WORLD",         condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration("mlp"), "local")),
                               value=os.getenv("ASTROBEE_WORLD", LaunchConfiguration("world"))),
        SetEnvironmentVariable(name="ASTROBEE_CONFIG_DIR",    condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration("mlp"), "local")),
                               value=os.getenv("ASTROBEE_CONFIG_DIR", "/opt/astrobee/config")),
        SetEnvironmentVariable(name="ASTROBEE_RESOURCE_DIR",  condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration("mlp"), "local")),
                               value=os.getenv("ASTROBEE_RESOURCE_DIR", "/res")),
        SetEnvironmentVariable(name="ROSCONSOLE_CONFIG_FILE", condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration("mlp"), "local")),
                               value=os.getenv("ROSCONSOLE_CONFIG_FILE", "/res/logging.config")),
        
        SetEnvironmentVariable(name="ROS_HOSTNAME", condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration("mlp"), "local")),
                               value=LaunchConfiguration("mlp")),


        DeclareLaunchArgument("drivers",),                    # Start platform drivers
        DeclareLaunchArgument("spurn", default_value=""),     # Prevent a specific node
        DeclareLaunchArgument("nodes", default_value=""),     # Launch specific nodes
        DeclareLaunchArgument("extra", default_value=""),     # Inject an additional node
        DeclareLaunchArgument("debug", default_value=""),     # Debug node group
        DeclareLaunchArgument("dds",   default_value="true"), # Enable DDS

        DeclareLaunchArgument("output",  default_value="log"),    # Output to screen or log
        DeclareLaunchArgument("gtloc",   default_value="false"),  # Use Ground Truth Localizer

        ComposableNodeContainer(
        name='mlp_localization',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration("gtloc")),
        composable_node_descriptions=[
            ComposableNode(
                 package='localization_manager',
                 plugin='localization_manager::LocalizationManagerComponent',
                 name='localization_manager',
                 parameters=[{'use_sim_time': True}],
                extra_arguments=[{'use_intra_process_comms': False}]),
            ComposableNode(
                package='ground_truth_localizer',
                plugin='ground_truth_localizer::GroundTruthLocalizerComponent',
                name='ground_truth_localizer',
                parameters=[{'use_sim_time': True}],
                extra_arguments=[{'use_intra_process_comms': False}]),
            # ComposableNode(
            #     package='image_sampler',
            #     plugin='image_sampler::ImageSampler',
            #     name='image_sampler',
            #     remappings=[('/image', '/burgerimage')],
            #     parameters=[{'history': 'keep_last'}],
            #     extra_arguments=[{'use_intra_process_comms': True}])
            ],
            output=LaunchConfiguration("output")
        ),
        ComposableNodeContainer(
        name='mlp_localization',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=UnlessCondition(LaunchConfiguration("gtloc")),
        composable_node_descriptions=[
            ComposableNode(
                package='localization_manager',
                plugin='localization_manager::LocalizationManagerNodelet',
                name='localization_manager',
                parameters=[{'use_sim_time': True}],
                extra_arguments=[{'use_intra_process_comms': False}]),
            # ComposableNode(
            #     package='image_sampler',
            #     plugin='image_sampler::ImageSampler',
            #     name='image_sampler',
            #     remappings=[('/image', '/burgerimage')],
            #     parameters=[{'history': 'keep_last'}],
            #     extra_arguments=[{'use_intra_process_comms': True}])
            ],
        output=LaunchConfiguration("output")    
        ),
        
# TEMPORAL, DELETE THIS ONE AND RESTORE ALL OF THE MLP_TAKE_OUT_FOR_NOW
        ComposableNodeContainer(
        name='mlp_mobility',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='choreographer',
                plugin='choreographer::ChoreographerComponent',
                name='choreographer',
                parameters=[{'use_sim_time': True}],                
                extra_arguments=[{'use_intra_process_comms': False}]),
            ComposableNode(
                package='planner_trapezoidal',
                plugin='planner_trapezoidal::PlannerTrapezoidalComponent',
                name='planner_trapezoidal',
                parameters=[{'use_sim_time': True}],                
                extra_arguments=[{'use_intra_process_comms': False}]),
            ComposableNode(
                package='framestore',
                plugin='mobility::FrameStore',
                name='framestore',
                parameters=[{'use_sim_time': True}],                
                extra_arguments=[{'use_intra_process_comms': False}]),
            ]
        ),        


    ])

