import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, ThisLaunchFileDir
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource 







def generate_launch_description():


  slam_toolbox_params_file = os.path.join(get_package_share_directory("treespotte"), 'mapper_params_online_async.yaml')

  use_sim_time = LaunchConfiguration('use_sim_time', default='false') 
  #urdf_file_name = 'treespotte.xacro'
  urdf_file_name = 'smalls.xacro'


  print("urdf_file_name : {}".format(urdf_file_name))

  urdf = os.path.join(
      get_package_share_directory('treespotte'), 
      urdf_file_name)

  controllers_file = os.path.join(
      get_package_share_directory('treespotte'), 
      'smalls_controllers.yaml')

  robot_description_content = Command(['xacro',' ', urdf])

  return LaunchDescription([ 
      DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true'),

# Old realsense driver setup
      Node(
          package='realsense2_camera',
          executable='realsense2_camera_node',
          name='realsense_t265_node',
          output='screen', 
       #   arguments=['--ros-args', '--log-level', 'debug' ],
          parameters=[{ 
                'use_sim_time': use_sim_time,
                'serial_no': '_20422110348',
                'device_type' : 't265',                 
                'camera_name' : 'T265', 
                'publish_odom_tf' : True,
                'base_frame_id' : 'base_link',  
                'odom_frame_id' : 'odom',  
          }]), 


      Node(
          package='ydlidar',
          executable='ydlidar_node',
          name='ydlidar_node',
          output='screen', 
          parameters=[{ 
               'use_sim_time': use_sim_time,
               'frame_id': 'lidar', 
               'port': '/dev/ttyUSB1', 
          }]), 


      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen', 
          parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description':Command(['xacro',' ', urdf])
          }]), 


# These 3 transforms seem to be needed for SLAM_TOOLBOX
#Node(
#    name='tf2_ros_fp_laser',
#    package='tf2_ros',
#    executable='static_transform_publisher',
#    output='screen',
#    arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'base_footprint', 'lidar'],   
#),

Node(
    name='tf2_ros_fp_map',
    package='tf2_ros',
    executable='static_transform_publisher',
    output='screen',
    #arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'base_footprint', 'map'], 
    arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'map', 'odom'], 
),


# Removed this because roboclaw_node should be publishing this transform now
Node(
    name='tf2_ros_rs_pose',
    package='tf2_ros',
    executable='static_transform_publisher',
    output='screen',
    arguments=['0', '0', '0', '0.0', '0.0', '0.0', '_pose_frame', 'base_link' ],
),




#      Node(
#          package='tf2_ros',
#          executable='static_transform_publisher',
#          name='static_transform_publisher',
#          output='screen', 
# # Also impacted by all this is the base_footprint_joint URDF node, rpy
# # When this is used, the map renders correctly but the navigation fails
# #         arguments=["0", "0", "0", "1.5708", "0", "0", "_pose_frame", "base_link"]  , 
# # When this is used, the map renders wrong but the navigation works
#          arguments=["0", "0", "0", "0", "0", "0", "_pose_frame", "base_link"]  , 
#          parameters=[{
#                'use_sim_time': use_sim_time,}], 
#
#         ), 








#      Node(
#          package='tf2_ros',
#          executable='static_transform_publisher',
#          name='b_static_transform_publisher',
#          output='screen', 
#          arguments=["0", "0", "0", "0", "0", "0", "odom", "_pose_frame"]  , 
#          parameters=[{
#                'use_sim_time': use_sim_time,}], 
#         ), 





#      Node(
#          package='tf2_ros',
#          executable='static_transform_publisher',
#          name='static_transform_publisher',
#          output='screen', 
#          arguments=["0", "0", "0", "0", "0", "0", "tracking_camera", "T265_tracking_camera"]  , 
#          parameters=[{
#                'use_sim_time': use_sim_time,}], 
# 
#          ), 





  #    Node(
  #        package='tf2_ros',
  #        executable='static_transform_publisher',
  #        name='static_transform_publisher',
  #        output='screen', 
   #       arguments=["0", "0", "0", "0", "0", "0", "tracking_camera", "ts_camera_link"]  , 
   #       parameters=[{
   #             'use_sim_time': use_sim_time,}], 
# 
#          ), 



  #    Node(
  #        package='tf2_ros',
  #        executable='static_transform_publisher',
  #        name='static_transform_publisher',
  #        output='screen', 
  #        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]  , 
  #        parameters=[{
  #              'use_sim_time': use_sim_time,}], 
  #        ), 



   
  # SLAM
     Node(
        parameters=[
          slam_toolbox_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen') ,




  #  Node(
  #      parameters=[
  #        controllers_file,
  #        {'robot_description': robot_description_content },
  #        {'use_sim_time': use_sim_time}
  #      ],
  #      package='controller_manager',
  #      executable='ros2_control_node',
  #      name='ros2_controll_node', 
  #      output='screen') ,


  

    Node(
          
        package='roboclaw_ros',
        namespace='roboclaw_ros',
        executable='roboclaw_node_exec',
        name='roboclaw_ros',
        parameters=[
          {"~dev":  "/dev/ttyUSB0" ,
           "~topic_odom_out":  "/wheel_odom", 
           "~publish_odom_tf":  False, 
           "~max_speed":  1.0, 
           "~ticks_per_meter":  10200, 
           "~base_width":  0.235, 
           'use_sim_time': use_sim_time, 
           
           


  } 
        ], 
        arguments=['--ros-args', '--log-level', "warn"  ]
        ),



  



  ])
