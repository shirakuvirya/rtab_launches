'''
Author: Shivaram Kumar Jagannathan
Description:
  Launch RTAB-Map based LiDAR SLAM with optional IMU and RGB input,
  including automatic conversion of pointcloud to depth image via rtabmap_util.

Usage Example:
  1) ros2 launch rtabmap_launch mono_lidar_rgb.launch.py

Pipeline understanding:
1. Visual feature extraction              Uses ORB + GFTT to extract features
2. Monocular -> Depth Cloud Transform     rtabmap_util to intake camera info and pcl2 from lidar
3. Odometry                               ICP, libpointmatcher
4. Registration                           VisICP -> Visual + ICP
5. Graph Optimization                     g2o
'''
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context: LaunchContext, *args, **kwargs):
  
  frame_id = LaunchConfiguration('frame_id')
  
  imu_topic = LaunchConfiguration('imu_topic')
  imu_used =  imu_topic.perform(context) != ''
  
  fill_holes_size = LaunchConfiguration('fill_holes_size')
  fill_holes_error = LaunchConfiguration('fill_holes_error')

  rgbd_image_topic = LaunchConfiguration('rgbd_image_topic')
  rgbd_images_topic = LaunchConfiguration('rgbd_images_topic')
  rgbd_image_used =  rgbd_image_topic.perform(context) != '' or rgbd_images_topic.perform(context) != ''
  rgbd_cameras = 1 if rgbd_images_topic.perform(context) != '' else 1
  
  voxel_size = LaunchConfiguration('voxel_size')
  voxel_size_value = float(voxel_size.perform(context))
  
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  lidar_topic = LaunchConfiguration('lidar_topic')
  lidar_topic_value = lidar_topic.perform(context)
  lidar_topic_deskewed = lidar_topic_value + "/deskewed"
  
  localization = LaunchConfiguration('localization').perform(context)
  localization = localization == 'true' or localization == 'True'
  
  deskewing = LaunchConfiguration('deskewing').perform(context)
  deskewing = deskewing == 'true' or deskewing == 'True'
  
  deskewing_slerp = LaunchConfiguration('deskewing_slerp').perform(context)
  deskewing_slerp = deskewing_slerp == 'true' or deskewing_slerp == 'True'
  
  fixed_frame_from_imu = False
  fixed_frame_id =  LaunchConfiguration('fixed_frame_id').perform(context)
  if not fixed_frame_id and imu_used:
    fixed_frame_from_imu = True
    fixed_frame_id = frame_id.perform(context) + "_stabilized"
  
  if not fixed_frame_id or not deskewing:
    lidar_topic_deskewed = lidar_topic
  
  # Rule of thumb:
  max_correspondence_distance = voxel_size_value * 10.0

  shared_parameters = {
    'use_sim_time': use_sim_time,
    'frame_id': frame_id,
    'qos': LaunchConfiguration('qos'),
    'approx_sync': True,
    'wait_for_transform': 0.2,

    # RTAB-Map's internal parameters are strings:
    'Icp/PointToPlane': 'true',
    'Icp/Iterations': '100',
    'Icp/VoxelSize': str(voxel_size_value),
    'Icp/Epsilon': '0.0001',
    'Icp/PointToPlaneK': '40',
    'Icp/PointToPlaneRadius': '0',
    'Icp/MaxTranslation': '7',
    'Icp/MaxCorrespondenceDistance': str(max_correspondence_distance),
    'Icp/Strategy': '1',                              # Use libpointmatcher
    'Icp/OutlierRatio': '0.7',
    'Vis/MaxDepth': '30.0',
    'Vis/MaxFeatures': '5000',
    'RGBD/CreateOccupancyGrid': 'false',              # Disable RGBD octomap generation
    'Kp/MaxFeatures': '5000',
    'RGBD/LoopClosureReextractFeatures': 'false',
    'RGBD/LoopCovLimited': 'false',
    'RGBD/Enabled':         'true',
    # 'VhEp/Enabled': 'true',                         # Enable epipolar geometry visual hypothesis generation
    'Rtabmap/LoopThr': '0.03',                      
    'Optimizer/Iterations': '100', 
    'Mem/DepthAsMask': 'true',
    'Vis/DepthAsMask': 'true',

    'Rtabmap/StatisticLogged': 'true',
    'Mem/StereoFromMotion': 'false',                 # Parameter used to derive the depth calculations from purely rgb motion
    'Vis/EpipolarGeometryVar': '0.3',
    'Optimizer/Robust': 'true',
    'Vis/MinInliers': '20',
    'Rtabmap/DetectionRate': '10',
    'Kp/DetectorStrategy': '8',                       # ORB + GFTT for visual feature extraction
    'Vis/EstimationType': '1',
    'Vis/SSC': 'true',
    'Vis/FeatureType': '8',                           # ORB + GFTT for visual feature extraction
    'Rtabmap/PublishStats': "true", 
    'Reg/Strategy': '2',                              # Registration through Visual ICP
    'Optimizer/Strategy': '1',                        # Use G2O for global optimization
    'RGBD/LoopClosureIdentityGuess': 'true',          # Starts registration assuming no transform between loop candidates
  }
  
  icp_odometry_parameters = {
    'expected_update_rate': LaunchConfiguration('expected_update_rate'),
    'deskewing': not fixed_frame_id and deskewing, # If fixed_frame_id is set, we do deskewing externally below
    'odom_frame_id': 'icp_odom',
    'guess_frame_id': fixed_frame_id,
    'deskewing_slerp': deskewing_slerp,
    # RTAB-Map's internal parameters are strings:
    'Odom/ScanKeyFrameThr': '0.4',
    'OdomF2M/ScanSubtractRadius': str(voxel_size_value),
    'OdomF2M/ScanMaxSize': '15000',
    'OdomF2M/BundleAdjustment': 'false',
    'Icp/CorrespondenceRatio': '0.1'
  }
  if imu_used:
    icp_odometry_parameters['wait_imu_to_init'] = True

  rtabmap_parameters = {
    'subscribe_depth': True,
    'subscribe_rgb': True,
    # 'subscribe_rgbd': False,
    
    'subscribe_odom_info': True,
    'subscribe_scan_cloud': True,
    'Mem/GenerateCloud': True,
    'Mem/SaveDepthData': True,
    'map_frame_id': 'new_map',
    'odom_sensor_sync': True, # This will adjust camera position based on difference between lidar and camera stamps.

    # RTAB-Map's internal parameters are strings:
    'RGBD/ProximityBySpace': 'true',      # Enable proximity detection in space
    'RGBD/ProximityMaxGraphDepth': '5',   # Allow checking nearby nodes
    'RGBD/ProximityPathMaxNeighbors': '1',
    'RGBD/AngularUpdate': '0.05',
    'RGBD/LinearUpdate': '0.05',
    'RGBD/CreateOccupancyGrid': 'false',
    'Mem/NotLinkedNodesKept': 'false',
    'Mem/RecentWmRatio': '0.7',
    'Mem/STMSize': '200',
    'Icp/CorrespondenceRatio': str(LaunchConfiguration('min_loop_closure_overlap').perform(context)),
    'publish_pose': True
  }

  arguments = []
  if localization:
    rtabmap_parameters['Mem/IncrementalMemory'] = 'False'
    rtabmap_parameters['Mem/InitWMWithAllNodes'] = 'True'
  else:
    arguments.append('-d') # This will delete the previous database (~/.ros/rtabmap.db)
  
  remappings = [
    ('odom', '/icp_odom'),
    ('rgb/image', '/camera/camera/color/image_raw'),
    ('depth/image', '/image'),
    ('rgb/camera_info', '/camera/camera/color/camera_info')
  ]

  if imu_used:
    remappings.append(('imu', LaunchConfiguration('imu_topic')))
  else:
    remappings.append(('imu', 'imu_not_used'))
  if rgbd_image_used:
    if rgbd_cameras == 1:
      remappings.append(('rgbd_image', LaunchConfiguration('rgbd_image_topic')))
    else:
      remappings.append(('rgbd_images', LaunchConfiguration('rgbd_images_topic')))
  
  nodes = [
    Node(
      package='rtabmap_odom', executable='icp_odometry', output='screen',
      parameters=[shared_parameters, icp_odometry_parameters],
      remappings=remappings + [('scan_cloud', lidar_topic_deskewed)]),
    
    Node(
      package='rtabmap_slam', executable='rtabmap', output='screen',
      parameters=[shared_parameters, rtabmap_parameters, 
                  {'subscribe_rgbd': rgbd_image_used, 
                   'rgbd_cameras': rgbd_cameras}],
      remappings=remappings + [('scan_cloud', lidar_topic_deskewed)],
      arguments=arguments), 
  
    Node(
      package='rtabmap_viz', executable='rtabmap_viz', output='screen',
      parameters=[shared_parameters, rtabmap_parameters],
      remappings=remappings + [('scan_cloud', 'odom_filtered_input_scan')]),

    Node(
      package='rtabmap_util',
      executable='pointcloud_to_depthimage',
      name='pointcloud_to_depthimage',
      output='screen',
      remappings=[
          ('cloud', lidar_topic),
          ('camera_info', '/camera/camera/color/camera_info')
      ],
      parameters=[{
          'fixed_frame_id': frame_id,
          'fill_holes_size': fill_holes_size,
          'fill_holes_error': fill_holes_error
      }])
  ]
  
  if fixed_frame_from_imu:
    # Create a stabilized base frame based on imu for lidar deskewing
    nodes.append(
      Node(
        package='rtabmap_util', executable='imu_to_tf', output='screen',
        parameters=[{
          'use_sim_time': use_sim_time,
          'fixed_frame_id': fixed_frame_id,
          'base_frame_id': frame_id,
          'wait_for_transform_duration': 0.001}],
        remappings=[('imu/data', imu_topic)]))

  if fixed_frame_id and deskewing:
    # Lidar deskewing
    nodes.append(
      Node(
        package='rtabmap_util', executable='lidar_deskewing', output='screen',
        parameters=[{
          'use_sim_time': use_sim_time,
          'fixed_frame_id': fixed_frame_id,
          'wait_for_transform': 0.2,
          'slerp': deskewing_slerp}],
        remappings=[
            ('input_cloud', lidar_topic)
        ])
    )
      
  return nodes
  
def generate_launch_description():
  return LaunchDescription([

    # Launch arguments
    DeclareLaunchArgument(
      'use_sim_time', default_value='false',
      description='Use simulated clock.'),
    
    DeclareLaunchArgument(
      'deskewing', default_value='true',
      description='Enable lidar deskewing.'),
    
    DeclareLaunchArgument(
      'frame_id', default_value='base_link',
      description='Base frame of the robot.'),
    
    DeclareLaunchArgument(
      'fixed_frame_id', default_value='',
      description='Fixed frame used for lidar deskewing. If not set, we will generate one from IMU.'),
    
    DeclareLaunchArgument(
      'localization', default_value='false',
      description='Localization mode.'),

    DeclareLaunchArgument(
      'lidar_topic', default_value='/velodyne_points',
      description='Name of the lidar PointCloud2 topic.'),

    DeclareLaunchArgument(
      'imu_topic', default_value='/imu/data',
      description='IMU topic (ignored if empty).'),
    
    DeclareLaunchArgument(
      'rgbd_image_topic', default_value='',
      description='RGBD image topic (ignored if empty). Would be the output of a rtabmap_sync\'s rgbd_sync, stereo_sync or rgb_sync node.'),
    
    DeclareLaunchArgument(
      'rgbd_images_topic', default_value='',
      description='RGBD images topic (ignored if empty, override "rgbd_image_topic" if set). Would be the output of a rtabmap_sync\'s rgbdx_sync node.'),
    
    DeclareLaunchArgument(
      'expected_update_rate', default_value='15.0',
      description='Expected lidar frame rate. Ideally, set it slightly higher than actual frame rate, like 15 Hz for 10 Hz lidar scans.'),
    
    DeclareLaunchArgument(
      'voxel_size', default_value='0.5',
      description='Voxel size (m) of the downsampled lidar point cloud. For indoor, set it between 0.1 and 0.3. For outdoor, set it to 0.5 or over.'),
    
    DeclareLaunchArgument(
      'min_loop_closure_overlap', default_value='0.20',
      description='Minimum scan overlap pourcentage to accept a loop closure.'),
    
    DeclareLaunchArgument(
      'deskewing_slerp', default_value='true',
      description='Use fast slerp interpolation between first and last stamps of the scan for deskewing. It would less accruate than requesting TF for every points, but a lot faster. Enable this if the delay of the deskewed scan is significant larger than the original scan.'),

    DeclareLaunchArgument(
      'qos', default_value='1',
      description='Quality of Service: 0=system default, 1=reliable, 2=best effort.'),
    
    DeclareLaunchArgument(
      'fill_holes_size', default_value='3',
      description='Depth image hole fill size'),
      
    DeclareLaunchArgument(
      'fill_holes_error', default_value='0.1', 
      description='Depth image hole fill error'),

    OpaqueFunction(function=launch_setup),
  ])

    