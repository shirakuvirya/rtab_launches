

"""

DESCRIPTION:

Simplified RTAB-Map launch for LiDAR + IMU only SLAM using DLIO odometry and deskewed point clouds.

- Focuses on low-drift mapping with ICP loop closures.

- Configurable for mapping or localization mode.
 
DATA FLOW:

- Inputs from DLIO: Deskewed point clouds (/dlio/odom_node/pointcloud/deskewed) and odometry (/dlio/odom_node/odom).

- RTAB-Map processes these for SLAM: Uses ICP for registration, builds a graph, detects loop closures, and generates maps.

- Outputs: Optimized map (/rtabmap/cloud_map for 3D point cloud), poses, and TF transforms (map -> odom -> base_link).

- In localization mode: Loads existing map from database for pose estimation without building a new one.
 
TOPIC NAMES (Key Inputs/Outputs):

- Input: scan_cloud (remapped to /dlio/odom_node/pointcloud/deskewed) - Deskewed LiDAR points for mapping.

- Input: odom (remapped to /dlio/odom_node/odom) - Odometry poses from DLIO.

- Output: /rtabmap/cloud_map - Generated 3D point cloud map.

- Output: /rtabmap/mapData - Map graph data.

- Output: /rtabmap/odom - Optimized odometry (if published).
 
FRAME NAMES (TF Hierarchy):

- base_link: Robot's base frame (provided by DLIO).

- odom: Odometry frame (DLIO publishes odom -> base_link).

- map: Global map frame (RTAB-Map publishes map -> odom for drift correction).

"""
 
from launch import LaunchDescription, LaunchContext

from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
 
def launch_setup(context: LaunchContext, *args, **kwargs):

    use_sim_time = LaunchConfiguration('use_sim_time')

    localization = LaunchConfiguration('localization').perform(context).lower() == 'true'

    voxel_size = float(LaunchConfiguration('voxel_size').perform(context))

    max_correspondence_distance = voxel_size * 10.0

    qos = LaunchConfiguration('qos')
 
    shared_parameters = {

        # General Settings

        'use_sim_time': use_sim_time,  # Use simulation clock if true (useful for testing with recorded data).

        'qos': qos,  # QoS profile for topics (1 = reliable, adjust for network conditions in Bangalore's variable connectivity).

        'frame_id': 'base_link',  # Robot's base frame for poses and transforms.

        'odom_frame_id': 'odom',  # Odometry frame from DLIO.

        'map_frame_id': 'map',  # Global map frame published by RTAB-Map.

        'wait_for_transform': 0.2,  # Time to wait for TF transforms (reduced for faster startup).
 
        # ICP Settings (Core for LiDAR registration and loop closure)

        'Icp/VoxelSize': str(voxel_size),  # Downsampling voxel size for point clouds (larger = faster but less detail; 0.1 suits 16-channel LiDAR at 9.9Hz).

        'Icp/MaxCorrespondenceDistance': str(max_correspondence_distance),  # Max distance for point matching (scaled from voxel_size for robustness).

        'Icp/PointToPlane': 'true',  # Use point-to-plane ICP for better alignment in structured environments like urban Bangalore.

        'Icp/PointToPlaneK': '20',  # Number of nearest neighbors for plane estimation (higher = more robust but slower).

        'Icp/Epsilon': '0.001',  # Convergence threshold for ICP iterations (small value for precision).

        'Icp/Iterations': '20',  # Max ICP iterations per alignment (balanced for real-time at ~8 kmph).

        'Icp/OutlierRatio': '0.7',  # Ratio of outliers to ignore (0.7 allows some noise in cluttered areas).

        'Icp/Strategy': '1',  # ICP strategy (1 = point-to-plane, ideal for LiDAR-only).
 
        # Graph Optimization

        'Reg/Strategy': '2',  # Registration strategy (1 = ICP only, no visual features).

        'Optimizer/Strategy': '2',  # Optimizer (0 = G2O for efficient graph SLAM).
 
        # Features (For odometry and matching)

        'Kp/DetectorStrategy': '8',  # Keypoint detector (1 = SIFT, good for feature extraction if needed).

        'Vis/FeatureType': '8', 

        'Kp/MaxFeatures': '5000',  # Max keypoints per frame (limits computation for performance).

        'Mem/UseOdomFeatures': 'true',  # Use features from odometry for better matching.

        'Vis/MaxFeatures': '5000',
 
        # General Subscriptions and Sync

        'RGBD/CreateOccupancyGrid': 'false',  # Disable 2D grid creation (enable if needed for navigation; currently off for lightweight 3D mapping).

        'subscribe_scan_cloud': True,  # Subscribe to LiDAR point clouds for mapping.

        'subscribe_depth': True,  # No depth images (LiDAR-only mode).

        'subscribe_rgb': True,  # No RGB images (LiDAR-only mode).

        'subscribe_odom_info': False,  # Disabled as DLIO doesn't provide extra odom metadata.

        'odom_sensor_sync': True,  # Sync odometry with sensor data for accurate timing.

        'approx_sync': True,  # Allow approximate timestamp syncing (useful for unsynced DLIO topics).

        'queue_size': 30,  # Queue size for message buffering (increased for high-rate LiDAR at 9.9Hz).



        # Default RTAB-Map parameters
        'BRIEF/Bytes': '32',
        'BRISK/Octaves': '3',
        'BRISK/PatternScale': '1',
        'BRISK/Thresh': '30',
        'Bayes/FullPredictionUpdate': 'false',
        'Bayes/PredictionLC': '0.1 0.36 0.30 0.16 0.062 0.0151 0.00255 0.000324 2.5e-05 1.3e-06 4.8e-08 1.2e-09 1.9e-11 2.2e-13 1.7e-15 8.5e-18 2.9e-20 6.9e-23',
        'Bayes/VirtualPlacePriorThr': '0.9',
        'Db/TargetVersion': '',
        'DbSqlite3/CacheSize': '10000',
        'DbSqlite3/InMemory': 'false',
        'DbSqlite3/JournalMode': '3',
        'DbSqlite3/Synchronous': '0',
        'DbSqlite3/TempStore': '2',
        'FAST/CV': '0',
        'FAST/Gpu': 'false',
        'FAST/GpuKeypointsRatio': '0.05',
        'FAST/GridCols': '0',
        'FAST/GridRows': '0',
        'FAST/MaxThreshold': '200',
        'FAST/MinThreshold': '7',
        'FAST/NonmaxSuppression': 'true',
        'FAST/Threshold': '20',
        'FREAK/NOctaves': '4',
        'FREAK/OrientationNormalized': 'true',
        'FREAK/PatternScale': '22',
        'FREAK/ScaleNormalized': 'true',
        'GFTT/BlockSize': '3',
        'GFTT/Gpu': 'false',
        'GFTT/K': '0.04',
        'GFTT/MinDistance': '7',
        'GFTT/QualityLevel': '0.001',
        'GFTT/UseHarrisDetector': 'false',
        'GMS/ThresholdFactor': '6.0',
        'GMS/WithRotation': 'false',
        'GMS/WithScale': 'false',
        'GTSAM/IncRelinearizeSkip': '1',
        'GTSAM/IncRelinearizeThreshold': '0.01',
        'GTSAM/Incremental': 'false',
        'GTSAM/Optimizer': '1',
        'Grid/3D': 'true',
        'Grid/CellSize': '0.05',
        'Grid/ClusterRadius': '0.1',
        'Grid/DepthDecimation': '4',
        'Grid/DepthRoiRatios': '0.0 0.0 0.0 0.0',
        'Grid/FlatObstacleDetected': 'true',
        'Grid/FootprintHeight': '0.0',
        'Grid/FootprintLength': '0.0',
        'Grid/FootprintWidth': '0.0',
        'Grid/GroundIsObstacle': 'false',
        'Grid/MapFrameProjection': 'false',
        'Grid/MaxGroundAngle': '45',
        'Grid/MaxGroundHeight': '0.0',
        'Grid/MaxObstacleHeight': '0.0',
        'Grid/MinClusterSize': '10',
        'Grid/MinGroundHeight': '0.0',
        'Grid/NoiseFilteringMinNeighbors': '5',
        'Grid/NoiseFilteringRadius': '0.0',
        'Grid/NormalK': '20',
        'Grid/NormalsSegmentation': 'true',
        'Grid/PreVoxelFiltering': 'true',
        'Grid/RangeMax': '0',
        'Grid/RangeMin': '0.0',
        'Grid/RayTracing': 'false',
        'Grid/Scan2dUnknownSpaceFilled': 'false',
        'Grid/ScanDecimation': '1',
        'Grid/Sensor': '0',
        'GridGlobal/AltitudeDelta': '0',
        'GridGlobal/Eroded': 'false',
        'GridGlobal/FloodFillDepth': '0',
        'GridGlobal/FootprintRadius': '0.0',
        'GridGlobal/MaxNodes': '0',
        'GridGlobal/MinSize': '0.0',
        'GridGlobal/OccupancyThr': '0.5',
        'GridGlobal/ProbClampingMax': '0.971',
        'GridGlobal/ProbClampingMin': '0.1192',
        'GridGlobal/ProbHit': '0.7',
        'GridGlobal/ProbMiss': '0.4',
        'GridGlobal/UpdateError': '0.01',
        'Icp/CCFilterOutFarthestPoints': 'false',
        'Icp/CCMaxFinalRMS': '0.2',
        'Icp/CCSamplingLimit': '50000',
        'Icp/CorrespondenceRatio': '0.05',
        'Icp/DebugExportFormat': '',
        'Icp/DownsamplingStep': '1',
        # 'Icp/Epsilon': '0.001',
        'Icp/FiltersEnabled': '3',
        'Icp/Force4DoF': 'false',
        # 'Icp/Iterations': '10',
        # 'Icp/MaxCorrespondenceDistance': '1.0',
        'Icp/MaxRotation': '0.78',
        'Icp/MaxTranslation': '3',
        # 'Icp/OutlierRatio': '0.7',
        'Icp/PMConfig': '',
        'Icp/PMMatcherEpsilon': '0.0',
        'Icp/PMMatcherIntensity': 'false',
        'Icp/PMMatcherKnn': '1',
        # 'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneGroundNormalsUp': '0.0',
        # 'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneLowComplexityStrategy': '1',
        'Icp/PointToPlaneMinComplexity': '0.02',
        'Icp/PointToPlaneRadius': '0',
        'Icp/RangeMax': '0',
        'Icp/RangeMin': '0',
        'Icp/ReciprocalCorrespondences': 'true',
        # 'Icp/Strategy': '1',
        # 'Icp/VoxelSize': '0.1',
        'ImuFilter/ComplementaryBiasAlpha': '0.01',
        'ImuFilter/ComplementaryDoAdpativeGain': 'true',
        'ImuFilter/ComplementaryDoBiasEstimation': 'true',
        'ImuFilter/ComplementaryGainAcc': '0.01',
        'ImuFilter/MadgwickGain': '0.1',
        'ImuFilter/MadgwickZeta': '0.0',
        'KAZE/Diffusivity': '1',
        'KAZE/Extended': 'false',
        'KAZE/NOctaveLayers': '4',
        'KAZE/NOctaves': '4',
        'KAZE/Threshold': '0.001',
        'KAZE/Upright': 'false',
        'Kp/BadSignRatio': '0.5',
        'Kp/ByteToFloat': 'false',
        'Kp/DetectorStrategy': '8',
        'Kp/DictionaryPath': '',
        'Kp/FlannRebalancingFactor': '2.0',
        'Kp/GridCols': '1',
        'Kp/GridRows': '1',
        'Kp/IncrementalDictionary': 'true',
        'Kp/IncrementalFlann': 'true',
        'Kp/MaxDepth': '0',
        # 'Kp/MaxFeatures': '-1',
        'Kp/MinDepth': '0',
        'Kp/NNStrategy': '1',
        'Kp/NewWordsComparedTogether': 'true',
        'Kp/NndrRatio': '0.8',
        'Kp/Parallelized': 'true',
        'Kp/RoiRatios': '0.0 0.0 0.0 0.0',
        'Kp/SSC': 'false',
        'Kp/SubPixEps': '0.02',
        'Kp/SubPixIterations': '0',
        'Kp/SubPixWinSize': '3',
        'Kp/TfIdfLikelihoodUsed': 'true',
        'Marker/CornerRefinementMethod': '0',
        'Marker/Dictionary': '0',
        'Marker/Length': '0',
        'Marker/MaxDepthError': '0.01',
        'Marker/MaxRange': '0.0',
        'Marker/MinRange': '0.0',
        'Marker/Priors': '',
        'Marker/PriorsVarianceAngular': '0.001',
        'Marker/PriorsVarianceLinear': '0.001',
        'Marker/VarianceAngular': '0.01',
        'Marker/VarianceLinear': '0.001',
        'Marker/VarianceOrientationIgnored': 'false',
        'Mem/BadSignaturesIgnored': 'false',
        'Mem/BinDataKept': 'true',
        'Mem/CompressionParallelized': 'true',
        'Mem/CovOffDiagIgnored': 'true',
        'Mem/DepthAsMask': 'true',
        'Mem/DepthCompressionFormat': '.rvl',
        'Mem/DepthMaskFloorThr': '0.0',
        'Mem/GenerateIds': 'true',
        'Mem/GlobalDescriptorStrategy': '0',
        'Mem/ImageCompressionFormat': '.jpg',
        'Mem/ImageKept': 'false',
        'Mem/ImagePostDecimation': '1',
        'Mem/ImagePreDecimation': '1',
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        'Mem/IntermediateNodeDataKept': 'false',
        'Mem/LaserScanDownsampleStepSize': '1',
        'Mem/LaserScanNormalK': '0',
        'Mem/LaserScanNormalRadius': '0.0',
        'Mem/LaserScanVoxelSize': '0.0',
        'Mem/LocalizationDataSaved': 'false',
        'Mem/MapLabelsAdded': 'true',
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/RawDescriptorsKept': 'true',
        'Mem/RecentWmRatio': '0.2',
        'Mem/ReduceGraph': 'false',
        'Mem/RehearsalIdUpdatedToNewOne': 'false',
        'Mem/RehearsalSimilarity': '0.6',
        'Mem/RehearsalWeightIgnoredWhileMoving': 'false',
        'Mem/RotateImagesUpsideUp': 'false',
        'Mem/STMSize': '30',
        'Mem/SaveDepth16Format': 'false',
        'Mem/StereoFromMotion': 'false',
        'Mem/TransferSortingByWeightId': 'false',
        # 'Mem/UseOdomFeatures': 'true',
        'Mem/UseOdomGravity': 'false',
        'ORB/EdgeThreshold': '19',
        'ORB/FirstLevel': '0',
        'ORB/Gpu': 'false',
        'ORB/NLevels': '3',
        'ORB/PatchSize': '31',
        'ORB/ScaleFactor': '2',
        'ORB/ScoreType': '0',
        'ORB/WTA_K': '2',
        'Optimizer/Epsilon': '0.00001',
        'Optimizer/GravitySigma': '0.3',
        # 'Optimizer/Iterations': '20',
        'Optimizer/LandmarksIgnored': 'false',
        'Optimizer/PriorsIgnored': 'true',
        'Optimizer/Robust': 'false',
        # 'Optimizer/Strategy': '1',
        'Optimizer/VarianceIgnored': 'false',
        'PyDescriptor/Dim': '4096',
        'PyDescriptor/Path': '',
        'PyDetector/Cuda': 'true',
        'PyDetector/Path': '',
        'PyMatcher/Cuda': 'true',
        'PyMatcher/Iterations': '20',
        'PyMatcher/Model': 'indoor',
        'PyMatcher/Path': '',
        'PyMatcher/Threshold': '0.2',
        'RGBD/AggressiveLoopThr': '0.05',
        'RGBD/AngularSpeedUpdate': '0.0',
        'RGBD/AngularUpdate': '0.05',
        # 'RGBD/CreateOccupancyGrid': 'false',
        'RGBD/Enabled': 'true',
        'RGBD/ForceOdom3DoF': 'true',
        'RGBD/GoalReachedRadius': '0.5',
        'RGBD/GoalsSavedInUserData': 'false',
        'RGBD/InvertedReg': 'false',
        'RGBD/LinearSpeedUpdate': '0.0',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/LocalBundleOnLoopClosure': 'false',
        'RGBD/LocalImmunizationRatio': '0.25',
        'RGBD/LocalRadius': '10',
        'RGBD/LocalizationPriorError': '0.001',
        'RGBD/LocalizationSecondTryWithoutProximityLinks': 'true',
        'RGBD/LocalizationSmoothing': 'true',
        'RGBD/LoopClosureIdentityGuess': 'true',
        'RGBD/LoopClosureReextractFeatures': 'false',
        'RGBD/LoopCovLimited': 'false',
        'RGBD/MarkerDetection': 'false',
        'RGBD/MaxLocalRetrieved': '2',
        'RGBD/MaxLoopClosureDistance': '0.0',
        'RGBD/MaxOdomCacheSize': '10',
        'RGBD/NeighborLinkRefining': 'false',
        'RGBD/NewMapOdomChangeDistance': '0',
        'RGBD/OptimizeFromGraphEnd': 'true',
        'RGBD/OptimizeMaxError': '5.0',
        'RGBD/PlanAngularVelocity': '0',
        'RGBD/PlanLinearVelocity': '0',
        'RGBD/PlanStuckIterations': '0',
        'RGBD/ProximityAngle': '45',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityByTime': 'false',
        'RGBD/ProximityGlobalScanMap': 'false',
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityMaxPaths': '3',
        'RGBD/ProximityMergedScanCovFactor': '100.0',
        'RGBD/ProximityOdomGuess': 'false',
        'RGBD/ProximityPathFilteringRadius': '1',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/ProximityPathRawPosesUsed': 'true',
        'RGBD/ScanMatchingIdsSavedInLinks': 'true',
        'RGBD/StartAtOrigin': 'false',
        'Reg/Force3DoF': 'false',
        'Reg/RepeatOnce': 'true',
        # 'Reg/Strategy': '1',
        'Rtabmap/ComputeRMSE': 'true',
        'Rtabmap/CreateIntermediateNodes': 'false',
        # 'Rtabmap/DetectionRate': '1',
        'Rtabmap/ImageBufferSize': '1',
        'Rtabmap/ImagesAlreadyRectified': 'true',
        'Rtabmap/LoopGPS': 'true',
        'Rtabmap/LoopRatio': '0',
        'Rtabmap/LoopThr': '0.11',
        'Rtabmap/MaxRepublished': '2',
        'Rtabmap/MaxRetrieved': '2',
        'Rtabmap/MemoryThr': '0',
        'Rtabmap/PublishLastSignature': 'true',
        'Rtabmap/PublishLikelihood': 'true',
        'Rtabmap/PublishPdf': 'true',
        'Rtabmap/PublishRAMUsage': 'false',
        'Rtabmap/PublishStats': 'true',
        'Rtabmap/RectifyOnlyFeatures': 'false',
        'Rtabmap/SaveWMState': 'false',
        'Rtabmap/StartNewMapOnGoodSignature': 'false',
        'Rtabmap/StartNewMapOnLoopClosure': 'false',
        'Rtabmap/StatisticLogged': 'false',
        'Rtabmap/StatisticLoggedHeaders': 'true',
        'Rtabmap/StatisticLogsBufferedInRAM': 'true',
        'Rtabmap/TimeThr': '0',
        'Rtabmap/VirtualPlaceLikelihoodRatio': '0',
        'Rtabmap/WorkingDirectory': '/home/shivaram/.ros',
        'SIFT/ContrastThreshold': '0.04',
        'SIFT/EdgeThreshold': '10',
        'SIFT/GaussianThreshold': '2.0',
        'SIFT/Gpu': 'false',
        'SIFT/NOctaveLayers': '3',
        'SIFT/PreciseUpscale': 'false',
        'SIFT/RootSIFT': 'false',
        'SIFT/Sigma': '1.6',
        'SIFT/Upscale': 'false',
        'SURF/Extended': 'false',
        'SURF/GpuKeypointsRatio': '0.01',
        'SURF/GpuVersion': 'false',
        'SURF/HessianThreshold': '500',
        'SURF/OctaveLayers': '2',
        'SURF/Octaves': '4',
        'SURF/Upright': 'false',
        'Stereo/DenseStrategy': '0',
        'Stereo/Eps': '0.01',
        'Stereo/Gpu': 'false',
        'Stereo/Iterations': '30',
        'Stereo/MaxDisparity': '128.0',
        'Stereo/MaxLevel': '5',
        'Stereo/MinDisparity': '0.5',
        'Stereo/OpticalFlow': 'true',
        'Stereo/SSD': 'true',
        'Stereo/WinHeight': '3',
        'Stereo/WinWidth': '15',
        'StereoBM/BlockSize': '15',
        'StereoBM/Disp12MaxDiff': '-1',
        'StereoBM/MinDisparity': '0',
        'StereoBM/NumDisparities': '128',
        'StereoBM/PreFilterCap': '31',
        'StereoBM/PreFilterSize': '9',
        'StereoBM/SpeckleRange': '4',
        'StereoBM/SpeckleWindowSize': '100',
        'StereoBM/TextureThreshold': '10',
        'StereoBM/UniquenessRatio': '15',
        'StereoSGBM/BlockSize': '15',
        'StereoSGBM/Disp12MaxDiff': '1',
        'StereoSGBM/MinDisparity': '0',
        'StereoSGBM/Mode': '2',
        'StereoSGBM/NumDisparities': '128',
        'StereoSGBM/P1': '2',
        'StereoSGBM/P2': '5',
        'StereoSGBM/PreFilterCap': '31',
        'StereoSGBM/SpeckleRange': '4',
        'StereoSGBM/SpeckleWindowSize': '100',
        'StereoSGBM/UniquenessRatio': '20',
        'SuperPoint/Cuda': 'true',
        'SuperPoint/ModelPath': '',
        'SuperPoint/NMS': 'true',
        'SuperPoint/NMSRadius': '4',
        'SuperPoint/Threshold': '0.010',
        'VhEp/Enabled': 'false',
        'VhEp/MatchCountMin': '8',
        'VhEp/RansacParam1': '3',
        'VhEp/RansacParam2': '0.99',
        'Vis/BundleAdjustment': '1',
        'Vis/CorFlowEps': '0.01',
        'Vis/CorFlowGpu': 'false',
        'Vis/CorFlowIterations': '30',
        'Vis/CorFlowMaxLevel': '3',
        'Vis/CorFlowWinSize': '16',
        'Vis/CorGuessMatchToProjection': 'false',
        'Vis/CorGuessWinSize': '40',
        'Vis/CorNNDR': '0.8',
        'Vis/CorNNType': '1',
        'Vis/CorType': '0',
        'Vis/DepthAsMask': 'true',
        'Vis/DepthMaskFloorThr': '0.0',
        'Vis/EpipolarGeometryVar': '0.1',
        'Vis/EstimationType': '0',
        'Vis/FeatureType': '8',
        'Vis/GridCols': '1',
        'Vis/GridRows': '1',
        'Vis/InlierDistance': '0.3',
        'Vis/Iterations': '300',
        'Vis/MaxDepth': '0',
        # 'Vis/MaxFeatures': '1000',
        'Vis/MeanInliersDistance': '0.0',
        'Vis/MinDepth': '0',
        'Vis/MinInliers': '20',
        'Vis/MinInliersDistribution': '0.0',
        'Vis/PnPFlags': '0',
        'Vis/PnPMaxVariance': '0.0',
        'Vis/PnPRefineIterations': '0',
        'Vis/PnPReprojError': '5',
        'Vis/PnPSamplingPolicy': '1',
        'Vis/PnPSplitLinearCovComponents': 'false',
        'Vis/PnPVarianceMedianRatio': '4',
        'Vis/RefineIterations': '5',
        'Vis/RoiRatios': '0.0 0.0 0.0 0.0',
        'Vis/SSC': 'false',
        'Vis/SubPixEps': '0.02',
        'Vis/SubPixIterations': '0',
        'Vis/SubPixWinSize': '3',
        # 'approx_sync': 'false',
        # 'cloud_output_voxelized': 'true',
        # 'cloud_subtract_filtering': 'false',
        # 'cloud_subtract_filtering_min_neighbors': '2',
        # 'config_path': '',
        # 'database_path': '/home/shivaram/.ros/rtabmap.db',
        # 'delete_db_on_start': 'false',
        # # 'diagnostic_updater':
        # 'period': 2.0,
        # 'use_fqn': 'false',
        # 'frame_id': 'velodyne'
        # 'g2o/Baseline': '0.075'
        # 'g2o/Optimizer': '0'
        # 'g2o/PixelVariance': '1.0'
        # 'g2o/RobustKernelDelta': '8'
        # 'g2o/Solver': '0'
        # 'gen_depth': 'false'
        # 'gen_depth_decimation': 1
        # 'gen_depth_fill_holes_error': 0.1
        # 'gen_depth_fill_holes_size': 0
        # 'gen_depth_fill_iterations': 1
        # 'gen_scan': false
        # 'gen_scan_max_depth': 4.0
        # 'gen_scan_min_depth': 0.0
        # 'ground_truth_base_frame_id': 'velodyne'
        # 'ground_truth_frame_id': ''
        # 'initial_pose': ''
        # 'is_rtabmap_paused': 'false'
        # 'landmark_angular_variance': 0.001
        # 'landmark_linear_variance': 0.001
        # 'latch': 'true'
        # 'loc_thr': 0.0
        # 'log_to_rosout_level': 4
        # 'map_always_update': 'false'
        # 'map_cleanup': 'true'
        # 'map_empty_ray_tracing': 'true'
        # 'map_filter_angle': 30.0
        # 'map_filter_radius': 0.0
        # 'map_frame_id': 'new_map'
        # 'octomap_tree_depth': 16
        # 'odom_frame_id': ''
        # 'odom_frame_id_init': ''
        # 'odom_sensor_sync': 'true'
        # 'odom_tf_angular_variance': 0.001
        # 'odom_tf_linear_variance': 0.001
        # 'pub_loc_pose_only_when_localizing': 'false'
        # 'publish_tf': 'true'
        # 'qos': 1
        # 'qos_camera_info': 1
        # 'qos_gps': 0
        # 'qos_image': 1
        # 'qos_imu': 0
        # 'qos_odom': 1
        # 'qos_overrides':
        # '/parameter_events':
        #     'publisher':
        #     'depth': 1000
        #     'durability': 'volatile'
        #     'history': 'keep_last'
        #     'reliability': 'reliable'
        # '/tf':
        #     'publisher':
        #     'depth': 100
        #     'durability': 'volatile'
        #     'history': 'keep_last'
        #     'reliability': 'reliable'
        # 'qos_scan': 1
        # 'qos_sensor_data': 1
        # 'qos_user_data': 1
        # # 'queue_size': -1
        # 'rgbd_cameras': 1
        # 'scan_cloud_is_2d': 'false'
        # 'scan_cloud_max_points': 0
        # 'start_type_description_service': 'true'
        # 'stereo_to_depth': 'false'
        # 'subscribe_depth': 'false'
        # 'subscribe_odom': 'true'
        # 'subscribe_odom_info': 'true'
        # 'subscribe_rgb': 'false'
        # 'subscribe_rgbd': 'false'
        # 'subscribe_scan': 'false'
        # 'subscribe_scan_cloud': 'true'
        # 'subscribe_scan_descriptor': 'false'
        # 'subscribe_sensor_data': 'false'
        # 'subscribe_stereo': 'false'
        # 'subscribe_user_data': 'false'
        # 'sync_queue_size': 10
        # 'tf_delay': 0.05
        # 'tf_tolerance': 0.1
        # 'topic_queue_size': 10
        # 'use_action_for_goal': 'false'
        # 'use_saved_map': 'true'
        # 'use_sim_time': 'false'
        # 'wait_for_transform': 0.2

    
    }
 
    arguments = []
 
    # Mode-Specific Adjustments

    if localization:

        # Localization mode: Use existing map for pose estimation without adding new nodes.

        shared_parameters['Mem/IncrementalMemory'] = 'false'  # Disable incremental mapping.

        shared_parameters['Mem/InitWMWithAllNodes'] = 'true'  # Load full working memory from database.

    else:

        # Mapping mode: Build new map and clear previous database.

        arguments = ['-d']  # Clear previous database for fresh mapping.
 
    # Topic Remappings

    remappings = [

        ('scan_cloud', '/dlio/odom_node/pointcloud/deskewed'),  # Remap to DLIO's deskewed point cloud topic.

        ('odom', '/dlio/odom_node/odom'),  # Remap to DLIO's odometry topic.

        ('rgb/image', '/camera/camera/color/image_raw'),

        ('depth/image', '/image'),

        ('rgb/camera_info', '/camera/camera/color/camera_info')

    ]
 
    # Launch Nodes

    return [

        Node(

            package='rtabmap_slam',

            executable='rtabmap',

            output='screen',

            parameters=[shared_parameters],

            remappings=remappings,

            arguments=arguments

        ),

        Node(

            package='rtabmap_viz',

            executable='rtabmap_viz',

            output='screen',

            parameters=[shared_parameters],

            remappings=remappings

        ),

        Node(

            package='rtabmap_util',

            executable='pointcloud_to_depthimage',

            name='pointcloud_to_depthimage',

            output='screen',

            remappings=[

                ('cloud', '/dlio/odom_node/pointcloud/deskewed'),

                ('camera_info', '/camera/camera/color/camera_info')

            ],

            parameters=[{

                'fixed_frame_id': 'base_link',

                'fill_holes_size': 3,

                'fill_holes_error': 0.02

      }])

    ]
 
def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='false', description='Enable use of simulation time'),

        DeclareLaunchArgument('voxel_size', default_value='0.1', description='Voxel size for downsampling point clouds'),

        DeclareLaunchArgument('qos', default_value='1', description='QoS profile for topics (1 = reliable)'),

        DeclareLaunchArgument('localization', default_value='false', description='Run in localization-only mode (true/false)'),

        OpaqueFunction(function=launch_setup),

    ])

 