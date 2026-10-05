import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 1    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 20.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'right_lidar' # frame_id in published point cloud message, e.g., right_lidar, left_lidar, etc.
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    livox_rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_config_path]
        )

    # 静态变换: base_footprint -> left_lidar (159 雷达)
    # 位置取自 151 安装位关于 base_footprint 的 y = 0 平面的精确镜像:
    #   (x, -y, z), 来自机械图纸 (两个安装位 y = -/+178.97 mm, 间距 357.94 mm)。
    # 姿态以 151 的镜像旋转 (M R M, 即 rpy -> (-roll, pitch, -yaw)) 为基准,
    # 再叠加两项实测修正:
    #   +3.500 度 yaw —— 装配存在真实的相对偏航误差; 用纯镜像时约 3 m 处的重影会回来。
    #   绕 base x 轴 -1.000 度 —— 159 点云在 +y (左) 侧翘起。绕车体 x 轴旋转在 3 m 处
    #     只带来 0.8 cm 的侧向位移, 因此几乎不影响重影。若过冲, 先改回 -0.500 度试:
    #     对应 roll -0.644959, pitch 0.947872, yaw 1.058817。
    # 两项修正都只改姿态, 位置必须保持不动。直接改 rpy 里的 roll/pitch 数值会绕雷达自身
    # 的轴旋转 (3 m 处分别偏移 4.2 / 2.6 cm); 而绕 151 的位置旋转 (曾试过一次) 会把该
    # 安装位横向拖动约 30 mm, 丢掉机械位置。
    left_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_lidar_broadcaster',
        arguments=[
            '--x', '-0.14820', '--y', '0.17897', '--z', '0.43251',
            '--roll', '-0.652366', '--pitch', '0.955467', '--yaw', '1.052785',
            '--frame-id', 'base_footprint', '--child-frame-id', 'left_lidar'
        ]
    )

    # 静态变换: base_footprint -> right_lidar (151 雷达)
    # x / y / z 取自机械安装图纸: x = -148.20 mm, y = -178.97 mm,
    # z = 432.51 mm (离地高度, 量至雷达光学中心)。
    # roll / pitch 由 GX_test_show.pcd 的地面平面标定得到。该点云在 right_lidar 系下
    # 地面法向为 n = [-0.807707, 0.351012, 0.473709]; 解 R @ n = e_z 可将其精确摆平
    # (残差 0.00003 度), 这也独立印证了上面的机械值。
    # yaw 仍是假设值: 地面平面无法约束绕竖直轴的旋转。
    right_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_lidar_broadcaster',
        arguments=[
            '--x', '-0.14820', '--y', '-0.17897', '--z', '0.43251',
            '--roll', '0.637708', '--pitch', '0.940252', '--yaw', '-1.0036',
            '--frame-id', 'base_footprint', '--child-frame-id', 'right_lidar'
        ]
    )

    # FAST-LIO extrinsic is defined as T_imu_lidar, so publish the inverse here:
    # right_lidar -> imu_link
    right_lidar_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_lidar_to_imu_broadcaster',
        arguments=[
            '--x', '0.011', '--y', '0.02329', '--z', '-0.04412',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'right_lidar', '--child-frame-id', 'imu_link'
        ]
    )

    return LaunchDescription([
        livox_driver,
        #livox_rviz,
        right_lidar_tf,
        left_lidar_tf,
        right_lidar_to_imu_tf,
    ])
