from setuptools import find_packages, setup

package_name = 'uav_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['numpy', 'scipy', 'setuptools'],
    zip_safe=True,
    maintainer='qin',
    maintainer_email='email@example.com',
    description='UAV-USV simulation, guidance and offboard control nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'follow_mpc_shadow_node = '
            'uav_control.controllers.follow_mpc_shadow_node:main',
            'position_listener = uav_control.position_listener:main',
            'offboard_takeoff = uav_control.offboard_takeoff:main',
            'moving_target = uav_control.moving_target:main',
            'usv_estimation_analysis = '
            'uav_control.analysis.usv_estimation_analysis:main',
            'intercept_outcome_analysis = '
            'uav_control.analysis.intercept_outcome_analysis:main',
            'target_kalman_filter = '
            'uav_control.tracking.target_kalman_filter:main',
            'target_predictor_node = '
            'uav_control.tracking.target_predictor_node:main',
            'intercept_planner_node = '
            'uav_control.guidance.intercept_planner_node:main',
            'trajectory_tracker_node = '
            'uav_control.control.trajectory_tracker_node:main',
            'mission_manager_node = '
            'uav_control.mission.mission_manager_node:main',
            'intercept_evaluator_node = '
            'uav_control.evaluation.intercept_evaluator_node:main',
            'front_tof_monitor = '
            'uav_control.perception.front_tof_monitor:main',
            'target_bearing_node = '
            'uav_control.perception.target_bearing_node:main',
            'rgbd_target_localizer = '
            'uav_control.perception.rgbd_target_localizer:main',
            'tof_depth_node = uav_control.perception.tof_depth_node:main',
            'dual_tof_selector = '
            'uav_control.perception.dual_tof_selector:main',
        ],
    },
)
