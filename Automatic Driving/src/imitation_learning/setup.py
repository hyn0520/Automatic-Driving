from setuptools import setup

package_name = 'imitation_learning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
    		      'rclpy',                   # ROS2 Python  client library
        	      'opencv-python',           # OpenCV Library
        	      'torch',                   # PyTorch Library
        	      'torchvision',             # PyTorch visual tools
        	      'cv_bridge',               # ROS2 Image conversion library
        	      'sensor_msgs',             # ROS2 Sensor message library
        	      'std_msgs'  ],             # ROS2 Standard message library,
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 Node for road sign detection',
    license='Your license declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imitation_learning = imitation_learning.imitation_learning:main',  # Entry Point to the main function in the script
        ],
    },
)
