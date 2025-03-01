from setuptools import setup

package_name = 'road_sign_detector'

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
    		      'rclpy',                   
        	      'opencv-python',           
        	      'torch',                   
        	      'torchvision',             
        	      'cv_bridge',               
        	      'sensor_msgs',             
        	      'std_msgs'  ],               
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 Node for road sign detection',
    license='Your license declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'road_sign_detector = road_sign_detector.road_sign_detector:main',  
        ],
    },
)
