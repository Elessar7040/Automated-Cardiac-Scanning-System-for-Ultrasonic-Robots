from setuptools import find_packages, setup

package_name = 'yolo_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elessar',
    maintainer_email='1142097478@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector_node = yolo_node.detect_ros2:main',
            'yolo_detector_node_2 = yolo_node.detect_ros2_2:main',
            'yolo_test_node = yolo_node.detect_ros2_test:main',
            'yolo_test_node_2 = yolo_node.detect_ros2_test_2:main',
        ],
    },
)
