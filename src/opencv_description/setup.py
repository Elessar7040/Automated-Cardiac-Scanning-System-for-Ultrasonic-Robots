from setuptools import find_packages, setup

package_name = 'opencv_description'

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
    maintainer_email='elessar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "opencv_node = opencv_description.opencv_test:main",
            "pointcloud_node = opencv_description.pointcloud2_test:main",
            "ik_node = opencv_description.ikine_test_2:main",
            "joint_trajectory_node = opencv_description.joint_trajectory_test_1:main",
        ],
    },
)