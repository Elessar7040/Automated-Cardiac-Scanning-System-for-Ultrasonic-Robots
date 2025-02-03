from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ur_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
        *[(os.path.join('share', package_name, os.path.dirname(f)), [f])
          for f in glob('meshes/**/**/*', recursive=True) if os.path.isfile(f)],
        (os.path.join('share', package_name, 'world'), glob('world/**')),
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
        ],
    },
)
