from setuptools import find_packages, setup
from grpc_tools import command
from setuptools.command.build_py import build_py
import os
from glob import glob

package_name = 'aiva_ros_bridge'

class BuildWithProtos(build_py):
    def run(self):
        dir = os.path.join(os.path.dirname(os.path.realpath(__file__)))
        command.build_package_protos(dir)
        super().run()

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='oto.dusek@ysoft.com',
    description='AIVA ROS2 bridge',
    license='Closed',
    entry_points={
        'console_scripts': [
            'aiva_ros_bridge = aiva_ros_bridge.aiva_ros_bridge:main'
        ],
    },
    cmdclass={
    'build_proto_modules': command.BuildPackageProtos,
    'build_ext': BuildWithProtos,
  }
)
