from glob import glob
import os

from setuptools import setup

package_name = 'thr_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='th12-dv',
    maintainer_email='weichang0307@gmail.com',
    description='Simple bicycle-model vehicle simulator and sim control for ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vehicle_sim = thr_sim.vehicle_sim_node:main',
            'sim_ctrl = thr_sim.sim_ctrl_node:main',
        ],
    },
)
