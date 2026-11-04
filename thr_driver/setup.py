from glob import glob
import os

from setuptools import setup

package_name = 'thr_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='th12-dv',
    maintainer_email='weichang0307@gmail.com',
    description='Demo P-control driver for the THR simple vehicle simulator',
    license='MIT',
    entry_points={
        'console_scripts': [
            'driver = thr_driver.driver_node:main',
        ],
    },
)
