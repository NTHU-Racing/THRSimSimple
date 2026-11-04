from setuptools import setup

setup(
    name='demo_driver',
    version='0.1.0',
    packages=['demo_driver'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/demo_driver']),
        ('share/demo_driver', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='THR team',
    maintainer_email='weichang0307@gmail.com',
    description='Minimal driverless-control teaching example for THRSimSimple',
    license='MIT',
    entry_points={'console_scripts': ['driver = demo_driver.driver_node:main']},
)
