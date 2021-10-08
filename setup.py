from setuptools import setup

package_name = 'rosgraph_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ipa-hsd',
    maintainer_email='harshavardhan.deshpande@ipa.fraunhofer.de',
    description='ROS graph monitor',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'observer = rosgraph_monitor.observer:main',
            'graph_observer = rosgraph_monitor.observers.graph_observer:main'
        ],
    },
)
