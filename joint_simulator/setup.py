from setuptools import find_packages, setup

package_name = 'joint_simulator'

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
    maintainer='orjan',
    maintainer_email='orjan.t.lynnes@ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_simulator_node = joint_simulator.joint_simulator_node:main',
        ],
    },
)
