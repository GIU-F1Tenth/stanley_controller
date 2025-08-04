from setuptools import find_packages, setup

package_name = 'stanley_controller'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/stanley_controller_launch.py']),
        ('share/' + package_name + '/config', ['config/stanley_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fam Shihata',
    maintainer_email='fam@awadlouis.com',
    description='Stanley controller for F1Tenth autonomous racing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stanley_controller_node = stanley_controller.stanley_controller_node:main',
        ],
    },
)
