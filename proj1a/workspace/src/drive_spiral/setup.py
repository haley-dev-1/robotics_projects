from setuptools import setup

package_name = 'drive_spiral'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO: Group N (names here)',
    maintainer_email='TODO@example.com',
    description='Group project: Drive Spiral behavior for iRobot Create 3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Run with: ros2 run drive_spiral main
            'main = drive_spiral.main:main',
        ],
    },
)

