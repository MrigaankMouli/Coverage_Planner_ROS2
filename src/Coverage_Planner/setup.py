from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'Coverage_Planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'Waypoints'), glob('Waypoints/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zero',
    maintainer_email='sufy1707@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'Connect = Coverage_Planner.Connect:main',
            'Odometry = Coverage_Planner.OdometryPub:main',
            'CoverageMission = Coverage_Planner.Mission:main',
            'CircleWPGeneration  = Coverage_Planner.CircularCoverage:main',
            'SquareWPGeneration = Coverage_Planner.SquareCoverage:main',
            'Takeoff_Land = Coverage_Planner.Takeoff_and_Land_Test:main',
            'WaypointManual = Coverage_Planner.Waypoint:main',
            'ArmCheck = Coverage_Planner.ArmDisarm:main',
            'CubeOrangeTest = Coverage_Planner.CubeOrangeTest:main',
            'ScienceWPGeneration = Coverage_Planner.ScienceWaypointGen:main',
            'ScienceMission = Coverage_Planner.ScienceMission:main',
            'CoverageGeneration = Coverage_Planner.CoverageWP:main',
            'Disarm = Coverage_Planner.disarm:main',
            'Land = Coverage_Planner.Land:main'
        ],
    },
)
