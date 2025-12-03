from setuptools import find_packages, setup

package_name = 'py_srvcli'

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
    maintainer='ssmh',
    maintainer_email='ssmh1@outlook.com',
    description='Map Set-Sync Script',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'map_set_sync_script = py_srvcli.map_set_sync_script:main', 
        # 1. runs an rviz subprocess to connect to aurora
        # 2. call SyncSetStcm service to set map
        # 3. publish one SyncMapRequest to /sync_map
        # 4. subscribe to /system_status and /robot_pose till spin is shutdown
        ],
    },
)
