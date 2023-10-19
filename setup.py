from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'move_to_goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('lib/' + package_name, [package_name+'/python_file_name.py']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuko',
    maintainer_email='samuel.c.agba@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'executable_name = package_name.python_file_name:main'
            'move_to_goal = move_to_goal.move_to_goal:main',
            'move_to_goal2 = move_to_goal.move_to_goal2:main',
            'stop_motion = move_to_goal.stop_motion:main',
            'send_pose_cmd = move_to_goal.send_pose_cmd:main',
        ],
    },
)
