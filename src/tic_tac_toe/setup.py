import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tic_tac_toe'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('tic_tac_toe/config/*.yaml')),
        (os.path.join('share', package_name, 'templates'), glob('tic_tac_toe/templates/*.html')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agilex',
    maintainer_email='1506289682@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_board_state = tic_tac_toe.get_board_state:main',
            'compute_best_move = tic_tac_toe.compute_best_move:main',
            'arm_control = tic_tac_toe.arm_controller:main',
            'web_control = tic_tac_toe.web_controller:main',
        ],
    },
)
