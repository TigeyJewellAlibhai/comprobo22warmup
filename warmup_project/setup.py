from setuptools import setup

package_name = 'warmup_project'

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
    maintainer='tigris',
    maintainer_email='tigris@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_message = warmup_project.send_message:main',
            'teleop = warmup_project.teleop:main',
            'wall_follower = warmup_project.wall_follower:main',
            'person_follower = warmup_project.person_follower:main',
            'person_follower_basedonwallfollowerfancy = warmup_project.person_follower_basedonwallfollowerfancy:main',
            'drive_square = warmup_project.drive_square:main',
            'obstacle_avoider = warmup_project.obstacle_avoider:main'
        ],
    },
)
