from setuptools import setup

package_name = 'in_class_day03'

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
    maintainer='lilo',
    maintainer_email='lheinrich@olin.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emergency_stop = in_class_day03.emergency_stop:main',
            'bump_stop = in_class_day03.bump_stop:main',
            'distance_emergency_stop = in_class_day03.distance_emergency_stop:main'
        ],
    },
)
