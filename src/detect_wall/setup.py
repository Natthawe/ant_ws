from setuptools import setup
import os
from glob import glob

package_name = 'detect_wall'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='icgame',
    maintainer_email='natthawejumjai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_wall_node = detect_wall.detect_wall:main',
            'detect_ex = detect_wall.detect_ex:main',
        ],
    },
)
