from setuptools import setup
import os
from glob import glob

package_name = 'opencv_tools'

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
            'img_publisher = opencv_tools.image_publisher:main',
            'img_subscriber = opencv_tools.image_subscriber:main',
            'streaming_camera_cv = opencv_tools.streaming_camera_cv:main',
        ],
    },
)
