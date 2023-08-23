import os

from setuptools import setup
from glob import glob

package_name = 'assignment7'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lorenzo Busellato',
    maintainer_email='lorenzo.busellato_02@studenti.univr.it',
    description='Robotics, Vision and Control @ UniVR, assignment 7',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = assignment7.main:main'
        ],
    },
)
