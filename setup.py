from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'square_node'

setup(
    name=package_name,
    
    version='0.0.0',
    packages=find_packages(include=['tt_code']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tóth Tamás',
    maintainer_email='tomika9868@gmail.com',
    description='Package that makes turtlesim draw a square',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
     'console_scripts': [
        'square_node = tt_code.square_node:main',
    ],
},
)