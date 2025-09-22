from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ai_labs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jd',
    maintainer_email='chenjingdao@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square = ai_labs.square:main',
            'square_solution = ai_labs.square_solution:main',
            'subscribe_pose = ai_labs.subscribe_pose:main',
            'initials = ai_labs.initials:main',
        ],
    },
)
