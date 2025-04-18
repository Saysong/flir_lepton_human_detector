from setuptools import setup
from glob import glob
import os

package_name = 'flir_lepton_human_detector'


setup(
    name=package_name,
    version='0.0.1',
    packages=package_name,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],

    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Jiyun Won',
    maintainer_email='jwon75@gatech.edu',
    description='Human detection from FLIR Lepton camera using threshold and contour filtering.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'threshold_detector = flir_lepton_human_detector.threshold_contour_detector_node:main',
        ],
    },
)
