from setuptools import setup
import os
from glob import glob

package_name = 'robotsatyo_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # install URDFs
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),

        # install RViz configs
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),

        #  install worlds
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),

        # package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Satyo',
    maintainer_email='satyo@example.com',
    description='Your robot package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
