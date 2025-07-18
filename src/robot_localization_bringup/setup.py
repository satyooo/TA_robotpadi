import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_localization_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # --- INI BAGIAN PALING PENTING ---
    # Memberi tahu colcon cara meng-install file launch dan config
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install semua file launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install semua file config
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='satyo',
    maintainer_email='satyo@todo.todo',
    description='Package to launch robot_localization EKF',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # Paket ini tidak punya node Python, jadi entry_points kosong
    entry_points={
        'console_scripts': [],
    },
)

# from setuptools import find_packages, setup

# package_name = 'robot_localization_bringup'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='satyo03',
#     maintainer_email='satyosecond03@gmail.com',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )
