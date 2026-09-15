import os
from glob import glob        
from setuptools import find_packages, setup

package_name = 'gazebo_simulation'

def get_data_files():
    data = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [f for f in glob('launch/*launch.py') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'worlds'), [f for f in glob('worlds/*') if os.path.isfile(f)]),
    ]
    for root_dir, _, files in os.walk('maps'):
        valid_files = [os.path.join(root_dir, f) for f in files if os.path.isfile(os.path.join(root_dir, f))]
        if valid_files:
            target_dir = os.path.join('share', package_name, root_dir)
            data.append((target_dir, valid_files))
    return data

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
