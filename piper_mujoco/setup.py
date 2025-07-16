from setuptools import find_packages, setup
from glob import glob

package_name = 'piper_mujoco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/agilex_piper', glob('agilex_piper/*.xml')),
        ('share/' + package_name + '/agilex_piper/assets', glob('agilex_piper/assets/*')),
        ('share/' + package_name, ['environment.xml']),
        ('share/' + package_name + '/object', glob('object/*.xml')),
        ('share/' + package_name + '/object', glob('object/*.STL')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='min',
    maintainer_email='min@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'piper_interface = piper_mujoco.piper_interface:main',
        ],
    },
)
