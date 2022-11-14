from glob import glob
from setuptools import setup
import os

package_name = 'riptide_SNIB'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hayden',
    maintainer_email='hayden@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'SNIB = riptide_SNIB.SNIB:main',
        ],
    },
    py_modules=[
        'riptide_SNIB.simulinkControl',
        'riptide_SNIB.simulinkDataVisuals',
        'riptide_SNIB.gazeboControl',
    ]
)

