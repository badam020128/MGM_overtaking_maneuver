from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'overtaking_maneuver'

def package_files(directory: str):
    """Return list of package-relative file paths under `directory`.

    `directory` is an absolute path. Returned paths are relative to the package
    root so setuptools can include them correctly.
    """
    files = []
    for path in glob(os.path.join(directory, '*')):
        files.append(os.path.relpath(path, start=os.path.dirname(__file__)))
    return files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # Install package manifest and package resources (launch, rviz, etc.)
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), package_files(os.path.join(os.path.dirname(__file__), 'launch'))),
        (os.path.join('share', package_name, 'rviz'), ['rviz/gyak9.rviz']),
        (os.path.join('share', package_name, 'world'), package_files(os.path.join(os.path.dirname(__file__), 'world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adam',
    maintainer_email='badam020128@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_move = overtaking_maneuver.simple_move:main',
        ],
    },
)
