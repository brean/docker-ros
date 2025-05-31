from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


package_name = 'subscribing'
BASE_PATH = Path(__file__).parent

with open(BASE_PATH / 'README.md', 'r', encoding='utf-8') as fh:
    long_description = fh.read()


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('README.md', ['README.md'])
    ],
    install_requires=['setuptools', 'fastapi'],
    zip_safe=True,
    author="Andreas Bresser",
    description="Subscribing to data",
    long_description=long_description,
    long_description_content_type="markdown",
    license="BSD-3-Clause",
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscribing = subscribing.main:main',
        ],
    },
)
