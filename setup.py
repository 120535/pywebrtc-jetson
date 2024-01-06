from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pywebrtc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob('launch/*.launch.py')),
        ('lib/python3.8/site-packages/pywebrtc', glob('pywebrtc/*.html')),
        ('lib/python3.8/site-packages/pywebrtc', glob('pywebrtc/*.js')),
        # ('lib/python3.8/site-packages/pywebrtc', glob('pywebrtc/*.py')),        # shouldn't this already work?
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pgaston',
    maintainer_email='peter.gaston@gmail.com',
    description='webRTC for video publishing based on ros2 images.   Also a data channel.',
    license='Apache License 2.0',

    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'websrvr = pywebrtc.pywebrtc:main',
            'testpubimages = pywebrtc.testpubimages:main',

        ],
    },
)
