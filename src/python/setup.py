from setuptools import setup, find_packages

setup(
    name='redboard',
    version='0.1.0',
    description='Python library to drive the RedBoard+ motor controller',
    classifiers=['Programming Language :: Python :: 3.7'],
    packages=find_packages(),
    install_requires=['smbus2','luma.oled']
)