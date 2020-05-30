from setuptools import setup, find_packages

setup(
    name='redboard',
    version='0.1.0',
    author='Tom Oinn',
    author_email='tomoinn@gmail.com',
    url='https://github.com/ApproxEng/RedBoard',
    description='Python library to drive the RedBoard+ motor controller',
    classifiers=['Programming Language :: Python :: 3.6'],
    packages=find_packages(),
    install_requires=['smbus2', 'luma.oled', 'pigpio', 'future-fstrings', 'pyyaml']
)
