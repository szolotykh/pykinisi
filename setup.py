from setuptools import setup
from setuptools import find_packages

setup(name='pykinisi',
      version='0.0.1',
      description='Python package for Kinisi Controller',
      url='https://github.com/szolotykh/pykinisi',
      author='Sergey Zolotykh',
      author_email='szolotykh88@gmail.com',
      license='MIT',
      zip_safe=False,
      packages=find_packages(),
      install_requires=['serial'],
      keywords=['motor controller', 'hardware', 'robotics', 'kinisi', 'kinisi controller'],
      classifiers=['Development Status :: 3 - Alpha',
                  'Programming Language :: Python :: 3',
                  'License :: OSI Approved :: MIT License',
                  'Operating System :: OS Independent'
                   ]
      )