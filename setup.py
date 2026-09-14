# Filename: setup.py
# Description: Package metadata using the same SDK version sent in INIT.
from pathlib import Path
from runpy import run_path
from setuptools import setup

from setuptools import find_packages

setup(name='pykinisi',
      version=run_path(str(Path(__file__).parent / 'pykinisi' / '_version.py'))['__version__'],
      description='Python package for Kinisi Controller',
      url='https://github.com/szolotykh/pykinisi',
      author='Sergey Zolotykh',
      author_email='szolotykh88@gmail.com',
      license='MIT',
      zip_safe=False,
      packages=find_packages(),
      install_requires=['pyserial'],
      python_requires='>=3.8',
      keywords=['motor controller', 'hardware', 'robotics', 'kinisi', 'kinisi controller'],
      classifiers=['Development Status :: 3 - Alpha',
                  'Programming Language :: Python :: 3',
                  'License :: OSI Approved :: MIT License',
                  'Operating System :: OS Independent'
                   ],
      long_description=open('README.md').read(),
      long_description_content_type='text/markdown'
      )
