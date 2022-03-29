import setuptools
from setuptools import find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    packages=find_packages(
        where='src',
        include=['dpsim*'],
    ),
    package_dir={"": "src"},
    python_requires=">=3.6",
)
