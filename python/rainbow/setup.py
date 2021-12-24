import setuptools

with open("ReadME.md", "rt") as fh:
    long_description = fh.read()

setuptools.setup(
    name="rb-api-python",
    version="0.0.3",
    author="Hansol Kang, Hyojoon Lee",
    author_email="hskang@rainbow-robotics.com, hyojoon_lee04@rainbow-robotics.com",
    description="Python wrapper for rarinbow robotics cobot",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/messy-snail/rb-api/python",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.7',
)