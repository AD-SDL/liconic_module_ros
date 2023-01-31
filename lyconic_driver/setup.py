from setuptools import setup, find_packages


package_name = "lyconic_driver"

setup(
    name="lyconic_driver",
    version="0.0.1",
    packages=find_packages(),
    package_data={package_name: ["dotnet/*.dll", "dotnet/*.xml"]},
    install_requires=["pythonnet"],
    zip_safe=True,
    python_requires=">=3.8",
    maintainer="Rafael Vescovi",
    maintainer_email="ravescovi@anl.gov",
    description="",
    url="https://github.com/AD-SDL/lyconic_module/lyconic_driver",
    license="MIT License",
    entry_points={},
)
