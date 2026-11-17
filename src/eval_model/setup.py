from glob import glob

from setuptools import find_packages, setup


package_name = "eval_model"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="zme",
    maintainer_email="zme@example.com",
    description="Evaluate replayed expert actions against model actions on ROS 2.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["evaluate = eval_model.node:main"]},
)
