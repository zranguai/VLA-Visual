from glob import glob

from setuptools import find_packages, setup


package_name = "lerobot_ros2_replay"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/scripts", glob("scripts/*.sh")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="zme",
    maintainer_email="zme@example.com",
    description="Replay LeRobot v3 episodes as Lite robot ROS 2 topics.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "replay = lerobot_ros2_replay.replay_node:main",
        ],
    },
)
