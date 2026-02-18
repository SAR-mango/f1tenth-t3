from setuptools import setup

package_name = "wallfollowing2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robert Kastaman",
    maintainer_email="robertkastaman@ucsb.edu",
    description="ROS 2 port of the wallfollowing2 lidar algorithm.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "wallfollowing_node = wallfollowing2.wallfollowing_node:main",
            "wallbalancing_node = wallfollowing2.wallbalancing_node:main",
            "scan_stop_reverse_test_node = wallfollowing2.scan_stop_reverse_test_node:main",
        ],
    },
)
