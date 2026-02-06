from setuptools import setup

package_name = "racer_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/fortress_sim.launch.py",
            "launch/fortress_teleop_rviz.launch.py",
        ]),
        ("share/" + package_name + "/config", ["config/rviz2_lidar.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robert Kastaman",
    maintainer_email="robertkastaman@ucsb.edu",
    description="ROS 2 bringup and launch files for Gazebo Fortress simulation.",
    license="MIT",
)
