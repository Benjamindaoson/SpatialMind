from setuptools import setup
from glob import glob
from os.path import join

package_name="spatialmind_ros"
setup(
    name=package_name,version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",["resource/"+package_name]),
        ("share/"+package_name,["package.xml"]),
        (join("share",package_name,"launch"),glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SpatialMind contributors",
    maintainer_email="research@texasaiinstitute.org",
    description="Nav2 and camera interface for SpatialMind",
    license="MIT",
    entry_points={"console_scripts":[
        "spatialmind_nav_agent=spatialmind_ros.mission_runner:main",
        "spatialmind_rgbd_bridge=spatialmind_ros.perception_bridge:main",
    ]},
)