from setuptools import find_packages, setup

package_name = "fleet_coordinator"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "fleet_coordinator_node = fleet_coordinator.node:main",
        ],
    },
)
