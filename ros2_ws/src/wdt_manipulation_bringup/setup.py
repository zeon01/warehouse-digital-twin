"""ament_python setup for wdt_manipulation_bringup.

Exposes the ``pick_cell_orchestrator`` console script so it's launchable
with ``ros2 run wdt_manipulation_bringup pick_cell_orchestrator``.
"""

from setuptools import find_packages, setup

package_name = "wdt_manipulation_bringup"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            ["launch/move_group.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Saad Sharif Ahmed",
    maintainer_email="zeon01@users.noreply.github.com",
    description="MoveIt2 + pick_cell_orchestrator bringup.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pick_cell_orchestrator = " "wdt_manipulation_bringup.pick_cell_orchestrator:main",
        ],
    },
)
