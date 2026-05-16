"""ament_python setup for wdt_pure_pursuit.

Exposes ``pure_pursuit_driver`` as a console script for
``ros2 run wdt_pure_pursuit pure_pursuit_driver``. Post-build symlink in
the bootstrap docs handles the setuptools>=68 / ament-convention split
(``bin/`` vs ``lib/<pkg>/``); see feedback-foundationpose-install-gotchas
gotcha #15.
"""

from setuptools import find_packages, setup

package_name = "wdt_pure_pursuit"

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
            ["launch/single_amr.launch.py", "launch/multi_amr.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Saad Sharif Ahmed",
    maintainer_email="zeon01@users.noreply.github.com",
    description="Pure-pursuit NavigateToPose action server.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pure_pursuit_driver = wdt_pure_pursuit.driver:main",
        ],
    },
)
