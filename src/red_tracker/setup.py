from setuptools import setup, find_packages
import os
from glob import glob

package_name = "red_tracker"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament resource index
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),

        # package.xml
        ("share/" + package_name, ["package.xml"]),

        # launch files
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.py")),

        # red box model
        (os.path.join("share", package_name, "models", "red_box"),
            glob("models/red_box/*")),

        # cartographer config (if you add a custom .lua later)
        # (os.path.join("share", package_name, "config"),
        #     glob("config/*.lua")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="parth",
    maintainer_email="parth@example.com",
    description="Autonomous red box finder with SLAM mapping",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # legacy nodes (kept for reference)
            "red_tracker     = red_tracker.red_tracker_node:main",
            "decision_node   = red_tracker.decision_node:main",
            "safety_filter   = red_tracker.safety_filter:main",
            
            # frontier exploration
            "frontier_explorer = red_tracker.frontier_explorer:main",
            "obstacle_mover  = red_tracker.obstacle_mover:main",
            "maze_solver     = red_tracker.maze_solver:main",
        ],
    },
)
