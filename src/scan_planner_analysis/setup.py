from setuptools import find_packages, setup


package_name = "scan_planner_analysis"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="wei",
    maintainer_email="wei@example.com",
    description="Structured telemetry recording and offline plotting for SCAN-Planner.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "telemetry_recorder = scan_planner_analysis.telemetry_recorder:main",
            "plot_velocity = scan_planner_analysis.plot_velocity:main",
        ],
    },
)
