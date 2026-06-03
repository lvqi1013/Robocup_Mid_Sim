from setuptools import setup

package_name = "nubot_hwcontroller"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", ["launch/hwcontroller.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NuBot simulation maintainer",
    maintainer_email="todo@example.com",
    description="Minimal ROS 2 Python hardware controller adapter for NuBot simulation migration.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "hwcontroller_node = nubot_hwcontroller.hwcontroller_node:main",
        ],
    },
)
