from setuptools import setup

package_name = "r2_lightboard_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/lightboard_detector.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="r2",
    maintainer_email="you@example.com",
    description="Ultra-light 3x4 lightboard detector node.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lightboard_detector = r2_lightboard_vision.lightboard_detector_node:main",
        ],
    },
)
