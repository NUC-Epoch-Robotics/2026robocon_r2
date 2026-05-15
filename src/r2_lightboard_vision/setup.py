from setuptools import setup

package_name = "r2_lightboard_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/lightboard_detector.launch.py",
                "launch/grab_scene_detector.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="r2",
    maintainer_email="you@example.com",
    description="Vision nodes for R2: lightboard, spearhead, grab-scene detection.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lightboard_detector = r2_lightboard_vision.lightboard_detector_node:main",
            "spearhead_detector = r2_lightboard_vision.spearhead_detector_node:main",
            "grab_scene_detector = r2_lightboard_vision.grab_scene_detector_node:main",
        ],
    },
)
