from setuptools import setup

package_name = "axcend_focus_ros2_firmware_bridge"

setup(
    name=package_name,
    version="3.1.6",
    packages=[package_name],
    package_data={
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    extras_require={
        "debug": ["debugpy"],
    },
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="graham.harrison@axcendcorp.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest", "debugpy"],
    entry_points={
        "console_scripts": [
            "firmware_bridge = axcend_focus_ros2_firmware_bridge.firmware_bridge:main"
        ],
    },
)
