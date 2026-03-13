from setuptools import setup

package_name = "wheelbot_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Anders Roxenhag",
    maintainer_email="anders.roxenhag@emoco.com",
    description="Simulated differential-drive wheel bot for ROS 2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wheelbot_sim = wheelbot_sim.wheelbot_sim:main",
        ],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/start.launch.py"]),
    ],
)
