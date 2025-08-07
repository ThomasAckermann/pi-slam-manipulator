from setuptools import find_packages, setup

package_name = "image_receiver"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/receiver.launch.py"]),
        ("share/" + package_name + "/config", ["config/receiver_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Thomas Ackermann",
    maintainer_email="ackermann.th@outlook.com",
    description="Image receiver package for SLAM robotic arm project",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "image_receiver = image_receiver.image_receiver_node:main",
        ],
    },
)
