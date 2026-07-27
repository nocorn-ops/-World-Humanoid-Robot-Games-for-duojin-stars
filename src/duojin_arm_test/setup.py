from setuptools import find_packages, setup


package_name = "duojin_arm_test"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Duojin Team",
    maintainer_email="duojin@example.com",
    description="Safe Cartesian motion-path validation for the Galaxea R1 Lite arm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "move_arm_to_pose = duojin_arm_test.move_arm_to_pose:main",
        ],
    },
)
