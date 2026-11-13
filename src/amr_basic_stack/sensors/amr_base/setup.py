from setuptools import setup

package_name = "amr_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "basicmicro"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="AMR Team",
    maintainer_email="amr@example.com",
    description="Basic mecanum base driver for two RoboClaw motor controllers.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "roboclaw_mecanum_node = amr_base.roboclaw_mecanum_node:main",
            "joy_teleop_node = amr_base.joy_teleop_node:main",
            "list_serial_ports = amr_base.list_serial_ports:main",
            "velocity_step_test = amr_base.velocity_step_test:main",
            "roboclaw_probe = amr_base.roboclaw_probe:main",
        ],
    },
)
