from setuptools import find_packages, setup

package_name = 'my_pkg_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeyadcode',
    maintainer_email='zeyadcode@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "py_test = my_pkg_py.my_node:main",
            "robot_news_station = my_pkg_py.robot_news_station:main",
            "smartphone = my_pkg_py.smartphone:main",
            "num_publisher = my_pkg_py.num_publisher:main",
            "num_counter = my_pkg_py.num_counter:main",
            "add_two_ints_server = my_pkg_py.add_two_ints_server:main",
            "add_two_ints_client_no_oop = my_pkg_py.add_two_ints_client_no_oop:main",
            "add_two_ints_client = my_pkg_py.add_two_ints_client:main",
            "hw_status_publisher = my_pkg_py.hw_status_publisher:main",
            "compute_area_server = my_pkg_py.compute_area_server:main",
            "battery_node = my_pkg_py.battery_node:main",
            "led_node = my_pkg_py.led_node:main",
        ],
    },
)
