from setuptools import find_packages, setup

package_name = 'ros_basics'

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
    maintainer='satya_ubuntu',
    maintainer_email='spradhan143as@gmail.com',
    description='Beginner client libraries tutorials practice package. Like Publisher-Subscriber, etc.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ros_basics.publisher_member_function:main',
            'listener = ros_basics.subscriber_member_function:main',
            'service = ros_basics.service_member_function:main',
            'client = ros_basics.client_member_function:main',
            'py_node = ros_basics.my_first_node:main',
            'robot_news = ros_basics.robo_news_station:main',
            'smartphone = ros_basics.smartphone:main',
            'number_publisher = ros_basics.activity_node1:main',
            'number_counter = ros_basics.activity_node2:main',
            'add_ints_server = ros_basics.add_ints_server:main',
            'add_ints_client_noop = ros_basics.add_ints_client_no_oop:main',
            'add_ints_client = ros_basics.add_ints_client:main',
            'hw_status_publisher = ros_basics.hw_status_publisher:main',
            'battery = ros_basics.battery:main',
            'led_panel = ros_basics.led:main',
        ],
    },
)
