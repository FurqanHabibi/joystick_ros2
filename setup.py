from setuptools import setup

package_name = 'joystick_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
        'joystick_ros2',
        'inputs'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('shared/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    maintainer='Muhammad Furqan Habibi',
    maintainer_email='furqan.habibi1@gmail.com',
    author='Muhammad Furqan Habibi',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A cross-platform joystick drivers node.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'joystick_ros2 = joystick_ros2:main'
        ],
    },
)
