from setuptools import setup

package_name = 'kuka_rsi_simulator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Lars Tingelstad',
    author_email='lars.tingelstad@ntnu.no',
    maintainer='Lars Tingelstad',
    maintainer_email='lars.tingelstad@ntnu.no',
    keywords=['ROS', 'KUKA', 'RSI'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='KUKA RSI Simulator',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kuka_rsi_simulator = kuka_rsi_simulator.kuka_rsi_simulator:main',
            'joint_state_publisher = kuka_rsi_simulator.joint_state_publisher:main',
        ],
    },
)
