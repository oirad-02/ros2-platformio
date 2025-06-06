from setuptools import find_packages, setup

package_name = 'esp32_comm_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='snowy',
    maintainer_email='muellerdario02@gmail.com',
    description='ESP32 Serial Communication Package in Python',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_communicator=esp32_comm_python.esp32_comm_python:main',
        ],
    },
)
