from setuptools import find_packages, setup

package_name = 'guardian_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['guardian_control/models/yolo11n.engine']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='genozen',
    maintainer_email='nathanbiomedeng@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_to_serial_node = guardian_control.joy_to_serial_node:main',
            'guardian_nav_node = guardian_control.guardian_nav_node:main',
            'fire_detector = guardian_control.camera_detect_node:main',
        ],
    },
)
