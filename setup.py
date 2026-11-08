from setuptools import setup

package_name = 'nav2_doctor'

setup(
    name=package_name,
    version='0.2.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darshmenon',
    maintainer_email='darshmenon02@gmail.com',
    description='Attach to a running Nav2 stack and say why the robot is not moving.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['nav2_doctor = nav2_doctor.doctor:main']},
)
