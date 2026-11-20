from pathlib import Path

from setuptools import setup

package_name = 'nav2_doctor'

setup(
    name=package_name,
    version='0.6.1',
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
    long_description=Path(__file__).with_name('README.md').read_text(encoding='utf-8'),
    long_description_content_type='text/markdown',
    url='https://github.com/darshmenon/nav2_doctor',
    python_requires='>=3.8',
    classifiers=[
        'Framework :: Robot Framework',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
        'Topic :: Software Development :: Debuggers',
    ],
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['nav2_doctor = nav2_doctor.doctor:main']},
)
