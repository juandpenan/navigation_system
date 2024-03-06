from setuptools import find_packages
from setuptools import setup

package_name = 'navigation_system_cli'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Juan Carlos Manzanares Serrano',
    author_email='jc.manzanares.serrano@gmail.com',
    maintainer='Juan Carlos Manzanares Serrano',
    maintainer_email='jc.manzanares.serrano@gmail.com',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='Command line tools for Navigation System.',
    long_description="""\
The package provides the navigation_system command as a plugin for Navigation System.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'set = navigation_system_cli.command.set:SetCommand',
            'save = navigation_system_cli.command.save:SaveCommand',
        ],
        'ros2cli.extension_point': [
            'set.verb = navigation_system_cli.verb:VerbExtension',
            'save.verb = navigation_system_cli.verb:VerbExtension',
        ],
        'set.verb': [
            'map = navigation_system_cli.verb.map:SetMapVerb',
            'pose = navigation_system_cli.verb.pose:SetPoseVerb',
            'mode = navigation_system_cli.verb.mode:SetModeVerb',
        ],
        'save.verb': [
            'map = navigation_system_cli.verb.map:SaveMapVerb',
        ],
    }
)
