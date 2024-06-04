from setuptools import setup

package_name = 'axcend_focus_legacy_compatibility_layer'

setup(
    name=package_name,
    version='3.1.4',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'flask', 'flask_cors', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='graham.harrison@axcendcorp.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'legacy_compatibility_interface = axcend_focus_legacy_compatibility_layer.legacy_compatibility_interface:main'
        ],
    },
)
