from setuptools import setup

package_name = 'axcend_focus_device_config'

setup(
    name=package_name,
    version='3.1.6',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='graham.harrison@axcendcorp.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'device_config = axcend_focus_device_config.device_config:main'
        ],
    },
)
