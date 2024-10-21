from setuptools import setup

package_name = 'axcend_focus_operation'

setup(
    name=package_name,
    version='3.1.8',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', "scipy", "numpy", "pandas"],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='graham.harrison@axcendcorp.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'operation = axcend_focus_operation.operation:main'
        ],
    },
)
