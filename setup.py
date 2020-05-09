from setuptools import setup

package_name = 'etherbotix_python'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Ferguson',
    maintainer_email='mike@vanadiumlabs.com',
    description='Drivers for etherbotix board',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor_etherbotix = etherbotix_python.monitor_etherbotix:main',
        ],
    },
)
