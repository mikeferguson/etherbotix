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
            'get_trace = etherbotix_python.tools.get_trace:main',
            'monitor_etherbotix = etherbotix_python.tools.monitor_etherbotix:main',
            'read_etherbotix = etherbotix_python.tools.read_etherbotix:main',
            'read_servo = etherbotix_python.tools.read_servo:main',
            'read_unique_id = etherbotix_python.tools.read_unique_id:main',
            'reboot = etherbotix_python.tools.reboot:main',
            'upload = etherbotix_python.tools.upload:main',
        ],
    },
)
