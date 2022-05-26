from setuptools import setup

package_name = 'heading'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autonav',
    maintainer_email='rljudy4981@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "fusion=heading.fusion:main",
            "encoder_pub=heading.encoder:main",
            "gps_publisher=heading.gps_reader:main"
        ],
    },
)
