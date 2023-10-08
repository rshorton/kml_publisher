from setuptools import setup

package_name = 'kml_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shorton',
    maintainer_email='horton.rscotti@gmail.com',
    description='Publish robot position as KML for display in Google Earth',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = kml_publisher.kml_publisher:main',
        ],
    },
)
