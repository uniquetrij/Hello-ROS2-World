from setuptools import find_packages, setup

package_name = 'fun_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trijeetkr.modak',
    maintainer_email='uniquetrij@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fun_subscriber = fun_package.fun_subscriber:main',
            'fun_publisher = fun_package.fun_publisher:main',
        ],
    },
)
