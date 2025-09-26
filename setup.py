from setuptools import find_packages, setup

package_name = 'centroid_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IRON CREEPER',
    maintainer_email='developer-contact@ironcreeper.com',
    description='Continously moving forward autonomous navigation using only lidar /scan data',
    license='Proprietary - All rights reserved',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'centroid_nav = centroid_nav.centroid_nav:main',
            'navigation = centroid_nav.navigation:main',
            'follow_point = centroid_nav.follow_point:main',
            'graph = centroid_nav.graph:main',
        ],
    },
)
