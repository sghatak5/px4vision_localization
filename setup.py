from setuptools import find_packages, setup

package_name = 'px4vision_localization'

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
    maintainer='sghatak5',
    maintainer_email='sagnikghatak22@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'px4vision_sensordata = px4vision_localization.px4vision_sensordata:main',
            'ekf = px4vision_localization.ekf:main',
            'createPlot = px4vision_localization.createPlot:main',
            'px4visionEKFLocalization = px4vision_localization.localization:main',
        ],
    },
)
