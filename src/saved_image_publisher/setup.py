from setuptools import find_packages, setup

package_name = 'saved_image_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'cv_bridge', 'opencv-python', 'sensor_msgs'],
 
   zip_safe=True,
    maintainer='Oguzhan Demiroz',
    maintainer_email='oguzhan.demiroez@fau.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'image_sender = saved_image_publisher.image_sender:main'
    ],
},
)
