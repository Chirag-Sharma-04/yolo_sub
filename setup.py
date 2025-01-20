from setuptools import find_packages, setup

package_name = 'yolo_sub'

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
    maintainer='jarvis',
    maintainer_email='chiragcs2004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_sub = yolo_sub.pose_sub:main',
            'object_sub = yolo_sub.object_sub:main',
            'object_sub_pub = yolo_sub.obj_sub_pub:main'
        ],
    },
)
