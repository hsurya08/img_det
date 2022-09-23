from setuptools import setup

package_name = 'img_det'

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
    maintainer='hemanth',
    maintainer_email='hemanth@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'ros_detect = img_det.ros_detect:main',
                'ball_teleop= img_det.ball_teleop:main',
                'coord_teleop=img_det.coord_teleop:main',
        ],
    },
)
