from setuptools import setup

package_name = 'pubsub'

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
    maintainer='fjord',
    maintainer_email='fjord@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker_bb = pubsub.pub_bb:main',
            'talker_pc = pubsub.pub_pc:main',
            'bb = pubsub.bb_sub:main',
            'marker = pubsub.marker_sub:main',
            'bbimg = pubsub.bb_corner_sub:main',
            'liimg = pubsub.rsli_sub:main',
            'state = pubsub.state_sub:main',
            'image = pubsub.image_lidar_sub:main',
        ],
    },
)
