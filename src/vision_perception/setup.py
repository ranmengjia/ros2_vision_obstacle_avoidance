from setuptools import setup

package_name = 'vision_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
        ('share/' + package_name + '/param', ['param/perception.yaml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],

    zip_safe=True,
    maintainer='ranmengjia',
    maintainer_email='2700411138@qq.com',
    description='单目 RGB 可通行区域感知与避障决策节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'perception_node = vision_perception.perception_node:main',
        ],
    },
)