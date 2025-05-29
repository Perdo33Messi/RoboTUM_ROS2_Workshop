from setuptools import setup

package_name = 'learning_node'

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
    maintainer='Pedro',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'node_hellorobotum       = learning_node.node_hellorobotum:main', # learning_node 包的名字，node_hellorobotum 文件的名字
         'node_hellorobotum_class = learning_node.node_hellorobotum_class:main',
        ],
    },
)
