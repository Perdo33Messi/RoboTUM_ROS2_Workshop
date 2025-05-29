from setuptools import setup

package_name = 'learning_topic'

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
    maintainer_email='csf1peter@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'topic_hellorobotum_pub  = learning_topic.topic_hellorobotum_pub:main',
         'topic_hellorobotum_sub  = learning_topic.topic_hellorobotum_sub:main',
        ],
    },
)
