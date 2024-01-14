from setuptools import setup

package_name = 'tello_control'

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
    maintainer='wallnuts',
    maintainer_email='wallnuts@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ibvs_pid = tello_control.ibvs_pid_node:main',
            'ibvs_ismc = tello_control.ibvs_ismc_node:main',
        ],
    },
)
