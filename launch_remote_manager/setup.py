from setuptools import find_packages, setup

package_name = 'launch_remote_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/test.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nick Morales',
    maintainer_email='ngmorales97@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launch_server = launch_remote_manager.launch_server:entry',
            'launch_client = launch_remote_manager.launch_client:entry',
            'copy_install_space = launch_remote_manager.copy_install_space:entry',
        ],
    },
)
