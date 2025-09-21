from setuptools import setup

package_name = 'go2_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lprotche',
    maintainer_email='ladadulina@gmail.com',
    description='Go2 demo nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_driver = go2_demos.square_driver:main',
        ],
    },
)
