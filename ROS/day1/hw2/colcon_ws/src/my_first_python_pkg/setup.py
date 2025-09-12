from setuptools import find_packages, setup

package_name = 'my_first_python_pkg'

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
    maintainer='jy',
    maintainer_email='jywoo10041004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['publisher = my_first_python_pkg.publisher:main',
        'subscriber = my_first_python_pkg.subscriber:main',
        ],
    },
)
