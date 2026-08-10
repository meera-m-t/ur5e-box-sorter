from setuptools import find_packages, setup

package_name = 'ur_box_sorter'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sameerah',
    maintainer_email='sameerah.talafha@siu.edu',
    description='Command the Panda arm from Python',
    license='MIT',
    entry_points={
      'console_scripts': [
            'sort_boxes = ur_box_sorter.sort_boxes:main',
            'wave_ur = ur_box_sorter.wave_ur:main',
            'measure_boxes = ur_box_sorter.measure_boxes:main',
        ],
    },
)


