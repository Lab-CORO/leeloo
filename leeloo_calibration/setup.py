from setuptools import find_packages, setup
from glob import glob

package_name = 'leeloo_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='guillaume.dupoiron@protonmail.com',
    description='Noeud de calibration automatique main-oeil pour Leeloo.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_calibration_server = '
            'leeloo_calibration.auto_calibration_server:main',
            'auto_calibration_server_v2 = leeloo_calibration.auto_calibration_server_v2:main',
            'pose_saver_node = leeloo_calibration.pose_saver_node:main',
        ],
    },
)