from setuptools import find_packages, setup

package_name = 'proj_etape_1'

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
    maintainer='rosdev',
    maintainer_email='simondumesny@gmail.com',
    description='Stratégies de navigation et localisation​ des robots mobiles',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odometrie = odometrie.odometrie:main',
            'pid = PID.pid:main',
            'suivi_trajectoire = suivi_trajectoire.suivi_trajectoire:main',
             'evitement_obstacle = evitement_obstacle.evitement_obstacle:main',
        ],
    },
)


