from setuptools import find_packages, setup

package_name = 'nodo_missione'

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
    maintainer='tommaso',
    maintainer_email='tommaso.leonardi@studenti.unicam.it',
    description='Nodo di missione: gestione waypoint e routine di scansione',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigatore_missione = nodo_missione.navigatore_missione:main',
        ],
    },
)
