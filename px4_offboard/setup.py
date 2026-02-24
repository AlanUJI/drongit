from setuptools import find_packages, setup

package_name = 'px4_offboard'

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
    maintainer='alanportatil2',
    maintainer_email='al415508@uji.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'control = px4_offboard.offboard_control:main',
            'ocho = px4_offboard.trayecto_ocho:main',
            'cuadrado = px4_offboard.waypoints_cuadrado:main',
            'aterrizaje = px4_offboard.aterrizaje:main',
            'ocho_suave = px4_offboard.trayecto_ocho_suave:main',
        ],
    },
)
