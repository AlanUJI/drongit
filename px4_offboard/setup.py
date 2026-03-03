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
            'aterrizaje = px4_offboard.aterrizaje_vertical_justo_debajo:main',
            'returnth = px4_offboard.return_to_home:main',
            'ocho_suave = px4_offboard.trayecto_ocho_suave:main',
            'voxl_offboard_figure8 = px4_offboard.voxl_offboard_figure8:main',
            'despegue = px4_offboard.despegue_normal:main',
            'despegue_y_return_home = px4_offboard.despegue_y_return_home:main',
            'despegue_aterrizar = px4_offboard.despegue_y_aterrizar:main',
            'first_mission = px4_offboard.first_mission:main',
            'second_mission = px4_offboard.second_mission:main',
            'third_mission = px4_offboard.third_mission:main',
            'fourth_mission = px4_offboard.fourth_mission:main',
        ],
    },
)
