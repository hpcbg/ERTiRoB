from glob import glob
from setuptools import find_packages, setup

package_name = 'board_webots_sim'
data_files = []
data_files.append(('share/ament_index/resource_index/packages',
                  ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch',
                  glob('launch/*', recursive=True)))
data_files.append(('share/' + package_name + '/worlds',
                  glob('worlds/*', recursive=True)))
data_files.append(('share/' + package_name + '/protos',
                  glob('protos/*', recursive=True)))
data_files.append(('share/' + package_name + '/controllers/panda_controller',
                  glob('controllers/panda_controller/*', recursive=True)))
data_files.append(('share/' + package_name + '/controllers/panda_player',
                  glob('controllers/panda_player/*', recursive=True)))
data_files.append(('share/' + package_name + '/resource',
                  ['resource/task_board.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_board_driver = board_webots_sim.task_board_driver:main'
        ],
    },
)
