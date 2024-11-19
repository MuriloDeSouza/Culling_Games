from setuptools import find_packages, setup

package_name = 'mov'

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
    maintainer='inteli',
    maintainer_email='murilo.silva@sou.inteli.edu.br',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "busca_largura = mov.busca_largura:main",
            "busca_profundidade = mov.busca_profundidade:main",
            "busca_mapa = mov.busca_mapa:main",
        ],
    },
)
