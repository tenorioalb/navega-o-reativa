from setuptools import setup
import os
from glob import glob

package_name = 'bug_algorithms_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # The line below is the correct way to add world files
        # It's better to add the world files to the 'my_worlds' package
        # as we discussed previously, not here.
        # This package should contain only the launch files and code.
        # Let's remove this line for now.
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seu_nome',
    maintainer_email='seu_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bug1_node = bug_algorithms_pkg.bug1_node:main',
            'tangent_bug_node = bug_algorithms_pkg.tangent_bug_node:main', # <--- ADICIONE ESTA LINHA
        ],
    },
)