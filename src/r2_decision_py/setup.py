from setuptools import setup

package_name = 'r2_decision_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lonelystar',
    maintainer_email='lonelystar@todo.todo',
    description='R2 decision node (Python async FSM)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r2_decision_py = r2_decision_py.node:main',
        ],
    },
)
