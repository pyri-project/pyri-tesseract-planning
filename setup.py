from setuptools import setup, find_packages, find_namespace_packages

setup(
    name='pyri-tesseract-planner',
    version='0.1.0',
    description='PyRI Teach Pendant Tesseract Planner Plugin',
    author='John Wason',
    author_email='wason@wasontech.com',
    url='http://pyri.tech',
    package_dir={'': 'src'},
    packages=find_namespace_packages(where='src'),
    include_package_data=True,
    package_data = {
        'pyri.tesseract_planner.tesseract_planner_service': ['*.robdef','*.yml'],
    },
    zip_safe=False,
    install_requires=[
        'pyri-common',
        'pyri-robotics'
    ],
    tests_require=['pytest','pytest-asyncio'],
    extras_require={
        'test': ['pytest','pytest-asyncio']
    },
    entry_points = {
        #'pyri.plugins.sandbox_functions': ['pyri-robotics-sandbox-functions=pyri.robotics.sandbox_functions:get_sandbox_functions_factory'],
        'pyri.plugins.device_type_adapter': ['pyri-robotics-type-adapter = pyri.tesseract_planner.device_type_adapter:get_device_type_adapter_factory'],
        #'pyri.plugins.blockly': ['pyri-robotics-plugin-blockly=pyri.robotics.blockly:get_blockly_factory'],
        'console_scripts': ['pyri-tesseract-planner-service = pyri.tesseract_planner.tesseract_planner_service.__main__:main'],
        #'pyri.plugins.service_node_launch': ['pyri-robotics-launch = pyri.robotics.service_node_launch:get_service_node_launch_factory']
    }
)