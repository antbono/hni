from setuptools import find_packages, setup

package_name = 'hni_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antbono',
    maintainer_email='bnontn89@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chat_client = hni_py.chat_client:main',
            'chat_service = hni_py.chat_service:main',
            'g2stt_service = hni_py.g2stt_service:main',
            'gstt_client = hni_py.gstt_client:main',
            'gstt_service = hni_py.gstt_service:main',
            'gtts_client = hni_py.gtts_client:main',
            'gtts_service = hni_py.gtts_service:main',
            'video_track_client = hni_py.video_track_client:main',
            'yolo_video_track_server = hni_py.yolo_video_track_server:main',
        ],
    },
    
)
