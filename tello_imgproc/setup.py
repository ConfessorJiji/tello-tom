from setuptools import setup

package_name = 'tello_imgproc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ben',
    maintainer_email='benevans07@googlemail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'imgtest = tello_imgproc.imgtest:main', 'adj_hsv_filter = tello_imgproc.adj_hsv_filter:main', 'img_proc = tello_imgproc.img_proc:main'
        ],
    },
)

