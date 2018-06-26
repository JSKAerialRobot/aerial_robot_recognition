#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'aerial_robot_perception'

    download_data(
        pkg_name=PKG,
        path='sample/data/2018-06-25-21-21-10_uav_with_red_object.bag',
        url='https://drive.google.com/uc?id=1chQSj09vqDXKY3niRJpXEDCfZfqCbLf6',
        md5='54a46cf71924aabbe49bcc32d8c14c0a',
        extract=False,
    )

if __name__ == '__main__':
    main()

