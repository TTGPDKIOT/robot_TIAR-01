#!/usr/bin/env python3

import rospy
import requests  # Import thư viện requests để thực hiện HTTP requests
from requests_toolbelt.multipart.encoder import MultipartEncoder   #pip install requests-toolbelt
from std_msgs.msg import String

def send_files():
    # Đường dẫn tới file PGM, YAML, và PNG trên máy của bạn
    pgm_file_path = '/home/iot3/catkin_ws/indoor.pgm'
    yaml_file_path = '/home/iot3/catkin_ws/indoor.yaml'
    png_file_path = '/home/iot3/catkin_ws/dog.png'

    # URL của API
    api_url = "http://14.241.244.228:3021/api/iotGuest/Map"

    multipart_data = MultipartEncoder(
    fields={
        # plain text fields
        'name': 'Bản đồ heheeee',
        'file1': ('indoor.pgm', open(pgm_file_path, 'rb'), 'text/plain'),
        'file2': ('dog.png', open(png_file_path, 'rb'), 'image/png'),
        'file3': ('indoor.yaml', open(yaml_file_path, 'rb'), 'text/plain'),
        'coordinate': '1',
        'negate': '1',
        'occupied': '1',
        'freeThreshold': '1',
        'description': '1',
        'xMin': '1',
        'xMax': '1',
        'yMin': '1',
        'yMin': '1',

    }
    )

        # Gửi POST request với cả ba file được đính kèm
    response = requests.post(api_url,
                         data=multipart_data,
                         headers={'Content-Type': multipart_data.content_type}, timeout=30)

    # Kiểm tra phản hồi từ server
    if response.status_code == 200 or 201:
        rospy.loginfo('Cả ba file đã được gửi thành công.')
    else:
        rospy.logerr(f'Có lỗi xảy ra. Mã trạng thái: {response.status_code}')
        rospy.logerr(response.text)

def api_request_node():
    rospy.init_node('api_node', anonymous=True)
    send_files()
    

if __name__ == '__main__':
    try:
        api_request_node()
    except rospy.ROSInterruptException:
        pass