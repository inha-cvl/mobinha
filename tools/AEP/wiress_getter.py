import requests
import time
import rospy

from std_msgs.msg import Int8

url = 'https://Mobinha.access.ly:5000/get'
url_reset = 'https://Mobinha.access.ly:5000/reset'

id_list = ['kana', 'heesang']
state_list = {'wait':0, 'call':1, 'dest1':2, 'dest2':3, 'dest3':4, 'go':5}

rospy.init_node('get_wiress', anonymous=True)
pub_call_button = rospy.Publisher('/gosilsil_call_button', Int8,queue_size = 1)

response = requests.post(url_reset, verify=False)
if response.status_code == 200:
    print('Data store has been reset.')
else:
    print('Failed to reset data store. Status code:', response.status_code)

while True:
    response = requests.get(url, verify=False)
    if response.status_code == 200:
        data = response.json()
        for key,value in data.items():
            if key in id_list:
                state = value['state']
                if state in state_list:
                    button = int(state_list.get(state))
                    print(f'user {key} push {state}')
                    pub_call_button.publish(Int8(button))
    else:
        print('Failed to retrieve data. Status code:', response.status_code)
    time.sleep(1)