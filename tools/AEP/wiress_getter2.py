import requests
import time
import rospy

from std_msgs.msg import Float32MultiArray

url = 'https://Mobinha.access.ly:5000/get'
url_reset = 'https://Mobinha.access.ly:5000/reset'

id_list = ['kana', 'heesang', 'jaejun', 'junmyeong']
state_list = {'wait':0, 'call':1, 'dest1':2, 'dest2':3, 'dest3':4, 'go':5}

rospy.init_node('get_wiress', anonymous=True)
gosilsil_publisher = rospy.Publisher("/gosilsil", Float32MultiArray, queue_size=1)

response = requests.post(url_reset, verify=False)
if response.status_code == 200:
    print('Data store has been reset.')
else:
    print('Failed to reset data store. Status code:', response.status_code)

dest_positions = [(35.6477372,128.3995581), (35.64925367,128.39812751), (35.6468156,128.4013399)] #KIAPI


while True:
    response = requests.get(url, verify=False)
    if response.status_code == 200:
        data = response.json()
        for key,value in data.items():
            if key in id_list:
                state = value['state']
                if state in state_list:
                    state_value = int(state_list.get(state))
                    if state_value == 2 or state_value == 3 or state_value == 4:
                        position_value = dest_positions[state_value-2]
                    else:
                        position_value = (float(value['position']['latitude']), float(value['position']['longitude']))
                    print(f'user {key} push {state} at position {position_value}')

                    float32multiarray = Float32MultiArray()
                    float32multiarray.data = [state_value, position_value[0], position_value[1]]
                    gosilsil_publisher.publish(float32multiarray)
    else:
        print('Failed to retrieve data. Status code:', response.status_code)
    time.sleep(0.5)