import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtTest import QTest
from PyQt5.QtCore import Qt
from visualize import MainWindow  # 가정하는 클래스 파일 이름
from std_msgs.msg import String, Float32, Int8, Int16MultiArray, Float32MultiArray



# PyQt5 앱 인스턴스 생성
app = QApplication(sys.argv)

# MainWindow 인스턴스 생성
main_window = MainWindow()
main_window.show()

# UI에 표시할 임의의 값 설정
test_velocity = 50  # km/h
target_vel = 30

test_sensor_status = Int16MultiArray()
test_sensor_status.data = [1, 1, 1, 1, 1, 1, 1, 1, 1]  

# 테스트 값을 MainWindow 인스턴스에 설정
main_window.info_cur_vel.setText(f"{test_velocity}")
main_window.setting_target_vel.setText(f'{target_vel}')
# main_window.gear_change(main_window.gear_r)
main_window.senser_check_callback(test_sensor_status)


# # 값이 UI에 정상적으로 반영되었는지 확인
# assert main_window.info_cur_vel.text() == "50"
# assert main_window.setting_target_vel.text() == "30"
# assert "background-color : #FC6C6C;" in main_window.gear_d.styleSheet()

# 앱 실행 (UI를 실제로 보고 싶다면 주석을 해제)
sys.exit(app.exec_())

# 앱 종료 (테스트를 위해 즉시 종료)
app.exit()