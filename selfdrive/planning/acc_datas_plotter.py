import matplotlib.pyplot as plt

# 데이터 파일을 읽어오기
x_data, y_data, z_data, w_data = [], [], [], []

# pose_data.txt 파일 열기
with open("/home/jourmain/mobinha/selfdrive/planning/pose_data.txt", "r") as f:
    # 첫 번째 라인(헤더)은 건너뜁니다.
    next(f)
    for line in f:
        # 데이터를 쉼표(,)로 분리하여 각 리스트에 저장
        x, y, z, w = map(float, line.strip().split(","))
        x_data.append(x)
        y_data.append(y)
        z_data.append(z)
        w_data.append(w)

# 그래프 그리기
plt.figure(figsize=(10, 6))

# 각 데이터에 대해 별도의 그래프 그리기
plt.plot(x_data, label='target s')
plt.plot(y_data, label='current s')
plt.plot(z_data, label='target v')
plt.plot(w_data, label='current v')

# 그래프 설정
plt.title("Pose Data Plot")
plt.xlabel("Data Point")
plt.ylabel("Value")
plt.legend()
plt.grid(True)

# 그래프 출력
plt.show()
