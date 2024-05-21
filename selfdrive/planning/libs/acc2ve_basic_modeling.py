# import numpy as np
# import matplotlib.pyplot as plt

# # Define the time range
# t = np.linspace(0, 20, 400)

# # Define the parabolic acceleration function (opens downwards)
# a = -50 * (t - 10)**2 + 5000

# # Integrate the acceleration to get the velocity
# # Initial velocity is assumed to be zero
# v = np.cumsum(a) * (t[1] - t[0])

# # Plotting the acceleration vs time
# plt.figure(figsize=(12, 6))

# plt.subplot(2, 1, 1)
# plt.plot(t, a, label='Acceleration', color='blue')
# plt.title('Acceleration vs Time')
# plt.xlabel('Time (s)')
# plt.ylabel('Acceleration (m/s^2)')
# plt.grid(True)
# plt.legend()

# # Plotting the velocity vs time
# plt.subplot(2, 1, 2)
# plt.plot(t, v, label='Velocity', color='red')
# plt.title('Velocity vs Time')
# plt.xlabel('Time (s)')
# plt.ylabel('Velocity (m/s)')
# plt.grid(True)
# plt.legend()

# plt.tight_layout()
# plt.savefig('/workspace/mobiniq_songdo/src/mobiniq/selfdrive/planning/libs/basic_big.jpg',dpi = 300)
# plt.show()




import numpy as np
import matplotlib.pyplot as plt

def sigmoid_logit_function(s, mu, v):
    if s <= 0:
        return 0
    elif s >= 1:
        return 1
    else:
        return ((1 + ((s * (1 - mu)) / (mu * (1 - s))) ** -v) ** -1).real

s_values = np.linspace(-0.5, 1.5, 400)
mu_values = [0.3, 0.5, 0.7]
mu_values = [0.5]
v_values = [0.2, 0.5, 1, 1.5, 5, 10]
# v_values = [0.5]

plt.figure(figsize=(12, 8))

for mu in mu_values:
    for v in v_values:
        out_values = [sigmoid_logit_function(s, mu, v) for s in s_values]
        plt.plot(s_values, out_values, label=f'μ={mu}, v={v}')

plt.xlabel('s')
plt.ylabel('out')
plt.title('Sigmoid Logit Function')
plt.legend()
plt.grid(True)
plt.show()
