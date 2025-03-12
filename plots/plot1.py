import pandas as pd
import chardet as ch
import matplotlib.pyplot as plt
import matplotlib

matplotlib.use('TkAgg')

path = "acq5.csv"

# Detect the encoding of the file
with open(path, 'rb') as file:
    result = ch.detect(file.read())
data = pd.read_csv(path, encoding=result['encoding'])
# Convertir l'axe de temps des millisecondes en secondes
data["time"] = data["time"] / 1000
# plt.figure()
# plt.plot(data["time"], data['yaw_master'], label='yaw_master')
# plt.plot(data["time"], data['pitch_master'], label='pitch_master')
# plt.plot(data["time"], data['roll_master'], label='roll_master')
# plt.plot(data["time"], data['speed'], label='speed')
# plt.legend()
# plt.title("Dérive des angles d'Euler en fonction du temps")
# plt.xlabel("Temps (s)")
# plt.ylabel("Angles d'Euler (°)")


# plt.figure()
# plt.plot(data["time"], data['yaw_diff'], label='yaw_diff')
# plt.plot(data["time"], data['pitch_diff'], label='pitch_diff')
# plt.plot(data["time"], data['roll_diff'], label='roll_diff')
# plt.legend()
# plt.title("Différence des angles d'Euler en fonction du temps")
# plt.xlabel("Temps (s)")
# plt.ylabel("Angles d'Euler (°)")

plt.figure()
# plt.plot(data["time"], data['yaw_master'], label='yaw_master')
# plt.plot(data["time"], data['yaw_slave'], label='yaw_slave')
plt.plot(data["time"], data['yaw_diff'], label='yaw_diff')
plt.legend()

plt.show()
