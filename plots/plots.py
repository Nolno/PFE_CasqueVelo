import pandas as pd
import chardet as ch
import matplotlib.pyplot as plt
import matplotlib

matplotlib.use('TkAgg')
# Detect the encoding of the file
with open("data_log.csv", 'rb') as file:
    result = ch.detect(file.read())
data = pd.read_csv("data_log.csv", encoding=result['encoding'])
print(data)
plt.figure()
plt.plot(range(data.shape[0]), data['x_angle'], label='x_angle')
plt.plot(range(data.shape[0]), data['y_angle'], label='y_angle')
plt.plot(range(data.shape[0]), data['z_angle'], label='z_angle')
plt.plot(range(data.shape[0]), data['x_raw_angle'], label='x_row_angle')
plt.plot(range(data.shape[0]), data['y_raw_angle'], label='y_row_angle')
plt.plot(range(data.shape[0]), data['z_raw_angle'], label='z_row_angle')
plt.plot(range(data.shape[0]), data['x_total_deviations'], label='x_total_deviations')
plt.plot(range(data.shape[0]), data['y_total_deviations'], label='y_total_deviations')
plt.plot(range(data.shape[0]), data['z_total_deviations'], label='z_total_deviations')
plt.legend()
plt.show()

