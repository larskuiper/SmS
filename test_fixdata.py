#%%
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

#%%
# Path to the CSV file
csv_file_path = 'test_data.csv'

# Read the data from the CSV file
df = pd.read_csv(csv_file_path)

# Convert 'time' from milliseconds to seconds for plotting
df['time_seconds'] = df['time'] / 1000

def kalman_filter(data, Q:float, R:float, P:float, x0)->np.ndarray:
    """_summary_

    Args:
        data (_type_): pendulum data
        Q (_type_): process variance
        R (_type_): measurement variance
        P (_type_): estimation error

    Returns:
        _type_: filtered data
    """
    n = len(data)
    x = np.zeros(n)
    P = np.zeros(n)
    x[0] = x0
    P[0] = 0
    for i in range(1, n):
        # time update
        x[i] = x[i-1]
        P[i] = P[i-1] + Q
        # measurement update
        K = P[i] / (P[i] + R)
        x[i] = x[i] + K * (data[i] - x[i])
        P[i] = (1 - K) * P[i]
    return x

#%%
kalman_filtered = kalman_filter(df['theta'], 0.0001, 0.01, 0, df['theta'][0])

print(kalman_filtered)

#%%
# Plot the data
plt.figure(figsize=(10, 5))
plt.plot(df['time_seconds'], df['theta'])
plt.plot(df['time_seconds'], kalman_filtered, label='Kalman Filter')
plt.xlabel('Time (s)')
plt.ylabel('Theta')
plt.title('Theta over Time')
plt.grid(True)

# Set the y-axis labels to display every second
plt.yticks(range(0, int(df['time_seconds'].max()) + 1, 1))

# Show the plot
plt.show()

# %%
