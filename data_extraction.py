import matplotlib.pyplot as plt
import re

# Regular expression to extract RSSI values
rssi_pattern = r'RSSI: (-\d+ dBm)'

# Load the text from experiment_log.txt file
with open('/home/kang/workspace/DroneComms/experiment_log.txt', 'r') as file:
    text = file.read()
# Extracting RSSI values
rssi_values = re.findall(rssi_pattern, text)

# Remove the ' dBm' part from the extracted RSSI values
rssi_values = [int(rssi.split()[0]) for rssi in rssi_values]
# Print the extracted RSSI values
print(rssi_values)
# Printing the extracted RSSI values
# Regular expression to extract unpacked bytes values
bytes_pattern = r'Writing (\d+) Bytes to serial'

# Extracting unpacked bytes values
bytes_values = re.findall(bytes_pattern, text)
# Regular expression to extract milliseconds since start
time_pattern = r';32mI \((\d+)\) ESP-COMM: Data received from MAC:'

# Extracting time values
time_values = re.findall(time_pattern, text)
# Convert the extracted time values to integers
time_values = [int(time) for time in time_values]

# Calculate the frequency of messages
frequencies = []
for i in range(1, len(time_values)):
    delta_time = (time_values[i] - time_values[i - 1]) / 1000.0  # Convert milliseconds to seconds
    if delta_time > 0:
        frequency = 1 / delta_time
        frequencies.append(frequency)

# Plotting the frequency of messages
plt.figure()
plt.plot(frequencies)
plt.title('Message Frequency Over Time')
plt.xlabel('Sample Number')
plt.ylabel('Frequency (Hz)')
plt.grid(True)
plt.show()
# Convert the extracted bytes values to integers
bytes_values = [int(bytes) for bytes in bytes_values]
# Plotting the unpacked bytes values
plt.figure()
plt.plot(bytes_values, marker='o', color='r')
plt.title('Unpacked Bytes Values Over Time')
plt.xlabel('Sample Number')
plt.ylabel('Bytes')
plt.grid(True)
plt.show()
# Print the extracted unpacked bytes values
# Plotting the RSSI values
plt.plot(rssi_values)
plt.title('RSSI Values Over Time')
plt.xlabel('Sample Number')
plt.ylabel('RSSI (dBm)')
plt.grid(True)
plt.show()