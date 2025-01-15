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
plt.plot(rssi_values, marker='o')
plt.title('RSSI Values Over Time')
plt.xlabel('Sample Number')
plt.ylabel('RSSI (dBm)')
plt.grid(True)
plt.show()