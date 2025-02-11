import pandas as pd
import matplotlib.pyplot as plt

# # Define the paths to the CSV files
# csv_file_paths = [
#     '/home/kang/workspace/DroneComms/experiments/uav_rssi_experiment/line_formation_OSD.txt',
#     '/home/kang/workspace/DroneComms/experiments/uav_rssi_experiment/line_formation_DS.txt',
#     '/home/kang/workspace/DroneComms/experiments/uav_rssi_experiment/line_formation_p4p.txt'
# ]

# # csv_file_paths = [
# #     '/home/kang/workspace/DroneComms/communication_firmware/experiment/triangle_formation_ds.txt',
# #     '/home/kang/workspace/DroneComms/communication_firmware/experiment/triangle_formation_os.txt',
# #     '/home/kang/workspace/DroneComms/communication_firmware/experiment/triangle_formation_p4p.txt'
# # ]
# # Read the CSV files into DataFrames
# dfs = [pd.read_csv(path, header=0, names=['time', 'rssi', 'bytes', 'source', 'destination']) for path in csv_file_paths]

# # Find the first common timestamp
# common = dfs[0].merge(dfs[1], on=["bytes", "source"], suffixes=("_df0", "_df1"), how="inner")
# common = common.merge(dfs[2], on=["bytes", "source"], suffixes=("", "_df2"), how="inner")
# common_with_zero = common[
#     (common["rssi"] == 0) |
#     (common["rssi_df0"] == 0) |
#     (common["rssi_df1"] == 0)
# ]

# first_line = common_with_zero.iloc[0]
# dfs[0]['time'] = dfs[0]["time"] 
# dfs[1]['time'] = dfs[1]["time"]
# dfs[2]['time'] = dfs[2]["time"] 
# # # Adjust all times to start at 0
# # min_time = max(df['time'].min() for df in dfs)
# # dfs = [df.assign(time=df['time'] - min_time) for df in dfs]

# # Combine the DataFrames
# combined_df = pd.concat(dfs, ignore_index=True)

# # Display the first few rows of the combined DataFrame
# print("First few rows of the combined data:")
# print(combined_df.head())

# # Basic statistics
# print("\nBasic statistics of the combined data:")
# print(combined_df.describe())

# # Count the number of unique destinations and sources
# unique_destinations_combined = combined_df['destination'].nunique()
# unique_sources_combined = combined_df['source'].nunique()
# print(f"\nNumber of unique destinations in combined data: {unique_destinations_combined}")
# print(f"Number of unique sources in combined data: {unique_sources_combined}")

# # Group by destination and calculate the average RSSI and total bytes
# grouped_by_destination_combined = combined_df.groupby('destination').agg({'rssi': 'mean', 'bytes': 'sum'})
# print("\nAverage RSSI and total bytes by destination in combined data:")
# print(grouped_by_destination_combined)

# # Group by source and calculate the average RSSI and total bytes
# grouped_by_source_combined = combined_df.groupby('source').agg({'rssi': 'mean', 'bytes': 'sum'})
# print("\nAverage RSSI and total bytes by source in combined data:")
# print(grouped_by_source_combined)

# # Plot RSSI over time for each destination with lines for each source
# grouped_by_destination = combined_df[combined_df['rssi'] != 0].groupby('destination')
# # Create a mapping from MAC addresses to agent names
mac_to_agent = {
    '24:58:7c:e4:01:18': 'UAV 0',
    '24:ec:4a:00:3c:ac': 'UAV 1',
    '64:e8:33:7e:ab:74': 'UAV 2'
}
# fig, axs = plt.subplots(3, 1, figsize=(8, 6), sharex=True)

# for i, (destination, group) in enumerate(grouped_by_destination):
#     grouped_by_source = group.groupby('source')
#     for source, source_group in grouped_by_source:
#         source_label = mac_to_agent.get(source)
#         color_map = {
#             'UAV 0': plt.get_cmap('tab10')(0),
#             'UAV 1': plt.get_cmap('tab10')(1),
#             'UAV 2': plt.get_cmap('tab10')(2)
#         }
#         axs[i].plot(source_group['time'] / 1000, source_group['rssi'], label=f'{source_label}', color=color_map.get(source_label, 'black'))
#     destination_label = mac_to_agent.get(destination)
#     axs[i].legend(fontsize='small', loc='upper center')
#     axs[i].set_title(f'{destination_label}', fontsize='large')
#     axs[i].set_xlabel('Time (seconds)', fontsize='large')
#     axs[i].set_ylabel('RSSI (dBm)', fontsize='large')
#     axs[i].set_ylim(top=-10,bottom=-100 )

# plt.tight_layout()
# plt.savefig('rssi_over_time_subplots.svg', format='svg')


# plt.figure(figsize=(4, 3))
# # Plot Not Received Packages by Size
# not_received_df = combined_df[combined_df['rssi'] == 0]
# # Filter the combined DataFrame to only include data between time=200000 and 600000
# filtered_combined_df = combined_df[(combined_df['time'] >= 200000) & (combined_df['time'] <= 600000)]

# # Plot Not Received Packages by Size using the filtered data
# not_received_df = filtered_combined_df[filtered_combined_df['rssi'] == 0]

# def has_received_in_window(row):
#     window = filtered_combined_df[
#         (filtered_combined_df['bytes'] == row['bytes'])
#         & (filtered_combined_df['time'].between(row['time'] - 2000, row['time'] + 2000))
#         & (filtered_combined_df['rssi'] != 0)
#         & (filtered_combined_df['source'] != row['source'])
#     ]
#     return not window.empty

# not_received_df = not_received_df[not_received_df.apply(has_received_in_window, axis=1)]
# not_received_count_by_size = not_received_df.groupby('bytes')['rssi'].count()

# bin_edges = range(0, not_received_count_by_size.index.max() + 50, 50)
# not_received_count_by_size_binned = not_received_count_by_size.groupby(pd.cut(not_received_count_by_size.index, bins=bin_edges)).sum()
# # Set x-ticks to show time in seconds
# plt.xticks(ticks=range(0, len(not_received_count_by_size_binned.index), 6), labels=[str(i * 50) for i in range(0, len(not_received_count_by_size_binned.index), 6)])
# plt.bar(not_received_count_by_size_binned.index.astype(str), not_received_count_by_size_binned.values)
# plt.xlabel('Packet Size (bytes)', fontsize='small')
# plt.ylabel('Number of dropped packets', fontsize='small')
# plt.yticks(range(0, int(not_received_count_by_size_binned.max()) + 1))  # Ensure y-axis uses integers
# plt.tight_layout()
# plt.savefig('not_received_packages.svg', format='svg')
# plt.show()
# # Print the total number of dropped packages
# total_dropped_packages = not_received_df.shape[0]
# # Calculate the percentage of dropped packages
# total_packages = combined_df.shape[0]
# print(f"\nTotal number of dropped packages: {total_dropped_packages} out of {total_packages}")
# dropped_packages_percentage = (total_dropped_packages / total_packages) * 100
# print(f"\nPercentage of dropped packages: {dropped_packages_percentage:.2f}%")

# # Plot average RSSI by destination
# plt.figure(figsize=(6, 3))  
# grouped_by_destination_combined['rssi'].plot(kind='bar')
# plt.title('Average RSSI by Destination', fontsize='medium')
# plt.xlabel('Destination', fontsize='small')
# plt.ylabel('Average RSSI (dBm)', fontsize='small')
# plt.tight_layout()
# plt.savefig('average_rssi_by_destination.svg', format='svg')


# # Plot total bytes by destination
# # Separate the data for destination 'ff:ff:ff:ff:ff:ff' and others
# ff_destination = grouped_by_destination_combined.loc['ff:ff:ff:ff:ff:ff', 'bytes']
# other_destinations = grouped_by_destination_combined.drop('ff:ff:ff:ff:ff:ff', errors='ignore')['bytes'].sum()

# # Create a DataFrame for plotting
# plot_data = pd.DataFrame({
#     'Destination': ['ff:ff:ff:ff:ff:ff', 'Others'],
#     'Total Bytes': [ff_destination, other_destinations]
# })

# # Plot the data
# plt.figure(figsize=(6, 3))
# plot_data.set_index('Destination')['Total Bytes'].plot(kind='bar', color=['blue', 'orange'])
# plt.title('Total Bytes by Destination', fontsize='medium')
# plt.xlabel('Destination', fontsize='small')
# plt.ylabel('Total Bytes', fontsize='small')
# plt.tight_layout()
# plt.figure(figsize=(6, 3))
# plt.savefig('total_bytes_by_destination.png')

# grouped_by_destination_combined['bytes'].plot(kind='bar', color='green')
# plt.title('Total Bytes by Destination', fontsize='medium')
# plt.xlabel('Destination', fontsize='small')
# plt.ylabel('Total Bytes', fontsize='small')
# plt.tight_layout()
# plt.savefig('total_bytes_by_destination.png')


# Define the paths to the new CSV files
new_csv_file_paths = [
    '/home/kang/workspace/DroneComms/experiments/range/range_os.txt',
    '/home/kang/workspace/DroneComms/experiments/range/range_p4p.txt'
]

# Read the new CSV files into DataFrames
range_dfs = [pd.read_csv(path, header=0, names=['time', 'rssi', 'bytes', 'source', 'destination']) for path in new_csv_file_paths]

first_timestamp_df0_time = range_dfs[0]['time'].iloc[0]
first_timestamp_df1_time = range_dfs[1]['time'].iloc[0]
print(f"First timestamp in df0: {first_timestamp_df0_time}")
print(f"First timestamp in df1: {first_timestamp_df1_time}")

range_dfs[0]['time'] = range_dfs[0]['time'] - (first_timestamp_df0_time-first_timestamp_df1_time)

# Combine the new DataFrames
combined_range_df = pd.concat(range_dfs, ignore_index=True)
# Calculate the time difference between packets with the same source, destination, and bytes
combined_range_df['time_diff'] = combined_range_df.groupby(['source', 'destination', 'bytes'])['time'].diff()


# Display the first few rows of the DataFrame with the time differences
print("First few rows of the data with time differences:")
print(combined_range_df.head())


# Filter out rows where RSSI is 0
filtered_range_df = combined_range_df[combined_range_df['rssi'] != 0]

# Plot RSSI over time
plt.figure(figsize=(10, 6))
for source, group in filtered_range_df.groupby('source'):
    source_label = mac_to_agent.get(source, source)
    plt.plot(group['time'] / 1000, group['rssi'], label=source_label)

plt.xlabel('Time (seconds)', fontsize='large')
plt.ylabel('RSSI (dBm)', fontsize='large')
plt.title('RSSI over Time for New Data', fontsize='large')
plt.legend(fontsize='small')
plt.tight_layout()
plt.savefig('rssi_over_time_new_data.svg', format='svg')
plt.show()