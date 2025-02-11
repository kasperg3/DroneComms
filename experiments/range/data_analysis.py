import pandas as pd
import matplotlib.pyplot as plt


# Define the paths to the new CSV files
new_csv_file_paths = [
    '/home/kang/workspace/DroneComms/experiments/range/range_2_os.txt',
    '/home/kang/workspace/DroneComms/experiments/range/range_2_p4p.txt'
]

# Read the new CSV files into DataFrames
range_dfs = [
    pd.read_csv(path, header=0, names=['time', 'rssi', 'bytes', 'source', 'destination'])
    for path in new_csv_file_paths
]

first_timestamp_df0_time = range_dfs[0]['time'].iloc[0]
first_timestamp_df1_time = range_dfs[1]['time'].iloc[0]
print(f"First timestamp in df0: {first_timestamp_df0_time}")
print(f"First timestamp in df1: {first_timestamp_df1_time}")

# Adjust the time of the first DataFrame so that both are aligned
range_dfs[0]['time'] = range_dfs[0]['time'] - (first_timestamp_df0_time - first_timestamp_df1_time)

# Combine the new DataFrames
combined_range_df = pd.concat(range_dfs, ignore_index=True)

# Calculate the time difference between packets with the same source, destination, and bytes
combined_range_df['time_diff'] = combined_range_df.groupby(['source', 'destination', 'bytes'])['time'].diff()

# Display the first few rows of the DataFrame with the time differences
print("First few rows of the data with time differences:")
print(combined_range_df.head())

# Filter out rows where RSSI is 0
filtered_range_df = combined_range_df[combined_range_df['rssi'] != 0]

# Get a sorted list of unique sources (agents)
unique_sources = sorted(filtered_range_df['source'].unique())

# Create subplots (one for each agent). Here we create 1 row and as many columns as there are agents.
fig, axes = plt.subplots( len(unique_sources),1, figsize=(8, 6), sharex=True)

# If there is only one agent, ensure axes is iterable
if len(unique_sources) == 1:
    axes = [axes]
    
for i, source in enumerate(unique_sources):
    ax = axes[i]
    # Select and sort the data for the current agent/source
    group = filtered_range_df[filtered_range_df['source'] == source].sort_values('time').copy()
    agent_label = mac_to_agent.get(source, source)
    
    # Plot RSSI on the primary y-axis (left)
    ax.plot(group['time'] / 1000, group['rssi'], color='tab:blue')
    ax.set_xlabel('Time (seconds)', fontsize='large')
    ax.set_ylabel('RSSI (dBm)', fontsize='large', color='tab:blue')
    ax.tick_params(axis='y', labelcolor='tab:blue')
    ax.set_title(f"{agent_label}", fontsize='large')
    ax.set_ylim(-90, -40)
    ax.set_xlim(85,430)
    # Create a secondary y-axis for the rolling message count
    ax2 = ax.twinx()
    ax2.set_ylim(0,25)
    # Convert the 'time' from milliseconds to datetime to enable time-based rolling window operations
    group['time_dt'] = pd.to_datetime(group['time'], unit='ms')
    group.set_index('time_dt', inplace=True)
    
    # Compute the rolling count of messages over the past 10 seconds (10,000 ms)
    rolling_count = group['rssi'].rolling('10s').count()
    # rolling_count = rolling_count.rolling('10s').mean()
    
    # Plot the rolling message count on the secondary y-axis (right)
    ax2.plot(group['time'] / 1000, rolling_count, linestyle='--', color='tab:orange')
    ax2.set_ylabel('Packets per 10 seconds', fontsize='large', color='tab:orange')
    ax2.tick_params(axis='y', labelcolor='tab:orange')
    
    # Combine legends from both axes so that both data series appear in the subplot legend
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    # ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize='medium')

plt.tight_layout()
plt.savefig('rssi_and_message_count_by_agent.svg', format='svg')
plt.show()

# Filter the combined_range_df to only include data between time=85,000 and 430,000 milliseconds
filtered_combined_range_df = combined_range_df[(combined_range_df['time'] >= 90000) & (combined_range_df['time'] <= 430000)]

# Calculate the number of sent, dropped, and received messages for each UAV using the filtered data
for source in unique_sources:
    source_label = mac_to_agent.get(source, source)
    source_df = filtered_combined_range_df[filtered_combined_range_df['source'] == source]
    
    # Calculate sent messages (RSSI = 0 in source_df)
    sent_messages = source_df[source_df['rssi'] == 0].shape[0]
    
    # Calculate received messages by looking for matching packets in other sources
    received_messages = filtered_combined_range_df[
        (filtered_combined_range_df['source'] == source) & 
        (filtered_combined_range_df['destination'] != source) &
        (filtered_combined_range_df['rssi'] != 0)
    ].shape[0]
    
    dropped_messages = sent_messages - received_messages
    
    print(f"\n{source_label}:")
    print(f"  Sent messages: {sent_messages}")
    print(f"  Dropped messages: {dropped_messages}")
    print(f"  Received messages: {received_messages}")
    print(f"  Percentage of dropped messages: {dropped_messages / sent_messages * 100:.2f}%")