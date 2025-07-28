import pandas as pd
import os
from datetime import datetime

log_dir = '/home/YOUR_USERNAME/ros2_ws/scenario2_experiments/logs'
files = [f for f in os.listdir(log_dir) if f.endswith('.csv')]

summary = []

for file in files:
    path = os.path.join(log_dir, file)
    df = pd.read_csv(path)
    event_counts = df['event'].value_counts()
    time_diff = pd.to_datetime(df['timestamp'].iloc[-1]) - pd.to_datetime(df['timestamp'].iloc[0])
    duration = time_diff.total_seconds()

    summary.append({
        'Condition': file.replace('eval_', '').replace('.csv', ''),
        'Time (s)': round(duration, 2),
        'Obstacles': event_counts.get('ObstacleDetected', 0),
        'Hazards': event_counts.get('HazardZoneReceived', 0),
        'Tasks': event_counts.get('TaskAssigned', 0),
    })

df_summary = pd.DataFrame(summary)
print(df_summary.to_string(index=False))

# 저장
df_summary.to_csv(os.path.join(log_dir, 'summary_table.csv'), index=False)

