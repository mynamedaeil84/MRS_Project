import pandas as pd
import os

log_dir = '/home/YOUR_USERNAME/ros2_ws/scenario2_experiments/logs_v2'
os.makedirs(log_dir, exist_ok=True)

files = [f for f in os.listdir(log_dir) if f.endswith('.csv')]

summary = []
for file in files:
    df = pd.read_csv(os.path.join(log_dir, file))
    df['timestamp'] = pd.to_datetime(df['timestamp'])

    duration = (df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]).total_seconds()
    events = df['event'].value_counts()

    summary.append({
        'Condition': file.replace('.csv', ''),
        'Time(s)': round(duration, 1),
        'Obstacles': events.get('ObstacleDetected', 0),
        'Hazards': events.get('HazardZone', 0),
        'Tasks': events.get('TaskAssigned', 0),
        'GoalReached': int('GoalReached' in events),
        'Coverage(%)': float(df[df['event'] == 'GoalReached']['detail'].str.extract(r"coverage=([0-9.]+)").fillna(0).values[-1][0]) if 'GoalReached' in df['event'].values else 0.0
    })

pd.DataFrame(summary).to_csv(os.path.join(log_dir, 'summary_table_v2.csv'), index=False)

