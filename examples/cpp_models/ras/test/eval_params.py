import json
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import sys
import pandas as pd

df = pd.DataFrame(columns = ["risk_num", "time_per_move", "delta_t", "reward"])
for file in sys.argv[1:]:
    with open(file, "r") as f:
        data = json.load(f)

    for log in data.get("log"):
        buf = pd.DataFrame([[len(log.get("risk_pose")), log.get("time_per_move"), log.get("delta_t"), log.get("total discounted reward")]], columns = df.columns)
        df = pd.concat([df, buf], ignore_index=True)



fig, ax = plt.subplots()
print(df)
sns.lineplot(data=df, x="time_per_move", y="reward", hue="risk_num")
plt.show()

