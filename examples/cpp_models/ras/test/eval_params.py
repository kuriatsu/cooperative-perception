import json
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import sys

df = pd.DataFrame(columns = ["risk_num", "time_per_move", "delta_t", "reward"])
for file in sys.argv[1:]:
    with open(file, "r") as f:
        data = json.load(f)

    for log in data.get("log"):
        buf = df.DataFrame([[len(log.get("risk_pose")), log.get("time_per_move"), log.get("delta_t"), log.get("total discount reward")]], columns = df.columns)
        df = pd.concat([df, buf], ignore_index=True)



fig, ax = plt.subplots()
sns.lineplot(data=data, x="time_per_move", y="reward", hue="risk_num")
plt.show()

