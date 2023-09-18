import json
import sys
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

with open(sys.argv[1], "r") as f:
    data = json.load(f)

travel_dist = []
ego_vel = []
for frame in data:
    travel_dist.append(frame.get("lane_position"))
    ego_vel.append(frame.get("speed"))

fig, ax = plt.subplots(3, 1, tight_layout=True)
ax[0].plot(travel_dist, ego_vel, ".", linestyle="-")
ax[0].set_xlim(0, 500)
ax[0].set_ylabel("vehivle speed [m/s]")
ax[1].set_xlim(0, 500)
ax[1].set_ylabel("risk probs")
ax[2].set_xlim(0, 500)
ax[2].set_ylabel("intervention request")
ax[2].set_xlabel("travel distance [m]")

risk_ids = [] 
color_map = plt.get_cmap("Set3")
risk_num = len(data[0].get("risks"))

for risk in data[0].get("risks"):
    risk_ids.append(risk.get("id"))

prev_ego_pose = 0.0
for i in range(0, len(risk_ids)):
    id = risk_ids[i]
    risk_prob = []
    risk_pose = []
    clossing_frame = 0

    for frame_num in range(0, len(data)):
        for risk in data[frame_num].get("risks"):

            if id != risk.get("id"): continue

            risk_prob.append(risk.get("prob")) 
            position = risk.get("lane_position") if id[0] != "-" else 500.0 - risk.get("lane_position")
            risk_pose.append(position)

            if prev_ego_pose <= position < data[frame_num].get("lane_position"):
                clossing_frame = frame_num

        prev_ego_pose = data[frame_num].get("lane_position")

    ax[1].plot(risk_pose, risk_prob, linestyle="-", color=color_map(i/len(risk_ids)))
    ax[1].plot(risk_pose[0], risk_prob[0]+0.01, marker="$S$", markersize=5, color=color_map(i/len(risk_ids)))
    ax[1].plot(risk_pose[clossing_frame], risk_prob[clossing_frame]+0.01, "x", markersize=5, color=color_map(i/len(risk_ids)))
    ax[1].plot(risk_pose[-1], risk_prob[-1]+0.01, marker="$G$", markersize=5, color=color_map(i/len(risk_ids)))


last_request_target = ""
request_position = []
hight = 1
for frame in data:
    print(request_position, last_request_target, frame.get("action_target"))

    if frame.get("action") == "REQUEST" \
       and (len(request_position)==0 or frame.get("action_target") == last_request_target):
            request_position.append(frame.get("lane_position"))
            last_request_target = frame.get("action_target")
    elif len(request_position) > 0:
        ax[2].plot(request_position, [hight]*len(request_position), marker=".", linestyle="-", color=color_map(risk_ids.index(last_request_target)/len(risk_ids)))
        request_position = []
        last_request_target = ""
        hight += 1


plt.show()



