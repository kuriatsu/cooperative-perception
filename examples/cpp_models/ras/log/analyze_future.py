#!/user/bin/python3
# -*-coding:utf-8-*-
import matplotlib.pyplot as plt
import json
import sys
import numpy as np
import seaborn as sns
import math
import pandas as pd
import re
import os

def CalcReward(state_prev, state_curr):
    reward = 0
    for risk in state_curr["risks"]:
        if state_prev["lane_position"] < risk["lane_position"] <= state_curr["lane_position"]:

            if risk["hidden"] == "RISK":
                reward += (state_curr["speed"] - 2.8) / (11.2 - 2.8) * -100
            else:
                reward += (state_curr["speed"] - 2.8) / (11.2 - 2.8) * 100

    reward += (state_curr["speed"] - 2.8) / (11.2 - 2.8) * 1

    if state_prev["action"] == "REQUEST" and (state_prev["action"] != "REQUEST" or state_prev["action_target"] != state_curr["action_target"]):
        reward += -10

    if state_prev["action"] == "REQUEST" and (state_prev["action"] != "REQUEST" or state_prev["action_target"] != state_curr["action_target"]):
        if risk["hidden"] != risk["pred"]:
            reward += -1000

    return reward


palette = {"REFERENCE": "orangered","EGOISTIC": "indigo", "MYOPIC": "dodgerblue", "DESPOT": "forestgreen"}

df = pd.DataFrame(columns = ["situation", "policy", "date", "risk_num", "travel_time", "total_fuel_consumption", "mean_fuel_consumption", "dev_accel", "mean_speed", "risk_omission", "ambiguity_omission", "request_time", "total_reward", "acc"])
obstacle_df = pd.DataFrame(columns = ["situation", "policy", "date", "risk_num", "prob", "hidden", "pred", "speed", "distance_pred_speed", "distance_prob_speed", "distance_risk_speed"])


fig, ax = plt.subplots(1, 1, tight_layout=True)
dir_list = {"current":"../log_pie", "future":"./"}
for situation, directory in dir_list.items():
    files = os.listdir(directory)
    for file in sys.argv[1:]:
        if not file.endswith(".json"):
            continue

        with open(file, "r") as f:
            data = json.load(f)

        log = data.get("log")

        policy = re.findall("([A-Z]*)\d", file)[0]
        date = re.findall("_([\d]*)", file)[0]
        risk_num = float(re.findall("([\d.]*)_", file)[0]) * 2.0 * 500.0
        travel_time = 0.0
        fuel_consumption = [] 
        accel = []
        speed = []
        risk_omission = [] 
        ambiguity_omission = [] 
        request_time = 0.0
        request_history = []
        reward = [0]
        correct_pred = 0
        passed_target = 0

        last_ego_position = 0.0

        if log[0].get("risks") is not None:
            for risk in log[0].get("risks"):
                ## intervention target risk probaility distribution
                if risk.get("prob") == 1.0:
                    risk_prob_count.get(policy)[-1] += 1
                else:
                    risk_prob_count.get(policy)[math.floor(risk.get("prob")*10)] += 1

                ## add recog evaluation
                buf_recog_evaluation = pd.DataFrame([[risk.get("hidden"), risk.get("pred"), risk.get("prob")]], columns=recog_evaluation.columns)
                recog_evaluation = pd.concat([recog_evaluation, buf_recog_evaluation], ignore_index=True)

        for frame_num in range(1, len(log)):
            frame = log[frame_num]
            travel_time += data.get("delta_t")
            speed.append(frame.get("speed"))
            accel.append(frame.get("accel"))
            fuel_consumption.append(frame.get("fuel_consumption"))
            

            if frame.get("risks") is None:
                print(f"skipped {file} because of no obstacle spawned")
                continue

            if frame.get("action") == "REQUEST":
                ## add request time
                request_time += data.get("delta_t") 

            ## when ego_vehivle and risk crossed 
            for risk in frame.get("risks"):
                position = risk.get("lane_position") if risk.get("id")[0] != "-" else LANE_LENGTH - risk.get("lane_position")
                if last_ego_position <= position < frame.get("lane_position") and position < LANE_LENGTH-5.0:

                    ## risk ambiguity
                    ambiguity_omission.append(0.5 - abs(0.5 - risk.get("prob")))

                    ## passing speed to RISK obstacle
                    if risk.get("hidden"):
                        risk_omission.append(log[frame_num-1].get("speed"))

                    ## distance from ideal speed based on hidden risk
                    distance_pred_speed = 0.0
                    if risk.get("pred"):
                        distance_pred_speed = log[frame_num-1].get("speed")/11.2
                    else:
                        distance_pred_speed = (11.2 - log[frame_num-1].get("speed"))/11.2

                    ## distance from ideal prob-speed 
                    distance_prob_speed = 0.0
                    if risk.get("prob") >= 0.5:
                        distance_prob_speed = (1.0 - risk.get("prob"))**2 + (log[frame_num-1].get("speed")/11.2)**2
                    else:
                        distance_prob_speed = (risk.get("prob"))**2 + ((11.2 - log[frame_num-1].get("speed"))/11.2)**2

                    ## distance from ideal speed based on hidden risk
                    distance_risk_speed = 0.0
                    if risk.get("hidden"):
                        distance_risk_speed = log[frame_num-1].get("speed")/11.2
                    else:
                        distance_risk_speed = (11.2 - log[frame_num-1].get("speed"))/11.2

                    correct_pred += (risk.get("hidden") == risk.get("pred"))
                    passed_target += 1

                    ## add to obstacle data frame
                    buf_obstacle_df = pd.DataFrame([[situation, policy, date, risk_num, risk.get("prob"), risk.get("hidden"), risk.get("pred"), log[frame_num-1].get("speed"), distance_pred_speed, distance_prob_speed, distance_risk_speed]], columns=obstacle_df.columns)
                    obstacle_df = pd.concat([obstacle_df, buf_obstacle_df], ignore_index=True)

            ## calculate reward
            reward.append(CalcReward(log[frame_num-1], log[frame_num]))

            last_ego_position = frame.get("lane_position") 

        ## summarize
        total_fuel_consumption = sum(fuel_consumption)
        mean_fuel_consumption = sum(fuel_consumption)/len(fuel_consumption)
        mean_speed = sum(speed)/len(speed)
        total_reward = sum(reward)

        dev_accel = 0.0
        mean_accel = sum(accel)/len(accel)
        for a in accel:
            dev_accel += (a - mean_accel) ** 2
        dev_accel = dev_accel / len(accel)

        mean_risk_omission = sum(risk_omission)/len(risk_omission) if risk_omission else 0.0
        mean_ambiguity_omission = sum(ambiguity_omission)/len(ambiguity_omission) if ambiguity_omission else 0.0
        
        acc = correct_pred / passed_target if passed_target > 0 else None 

        buf_df = pd.DataFrame([[situation, policy, date, risk_num, travel_time, total_fuel_consumption, mean_fuel_consumption, dev_accel, mean_speed, mean_risk_omission, mean_ambiguity_omission, request_time, total_reward, acc]], columns=df.columns)
        df = pd.concat([df, buf_df], ignore_index=True)


sns.lineplot(data=df, x="risk_num", y="travel_time", hue="policy", style="situation", markers=True, palette=palette)
plt.savefig("travel_time.svg", transparent=True)
plt.clf()

sns.lineplot(data=df, x="risk_num", y="total_fuel_consumption", hue="policy", style="situation", markers=True, palette=palette)
plt.savefig("total_fuel_consumption.svg", transparent=True)
plt.clf()

sns.lineplot(data=df, x="risk_num", y="mean_fuel_consumption", hue="policy", style="situation", markers=True, palette=palette)
plt.savefig("mean_fuel_consumption.svg", transparent=True)
plt.clf()

sns.lineplot(data=df, x="risk_num", y="dev_accel", hue="policy", style="situation", markers=True, palette=palette)
plt.ylim([0.0, 11.2])
plt.savefig("accel.svg", transparent=True)
plt.clf()

sns.lineplot(data=df, x="risk_num", y="mean_speed", hue="policy", style="situation", markers=True, palette=palette)
plt.ylim([0.0, 11.2])
plt.savefig("mean_speed.svg", transparent=True)
plt.clf()

sns.lineplot(data=df, x="risk_num", y="ambiguity_omission", hue="policy", style="situation", markers=True, palette=palette)
plt.savefig("ambiguity_omission.svg", transparent=True)
plt.clf()

sns.lineplot(data=df, x="risk_num", y="request_time", hue="policy", style="situation", markers=True, palette=palette)
plt.savefig("request_time.svg", transparent=True)
plt.clf()

sns.lineplot(data=df, x="risk_num", y="total_reward", hue="policy", style="situation", markers=True, palette=palette)
plt.savefig("reward.svg", transparent=True)
plt.clf()

sns.lineplot(data=df, x="risk_num", y="acc", hue="policy", style="situation", markers=True, palette=palette)
plt.set_ylim([0.0, 1.0])
plt.savefig("total_acc.svg", transparent=True)
plt.clf()


# speed - prediction distance plot
fig, ax = plt.subplots(tight_layout = True)
sns.lineplot(data=obstacle_df, x="risk_num", y="distance_pred_speed", hue="policy", style="situation", markers=True, ax=ax, palette=palette)

plt.savefig("speed_pred_distance.svg", transparent=True)

# speed - probability distance plot
fig, ax = plt.subplots(tight_layout = True)
sns.lineplot(data=obstacle_df, x="risk_num", y="distance_prob_speed", hue="policy", style="situation", markers=True, ax=ax, palette=palette)

plt.savefig("speed_prob_distance.svg", transparent=True)

# speed - hidden risk distance plot
fig, ax = plt.subplots(tight_layout = True)
sns.lineplot(data=obstacle_df, x="risk_num", y="distance_risk_speed", hue="policy", style="situation", markers=True, ax=ax, palette=palette)

plt.savefig("speed_risk_distance.svg", transparent=True)

# speed - hidden risk speed plot
fig, ax = plt.subplots(tight_layout = True)
sns.lineplot(data=obstacle_df, x="risk_num", y="speed", hue="policy", style="situation", markers=True, ax=ax, palette=palette)
ax.set_ylim(0.0, 12.0)
plt.savefig("pass_speed.svg", transparent=True)
plt.clf()

