import matplotlib.pyplot as plt
import json
import sys
import numpy as np
import seaborn as sns
import math
import pandas as pd
import re

def CarcReward(state_prev, state_curr, action):
    reward = 0
    for risk in state_curr["risks"]:
        if state_prev["lane_position"] < risk["lane_position"] <= state_curr["lane_position"]:

            if risk["risk_hidden"] == "RISK":
                raward += (state_curr["speed"] - 2.8) / (11.2 - 2.8) * -100
            else:
                raward += (state_curr["speed"] - 2.8) / (11.2 - 2.8) * 100

    raward += (state_curr["speed"] - 2.8) / (11.2 - 2.8) * 1

    if action == "REQUEST" and (state_curr["request_time"] == 1.0 or state_prev["target"] != state_curr["target"]):
        reward += -10

    if state_prev["req_time"] > 0 and (action != "REQUEST" or state_prev["target"] != state_prev["target"]):
        if risk["risk_hidden"] != risk["risk_pred"]:
            reward += -1000

    return reward

#### single data visualization ####
#### travel distance vs (speed, risk_prob, action) ####
#if len(sys.argv) == 2:
#
#    with open(sys.argv[1], "r") as f:
#        data = json.load(f)
#
#    travel_dist = []
#    ego_vel = []
#    for frame in data:
#        travel_dist.append(frame.get("lane_position"))
#        ego_vel.append(frame.get("speed"))
#
#    fig, ax = plt.subplots(3, 1, tight_layout=True)
#    ax[0].plot(travel_dist, ego_vel, ".", linestyle="-")
#    ax[0].set_xlim(0, 500)
#    ax[0].set_ylabel("vehicle speed [m/s]")
#    ax[1].set_xlim(0, 500)
#    ax[1].set_ylabel("risk probs")
#    ax[2].set_xlim(0, 500)
#    ax[2].set_ylabel("intervention request")
#    ax[2].set_xlabel("travel distance [m]")
#
#    risk_ids = [] 
#    color_map = plt.get_cmap("Set3")
#    risk_num = len(data[0].get("risks"))
#
#    for risk in data[0].get("risks"):
#        risk_ids.append(risk.get("id"))
#
#    prev_ego_pose = 0.0
#    for i in range(0, len(risk_ids)):
#        id = risk_ids[i]
#        risk_prob = []
#        risk_pose = []
#        clossing_frame = 0
#
#        for frame_num in range(0, len(data)):
#            for risk in data[frame_num].get("risks"):
#
#                if id != risk.get("id"): continue
#
#                risk_prob.append(risk.get("prob")) 
#                position = risk.get("lane_position") if id[0] != "-" else 500.0 - risk.get("lane_position")
#                risk_pose.append(position)
#
#                if prev_ego_pose < position <= data[frame_num].get("lane_position"):
#                    clossing_frame = frame_num
#
#            prev_ego_pose = data[frame_num].get("lane_position")
#
#        ax[1].plot(risk_pose, risk_prob, linestyle="-", color=color_map(i/len(risk_ids)))
#        ax[1].plot(risk_pose[0], risk_prob[0]+0.01, marker="$S$", markersize=5, color=color_map(i/len(risk_ids)))
#        ax[1].plot(risk_pose[clossing_frame], risk_prob[clossing_frame]+0.01, "x", markersize=5, color=color_map(i/len(risk_ids)))
#        ax[1].plot(risk_pose[-1], risk_prob[-1]+0.01, marker="$G$", markersize=5, color=color_map(i/len(risk_ids)))
#
#
#    last_request_target = ""
#    request_position = []
#    hight = 1
#    for frame in data:
#        print(request_position, last_request_target, frame.get("action_target"))
#
#        if frame.get("action") == "REQUEST" \
#           and (len(request_position)==0 or frame.get("action_target") == last_request_target):
#                request_position.append(frame.get("lane_position"))
#                last_request_target = frame.get("action_target")
#        elif len(request_position) > 0:
#            ax[2].plot(request_position, [hight]*len(request_position), marker=".", linestyle="-", color=color_map(risk_ids.index(last_request_target)/len(risk_ids)))
#            request_position = []
#            last_request_target = ""
#            hight += 1
#
#
#    plt.show()

if len(sys.argv) == 2:
    fig, ax = plt.subplots(2, 1, tight_layout=True)

    with open(sys.argv[1], "r") as f:
        data = json.load(f)


    log = data.get("log")

    ##########################
    # print("time-speed")
    ##########################
    elapse_time_list = []
    elapse_time = 0.0
    ego_vel = []
    for frame in log:
        elapse_time_list.append(elapse_time)
        elapse_time +=  data.get("delta_t")
        ego_vel.append(frame.get("speed"))
    
    ax[0].plot(elapse_time_list, ego_vel, ".", linestyle="-")

    risk_ids = [] 
    risk_num = len(log[0].get("risks"))
    for risk in log[0].get("risks"):
        risk_ids.append(risk.get("id"))
    color_map = plt.get_cmap("Set3")

    ##########################
    # print("risk position and prob")
    ##########################
    ax2 = ax[0].twinx()
    reserved_time_list = []
    for i in range(0, len(risk_ids)):
        id = risk_ids[i]
        elapsed_time = 0.0
        is_crossed = False
        last_prob = None
        crossing_time = 0
        

        for frame in log:
            elapsed_time += data.get("delta_t")
            for risk in frame.get("risks"):

                print(risk)
                if id != risk.get("id"): continue

                last_prob = risk.get("prob")
                position = risk.get("lane_position") if id[0] != "-" else 500.0 - risk.get("lane_position")
                if crossing_time == 0 and position <= frame.get("lane_position"):
                    print(f"crossed with {risk.get('id')} at {elapsed_time} risk: {risk.get('prob')}")
                    is_crossed = True

                    crossing_time = elapsed_time - 1.0
                    while True:
                        if crossing_time in reserved_time_list: 
                            crossing_time += 0.8
                        else:
                            break
                    crossing_prob = risk.get("prob")
                    if crossing_prob < 0.1: 
                        crossing_prob += 0.02 

                    ax2.bar(crossing_time, crossing_prob, color=color_map(i/len(risk_ids)))
                    ax2.annotate(id, xy=[crossing_time, crossing_prob], size=10, color="black")
                    reserved_time_list.append(crossing_time)
                    continue



        ## no crossing means the target are at the end of the course
        if not is_crossed:
            crossing_time = elapsed_time
            while True:
                if crossing_time in reserved_time_list: 
                    crossing_time += 1.0
                else:
                    break
            ax2.bar(crossing_time, last_prob, color=color_map(i/len(risk_ids)))
            print(f"{id} doesn't crossed time: {crossing_time} prob : {last_prob}")


    ##########################
    # print("intervention request")
    ##########################
    last_action_target = None
    buf_prob = []
    buf_time = []
    reward = [0]

    ## first log
    if log[0].get("action") == "REQUEST":
        target = log[0].get("action_target")
        for risk in log[0].get("risks"):
            if target == risk.get("id"):
                buf_prob.append(risk.get("prob"))
                buf_time.append(0.0)
                break
        last_action_target = target

    ## from second log
    for frame_num in range(1, len(log)):
        elapsed_time = frame_num * data.get("delta_t") 
        frame = log[frame_num]
        target = frame.get("action_target")

        if frame.get("action") == "REQUEST":

            ## keep intervention request
            if last_action_target == target:
                for risk in frame.get("risks"):
                    if target == risk.get("id"):
                        buf_prob.append(risk.get("prob"))
                        buf_time.append(elapsed_time)
                        break

            ## request 1 -> requet 2  
            ## plot last intervention request to 1 
            if last_action_target != target and last_action_target is not None:
                ax[1].plot(buf_time, buf_prob, linestyle="-", color=color_map(risk_ids.index(last_action_target)/len(risk_ids)))
                ax[1].plot(buf_time[1:], buf_prob[1:], marker=".", linestyle="", color=color_map(risk_ids.index(last_action_target)/len(risk_ids)))
                ax[1].plot(buf_time[0], buf_prob[0], marker="x", linestyle="", color=color_map(risk_ids.index(last_action_target)/len(risk_ids)))

            ## start intervention request
            ## 1. clear log log 
            ## 2. save initial prob state and prob state after first intervention request
            if last_action_target != target:
                buf_prob = []
                buf_time = []
                for risk in log[frame_num-1].get("risks"):
                    if target == risk.get("id"):
                        buf_prob.append(risk.get("prob"))
                        buf_time.append(elapsed_time-data.get("delta_t"))
                        break
                for risk in frame.get("risks"):
                    if target == risk.get("id"):
                        buf_prob.append(risk.get("prob"))
                        buf_time.append(elapsed_time)
                        break

                ax[1].annotate(target, xy=[buf_time[0], buf_prob[0]], size=10, color="black")
            last_action_target = target
                
        else:
            ## finish intervention request 
            ## plot last intervention request log and clear log
            if last_action_target is not None:
                ax[1].plot(buf_time, buf_prob, linestyle="-", color=color_map(risk_ids.index(last_action_target)/len(risk_ids)))
                ax[1].plot(buf_time[1:], buf_prob[1:], marker=".", linestyle="", color=color_map(risk_ids.index(last_action_target)/len(risk_ids)))
                ax[1].plot(buf_time[0], buf_prob[0], marker="x", linestyle="", color=color_map(risk_ids.index(last_action_target)/len(risk_ids)))
                buf_prob = []
                buf_time = []

            ## no intervention request
            else:
                buf_prob = []
                buf_time = []
                
            last_action_target = None

        ## reward
        reward.append(CalcReward(log[frame_num-1], log[frame_num]))

    ax[0].set_xlim(0, (len(log) + 1.0) * data.get("delta_t"))
    ax[0].set_ylabel("vehicle speed [m/s]")
    ax[1].set_xlim(0, (len(log) + 1.0) * data.get("delta_t"))
    ax[1].set_ylim(0, 1.0)
    ax[1].set_ylabel("risk probs")
    ax[0].annotate(sum(reward), xy=[len(log), 10], size=10, color="black")
    ax2.set_ylim(0, 1.0)
    ax2.set_ylabel("intervention request")
    ax2.set_xlabel("travel distance [m]")

    plt.show()


#################################
# print("summary")
#################################
elif len(sys.argv) > 2:

    df = pd.DataFrame(columns = ["policy", "risk_num", "travel_time", "total_fuel_consumption", "mean_fuel_consumption", "dev_accel", "mean_speed", "risk_omission", "ambiguity_omission", "request_time", "reward"])
    request_target_prob_count = {"DESPOT":[0] * 10, "MYOPIC":[0]*10, "EGOISTIC":[0]*10}
    risk_prob_count = {"DESPOT":[0] * 10, "MYOPIC":[0]*10, "EGOISTIC":[0]*10}

    fig, ax = plt.subplots(1, 1, tight_layout=True)

    for file in sys.argv[1:]:
        print(file)
        with open(file, "r") as f:
            data = json.load(f)

        log = data.get("log")

        policy = re.findall("([A-Z]*)\d", file)[0]
        risk_num = float(re.findall("([\d.]*)_", file)[0]) * 2 * 500
        travel_time = 0.0
        fuel_consumption = [] 
        accel = []
        speed = []
        risk_omission = [] 
        ambiguity_omission = [] 
        request_time = 0.0
        request_history = []
        reward = [0]

        last_ego_position = 0.0

        if log[0].get("risks") is not None:
            for risk in log[0].get("risks"):
                if risk.get("prob") == 1.0:
                    risk_prob_count.get(policy)[-1] += 1
                else:
                    risk_prob_count.get(policy)[math.floor(risk.get("prob")*10)] += 1

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
                ## update request-target_prob list
                target = frame.get("action_target")
                if target not in request_history:
                    for risk in log[0].get("risks"):
                        if target == risk.get("id"):
                            if risk.get("prob") == 1.0:
                                request_target_prob_count.get(policy)[-1] += 1
                            else:
                                request_target_prob_count.get(policy)[math.floor(risk.get("prob")*10)] += 1
                            break

                request_history.append(target)

            ## risk ambiguity omission and risk omission
            for risk in frame.get("risks"):
                position = risk.get("lane_position") if risk.get("id")[0] != "-" else 500.0 - risk.get("lane_position")
                if last_ego_position < position <= frame.get("lane_position"):

                    ## risk ambiguity
                    ambiguity_omission.append(0.5 - abs(0.5 - risk.get("prob")))

                    ## passing speed to RISK obstacle
                    if risk.get("hidden"):
                        risk_omission.append(log[frame_num-1].get("speed"))

            ## calculate reward
            reward.append(CarcReward(log[frame_num-1], log[frame_num]))

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

        buf_df = pd.DataFrame([[policy, risk_num, travel_time, total_fuel_consumption, mean_fuel_consumption, dev_accel, mean_speed, mean_risk_omission, mean_ambiguity_omission, request_time, total_reward]], columns=df.columns)
        df = pd.concat([df, buf_df], ignore_index=True)


    print(df["policy"])
    sns.lineplot(data=df, x="risk_num", y="travel_time", hue="policy", markers=True)
    plt.show()
    sns.lineplot(data=df, x="risk_num", y="total_fuel_consumption", hue="policy", markers=True)
    plt.show()
    sns.lineplot(data=df, x="risk_num", y="mean_fuel_consumption", hue="policy", markers=True)
    plt.show()
    sns.lineplot(data=df, x="risk_num", y="dev_accel", hue="policy", markers=True)
    plt.show()
    sns.lineplot(data=df, x="risk_num", y="mean_speed", hue="policy", markers=True)
    plt.show()
    sns.lineplot(data=df, x="risk_num", y="risk_omission", hue="policy", markers=True)
    plt.show()
    sns.lineplot(data=df, x="risk_num", y="ambiguity_omission", hue="policy", markers=True)
    plt.show()
    sns.lineplot(data=df, x="risk_num", y="request_time", hue="policy", markers=True)
    plt.show()
    sns.lineplot(data=df, x="risk_num", y="total_reward", hue="policy", markers=True)


    for policy in risk_prob_count.keys():
        for i in range(0, len(risk_prob_count.get(policy))):
            print(policy, i, len(risk_prob_count))
            if risk_prob_count.get(policy)[i] > 0:
                request_target_prob_count[policy][i] = request_target_prob_count[policy][i] / risk_prob_count[policy][i]

    fig, ax = plt.subplots(1,3, tight_layout=True)
    sns.barplot(x=np.arange(0.0, 1.0, 0.1), y=request_target_prob_count.get("DESPOT"), ax=ax[0])
    sns.barplot(x=np.arange(0.0, 1.0, 0.1), y=request_target_prob_count.get("MYOPIC"), ax=ax[1])
    sns.barplot(x=np.arange(0.0, 1.0, 0.1), y=request_target_prob_count.get("EGOISTIC"), ax=ax[2])
    for a in ax:
        a.set_xticklabels([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
        a.set_ylim(0.0, 1.0)
        a.set_xlabel("risk probability")
        a.set_ylabel("intervention request rate")

    plt.show()
