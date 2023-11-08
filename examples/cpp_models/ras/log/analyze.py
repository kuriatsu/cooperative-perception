import matplotlib.pyplot as plt
import json
import sys
import numpy as np
import seaborn as sns
import math
import pandas as pd
import re

palette = {"REFERENCE": "orangered","EGOISTIC": "indigo", "MYOPIC": "dodgerblue", "DESPOT": "forestgreen"}
LANE_LENGTH = 500

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
                    ax2.annotate(id, xy=[crossing_time, crossing_prob], size=10, color= "red" if risk["hidden"] else "black")
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
    ax[0].annotate(f"reward: \n{sum(reward):.1f}", xy=[len(log)-8, 1], size=10, color="black")
    ax2.set_ylim(0, 1.0)
    ax2.set_ylabel("intervention request")
    ax2.set_xlabel("travel distance [m]")

    plt.show()


#################################
# print("summary")
#################################
elif len(sys.argv) > 2:

    df = pd.DataFrame(columns = ["policy", "date", "risk_num", "travel_time", "total_fuel_consumption", "mean_fuel_consumption", "dev_accel", "mean_speed", "risk_omission", "ambiguity_omission", "request_time", "total_reward", "acc"])
    request_target_prob_count = {"DESPOT":[0] * 10, "MYOPIC":[0]*10, "EGOISTIC":[0]*10, "REFERENCE":[0]*10}
    risk_prob_count = {"DESPOT":[0] * 10, "MYOPIC":[0]*10, "EGOISTIC":[0]*10, "REFERENCE":[0]*10}
    prob_speed_count = pd.DataFrame(columns = ["policy", "date", "risk_num", "prob", "hidden", "pred", "speed", "distance_pred_speed", "distance_prob_speed", "distance_risk_speed"])
    recog_evaluation = pd.DataFrame(columns = ["hidden", "pred", "prob"])
    total_acc_df = pd.DataFrame(columns=["policy", "date", "risk_num", "acc"])


    fig, ax = plt.subplots(1, 1, tight_layout=True)

    for file in sys.argv[1:]:
        print(file)
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


                    # buf_prob_speed_count = pd.DataFrame([[policy, date, risk_num, int(risk.get("prob")*10)*0.1, log[frame_num-1].get("speed")]], columns=prob_speed_count.columns)
                    if risk.get("prob") == 1.0 and log[frame_num-1].get("speed") > 3.0:
                        print(date, risk_num, risk.get("id"), position, frame.get("lane_position"), travel_time, log[frame_num-1].get("speed"))
                    buf_prob_speed_count = pd.DataFrame([[policy, date, risk_num, risk.get("prob"), risk.get("hidden"), risk.get("pred"), log[frame_num-1].get("speed"), distance_pred_speed, distance_prob_speed, distance_risk_speed]], columns=prob_speed_count.columns)
                    prob_speed_count = pd.concat([prob_speed_count, buf_prob_speed_count], ignore_index=True)

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
        
        acc = correct_pred / passed_target if passed_target > 0 else 0.0

        buf_df = pd.DataFrame([[policy, date, risk_num, travel_time, total_fuel_consumption, mean_fuel_consumption, dev_accel, mean_speed, mean_risk_omission, mean_ambiguity_omission, request_time, total_reward, acc]], columns=df.columns)
        df = pd.concat([df, buf_df], ignore_index=True)


    sns.lineplot(data=df, x="risk_num", y="travel_time", hue="policy", markers=True, palette=palette)
    plt.savefig("travel_time.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="total_fuel_consumption", hue="policy", markers=True, palette=palette)
    plt.savefig("total_fuel_consumption.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="mean_fuel_consumption", hue="policy", markers=True, palette=palette)
    plt.savefig("mean_fuel_consumption.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="dev_accel", hue="policy", markers=True, palette=palette)
    plt.ylim([0.0, 11.2])
    plt.savefig("accel.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="mean_speed", hue="policy", markers=True, palette=palette)
    plt.ylim([0.0, 11.2])
    plt.savefig("mean_speed.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="ambiguity_omission", hue="policy", markers=True, palette=palette)
    plt.savefig("ambiguity_omission.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="request_time", hue="policy", markers=True, palette=palette)
    plt.savefig("request_time.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="total_reward", hue="policy", markers=True, palette=palette)
    plt.savefig("reward.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="acc", hue="policy", markers=True, palette=palette)
    ax.set_ylim([0.0, 1.0])
    plt.savefig("total_acc.svg", transparent=True)
    plt.clf()

    ## risk prob - intervention request
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

    plt.savefig("request_prob.svg", transparent=True, bbox_inches="tight")

    # speed - prob scatter plot
    fig, ax = plt.subplots(1, len(prob_speed_count["policy"].unique()), tight_layout = True)
    for i, policy in enumerate(prob_speed_count["policy"].unique()):
        sns.scatterplot(data=prob_speed_count[prob_speed_count["policy"]==policy], x="prob", y="speed", hue="risk_num", ax=ax[i])
        ax[i].set_title(f"{policy}")

    plt.savefig("speed_prob.svg", transparent=True)
    plt.clf()
    
    # speed - prediction distance plot
    fig, ax = plt.subplots(tight_layout = True)
    sns.lineplot(data=prob_speed_count, x="risk_num", y="distance_pred_speed", hue="policy", ax=ax, palette=palette)

    plt.savefig("speed_pred_distance.svg", transparent=True)

    # speed - probability distance plot
    fig, ax = plt.subplots(tight_layout = True)
    sns.lineplot(data=prob_speed_count, x="risk_num", y="distance_prob_speed", hue="policy", ax=ax, palette=palette)

    plt.savefig("speed_prob_distance.svg", transparent=True)

    # speed - hidden risk distance plot
    fig, ax = plt.subplots(tight_layout = True)
    sns.lineplot(data=prob_speed_count, x="risk_num", y="distance_risk_speed", hue="policy", ax=ax, palette=palette)

    plt.savefig("speed_risk_distance.svg", transparent=True)

    # speed - hidden risk speed plot
    fig, ax = plt.subplots(tight_layout = True)
    sns.lineplot(data=prob_speed_count, x="risk_num", y="speed", hue="policy", style="hidden", ax=ax, markers=True, palette=palette)
    ax.set_ylim(0.0, 12.0)
    plt.savefig("pass_speed.svg", transparent=True)
    plt.clf()
    

#    fig, ax = plt.subplots(len(prob_speed_count["risk_num"].unique()), len(prob_speed_count["policy"].unique()), tight_layout = True)
#    for i, policy in enumerate(prob_speed_count["policy"].unique()):
#        for j, risk_num in enumerate(prob_speed_count["risk_num"].unique()):
#            sns.boxplot(data=prob_speed_count[(prob_speed_count["policy"]==policy)&(prob_speed_count["risk_num"]==risk_num)], x="prob", y="speed", ax=ax[j][i])
#            ax[j][i].set_title(f"{policy}-{risk_num}")
#
#    plt.show()




    ## visualize recognition simulation 
    fig, ax = plt.subplots(1, 2, tight_layout=True)
    correct = np.array([0]*10)
    count = np.array([0]*10)
    for index, row in recog_evaluation.iterrows():
        if row["prob"] == 1.0:
            correct[-1] += (row["hidden"] == row["pred"])
            count[-1] += 1
        else:
            correct[int(row["prob"]*10)] += (row["hidden"] == row["pred"])
            count[int(row["prob"]*10)] += 1

    for index in np.where(count==0):
        count[index] = 1
        correct[index] = 1

    sns.lineplot(x=np.arange(0.0, 1.0, 0.1), y=correct/count, ax=ax[0])
    sns.barplot(x=np.arange(0.0, 1.0, 0.1), y=count, ax=ax[1])
    ax[0].set_ylim(0.0, 1.0)
    ax[0].set_xticks([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
    ax[0].set_xticklabels([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
