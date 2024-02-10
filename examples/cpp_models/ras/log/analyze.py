import matplotlib.pyplot as plt
import json
import sys
import numpy as np
import seaborn as sns
import math
import pandas as pd
import re
from mpl_toolkits.mplot3d import Axes3D
import ray

palette = {"REFERENCE": "navy","NOREQUEST": "turquoise", "MYOPIC": "green", "MYOPIC_PLUS": "yellowgreen", "MYOPIC_CONSERVATIVE": "olive", "OURS": "orangered", "OURS_LESS_REQUEST": "indigo", "OURS_LESS_REQUEST_PLUS": "fuchsia"}
LANE_LENGTH = 500

def CalcReward(state_prev, state_curr, policy):
    reward = 0
    for risk in state_curr["risks"]:
        if state_prev["lane_position"] < risk["lane_position"] <= state_curr["lane_position"]:

            if risk["hidden"] == "RISK":
                reward += (state_curr["speed"] - 2.8) / (11.2 - 2.8) * -100
            else:
                reward += (11.2 - state_curr["speed"]) / (11.2 - 2.8) * -100
            
            # if risk["pred"] == "RISK":
            #     reward += (state_curr["speed"] - 2.8) / (11.2 - 2.8) * -100

    ## request penalty
    # if state_prev["action"] == "REQUEST" and (state_prev["action"] != "REQUEST" or state_prev["action_target"] != state_curr["action_target"]):
    #     reward += -1

    ## deceleration penalty
    deceleration = state_prev["speed"] - state_prev["speed"]
    reward += -100 if deceleration < -0.3 * 9.8 else 0.0

    ## driving efficiency 
    # reward += (state_curr["speed"] - 2.8) / (11.2 - 2.8) * 1;

    ## intervention request
    if state_curr["action"] == "REQUEST" and (state_prev["action_target"] != state_curr["action_target"]):
        if policy == "OURS_LESS_REQUEST":
            reward += -1
        elif policy == "OURS_LESS_REQUEST_PLUS":
            reward += -10

    ## when operator mistake
    if state_prev["action"] == "REQUEST" and (state_prev["action_target"] != state_curr["action_target"]):
        for risk in state_curr["risks"]:
            if risk["id"] == state_prev["action_target"]:
                reward += -100 if risk["hidden"] != risk["pred"] else 0.0

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

if len(sys.argv) == 2 and sys.argv[1].endswith("json"):
    fig, ax = plt.subplots(2, 1, tight_layout=True)
    ax2 = ax[0].twinx()

    with open(sys.argv[1], "r") as f:
        data = json.load(f)


    log = data.get("log")
    policy = re.findall("([A-Z]*)\d", sys.argv[1])[0]

    initial_risk_id_prob = {} 
    risk_id_index_for_color = {}
    risk_num = len(log[0].get("risks"))
    for risk in log[0].get("risks"):
        initial_risk_id_prob[risk.get("id")] = risk.get("prob")
    color_map = plt.get_cmap("Set3")

    ##########################
    # print("risk position and prob")
    ##########################
    reserved_time_list = []
    for i, id in enumerate(initial_risk_id_prob.keys()):
        risk_id_index_for_color[id] = i
        elapsed_time = 0.0
        is_crossed = False
        last_prob = None
        last_obs = None
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

                    if policy == "OURS":
                        crossing_prob = last_prob
                    elif policy.startswith("MYOPIC"):
                        if risk.get("type") == "hard":
                            crossing_prob = 0.8
                        else:
                            crossing_prob = 0.95

                        if not risk.get("pred"):
                            crossing_prob = 1.0 - crossing_prob

                    else:
                        crossing_prob = initial_risk_id_prob[id]

                    if crossing_prob < 0.1: 
                        crossing_prob += 0.02 
                    elif crossing_prob > 0.9:
                        crossing_prob -= 0.02 


                    if risk["hidden"]:
                        ax2.bar(crossing_time, 1.0, color=color_map(i/len(initial_risk_id_prob)))
                        ax2.bar(crossing_time, crossing_prob, color="white", width=1.0)
                    else:
                        ax2.bar(crossing_time, crossing_prob, color=color_map(i/len(initial_risk_id_prob)))
                    ax2.annotate(f"{id}+({risk['type']})", xy=[crossing_time, crossing_prob], size=10, color= "red" if risk["hidden"] else "black")

                    reserved_time_list.append(crossing_time)
                    continue

            last_obs = frame.get("obs")


        ## no crossing means the target are at the end of the course
        if not is_crossed:
            crossing_time = elapsed_time
            while True:
                if crossing_time in reserved_time_list: 
                    crossing_time += 1.0
                else:
                    break
            ax2.bar(crossing_time, last_prob, color=color_map(i/len(initial_risk_id_prob)))
            print(f"{id} doesn't crossed time: {crossing_time} prob : {last_prob}")

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
    ax[0].text(elapse_time-20, 14.0, f"travel time: {elapse_time:.1f} s", size=10, color="black")
    ax[0].text(elapse_time-20, 13.0, f"average speed: {sum(ego_vel)/len(ego_vel):.1f} m/s", size=10, color="black")
    ax[0].set_xlim(0, (len(log) + 1.0) * data.get("delta_t"))
    ax[0].set_ylim(0, 12.0)
    ax[0].set_ylabel("vehicle speed [m/s]")
    ax2.set_ylim(0, 1.0)
    ax2.set_ylabel("risk probability")


    ##########################
    # print("intervention request")
    ##########################
    last_action_target = None
    last_observation = log[0].get("obs")
    request_time_target = 0
    buf_prob = []
    buf_time = []
    reward = [0]
    request_time = 0

    ## first log
    if log[0].get("action") == "REQUEST":
        target = log[0].get("action_target")
        for risk in log[0].get("risks"):
            if target == risk.get("id"):
                buf_prob.append(risk.get("prob"))
                buf_time.append(0.0)
                break
        last_action_target = target
        request_time_target = 1

    ## from second log
    for frame_num in range(1, len(log)):
        elapsed_time = frame_num * data.get("delta_t") 
        frame = log[frame_num]
        target = frame.get("action_target")

        if frame.get("action") == "REQUEST":
            request_time += data.get("delta_t")
            if last_action_target == target:
                request_time_target += 1
            else:
                request_time_target = 1

            ## keep intervention request
            if policy == "OURS":
                if last_action_target == target:
                    for risk in frame.get("risks"):
                        if target == risk.get("id"):
                            buf_prob.append(risk.get("prob"))
                            buf_time.append(elapsed_time)
                            break

            elif policy.startswith("MYOPIC"):
                if last_action_target == target:
                    prob = buf_prob[0] 
                    print(request_time_target)
                    for risk in frame.get("risks"):
                        if target == risk.get("id"):
                            if risk.get("type") == "hard": 
                                prob = min(0.8, 0.65+request_time_target*0.075)
                            if risk.get("type") == "easy": 
                                prob = min(0.95, 0.90+request_time_target*0.025)

                            if not risk.get("pred"):
                                prob = 1.0 - prob

                            break

                    buf_prob.append(prob)
                    buf_time.append(elapsed_time)
            else:
                if last_action_target == target:
                    for risk in frame.get("risks"):
                        if target == risk.get("id"):
                            buf_prob.append(buf_prob[0])
                            buf_time.append(elapsed_time)
                            break


            ## request 1 -> requet 2  
            ## plot last intervention request to 1 
            if last_action_target != target and last_action_target is not None:
                ax[1].plot(buf_time, buf_prob, linestyle="-", color=color_map(risk_id_index_for_color[last_action_target]/len(initial_risk_id_prob)))
                ax[1].plot(buf_time[1:], buf_prob[1:], marker=".", linestyle="", color=color_map(risk_id_index_for_color[last_action_target]/len(initial_risk_id_prob)))
                ax[1].plot(buf_time[0], buf_prob[0], marker="x", linestyle="", color=color_map(risk_id_index_for_color[last_action_target]/len(initial_risk_id_prob)))

            ## start intervention request
            ## 1. clear log log 
            ## 2. save initial prob state and prob state after first intervention request
            if last_action_target != target:
                buf_prob = []
                buf_time = []
                if policy == "OURS":
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

                elif policy.startswith("MYOPIC"):
                    prob = 0.0 
                    for risk in log[frame_num-1].get("risks"):
                        if target == risk.get("id"):
                            buf_prob.append(risk.get("prob"))
                            buf_time.append(elapsed_time-data.get("delta_t"))
                            break
                    for risk in frame.get("risks"):
                        if target == risk.get("id"):
                            if risk.get("type") == "hard": 
                                prob = 0.65
                            elif risk.get("type") == "easy": 
                                prob = 0.90

                            if not risk.get("pred"):
                                prob = 1.0 - prob

                            break

                    buf_prob.append(prob)
                    buf_time.append(elapsed_time)

                else:
                    for risk in log[frame_num-1].get("risks"):
                        if target == risk.get("id"):
                            buf_prob.append(risk.get("prob"))
                            buf_time.append(elapsed_time-data.get("delta_t"))
                            break
                    for risk in frame.get("risks"):
                        if target == risk.get("id"):
                            buf_prob.append(buf_prob[0])
                            buf_time.append(elapsed_time)
                            break

                ax[1].annotate(f"{target}+({risk['type']})", xy=[buf_time[0], buf_prob[0]], size=10, color="red" if risk["hidden"] else "black")
            last_action_target = target
            last_observation = frame["obs"] 
                
        else:
            ## finish intervention request 
            ## plot last intervention request log and clear log
            if last_action_target is not None:
                ax[1].plot(buf_time, buf_prob, linestyle="-", color=color_map(risk_id_index_for_color[last_action_target]/len(initial_risk_id_prob)))
                ax[1].plot(buf_time[1:], buf_prob[1:], marker=".", linestyle="", color=color_map(risk_id_index_for_color[last_action_target]/len(initial_risk_id_prob)))
                ax[1].plot(buf_time[0], buf_prob[0], marker="x", linestyle="", color=color_map(risk_id_index_for_color[last_action_target]/len(initial_risk_id_prob)))
                buf_prob = []
                buf_time = []

            ## no intervention request
            else:
                buf_prob = []
                buf_time = []
                
            last_action_target = None
            request_time_target = 0

        ## reward
        reward.append(CalcReward(log[frame_num-1], log[frame_num], policy))

    print(sum(reward))
    ax[0].text(elapse_time-20, 12.0, f"reward: {sum(reward):.1f}", size=10, color="black")
    ax[1].text(elapse_time-20, 1.02, f"request time: {request_time} s", size=10, color="black")
    ax[1].set_xlim(0, (len(log) + 1.0) * data.get("delta_t"))
    ax[1].set_ylim(0, 1.0)
    ax[1].set_ylabel("intervention request timing")
    ax[1].set_xlabel("travel distance [m]")

    filename = re.findall("([.\w]*).json", sys.argv[1])[0]

    plt.savefig(f"{filename}.svg", transparent=True)


#################################
# print("summary")
#################################
elif len(sys.argv) > 2 and sys.argv[1].endswith("json"):

    @ray.remote
    def process_raw_data(file):
        print(file)
        with open(file, "r") as f:
            data = json.load(f)

        log = data.get("log")

        policy = re.findall("([A-Z_]*)\d", file)[0]
        date = re.findall("_([\d]*)", file)[-1]
        risk_num = float(re.findall("([\d.]*)_", file)[-1]) * 2.0 * 500.0
        travel_time = 0.0
        fuel_consumption = [] 
        accel = []
        speed = []
        risk_omission = [] 
        ambiguity_omission = [] 
        request_time = 0.0
        reward = [0]
        correct_pred = 0
        passed_target = 0
        last_request_target = None
        request_count = 0

        buf_target_count = pd.DataFrame(columns = ["id", "prob", "is_requsted", "policy", "risk_num", "date"]) 
        # risk_prob_count = {"OURS":[0] * 10, "MYOPIC":[0]*10, "NOREQUEST":[0]*10, "REFERENCE":[0]*10}
        buf_prob_speed_count = pd.DataFrame(columns = ["policy", "date", "risk_num", "prob", "hidden", "pred", "speed", "distance_pred_speed", "distance_prob_speed", "distance_risk_speed", "easy_rate"])
        buf_recog_evaluation = pd.DataFrame(columns = ["hidden", "pred", "prob"])

        easy_rate = data.get("obstacle_type_rate").get("easy")
        if easy_rate == 0.0 :
            easy_rate = data.get("obstacle_type_rate").get("easy_plus")


        last_ego_position = 0.0

        ## log 0 data
        if log[0].get("risks") is not None:
            for risk in log[0].get("risks"):

                ## add recog evaluation
                buf = pd.DataFrame([[risk.get("hidden"), risk.get("pred"), risk.get("prob")]], columns=buf_recog_evaluation.columns)
                buf_recog_evaluation = pd.concat([buf_recog_evaluation, buf], ignore_index=True)

                quantized_prob = 0.0
                if risk.get("prob") == 1.0:
                    quantized_prob = 0.9 
                else:
                    quantized_prob = math.floor(risk.get("prob")*10) * 0.1 
                buf = pd.DataFrame([[risk.get("id"), quantized_prob, False, policy, risk_num, date]], columns=buf_target_count.columns)
                buf_target_count = pd.concat([buf_target_count, buf], ignore_index=True)

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
                ## flag target count "is_requested"
                target_index = buf_target_count.index[(buf_target_count.id == frame.get("action_target")) & (buf_target_count.policy == policy) & (buf_target_count.risk_num == risk_num) & (buf_target_count.date == date)].tolist()[0]
                buf_target_count.iloc[target_index, 2] = True
                
                if last_request_target != frame.get("action_target"):
                    request_count += 1

            last_request_target = frame.get("action_target") if frame.get("action") == "REQUEST" else None

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
                    if policy == "REFERENCE":
                        if risk.get("hidden"):
                            distance_pred_speed = abs(log[frame_num-1].get("speed")/11.2)
                        else:
                            distance_pred_speed = abs((11.2 - log[frame_num-1].get("speed"))/11.2)
                    else:
                        if risk.get("pred"):
                            distance_pred_speed = abs(log[frame_num-1].get("speed")/11.2)
                        else:
                            distance_pred_speed = abs((11.2 - log[frame_num-1].get("speed"))/11.2)

                    ## distance from ideal prob-speed 
                    distance_prob_speed = 0.0
                    if policy == "REFERENCE":
                        if risk.get("hidden"):
                            distance_prob_speed = abs(log[frame_num-1].get("speed")/11.2)
                        else:
                            distance_prob_speed = abs((11.2 - log[frame_num-1].get("speed"))/11.2)

                    else:
                        if risk.get("prob") >= 0.5:
                            distance_prob_speed = (1.0 - risk.get("prob"))**2 + (log[frame_num-1].get("speed")/11.2)**2
                            distance_prob_speed = distance_prob_speed**0.5
                        else:
                            distance_prob_speed = (risk.get("prob"))**2 + ((11.2 - log[frame_num-1].get("speed"))/11.2)**2
                            distance_prob_speed = distance_prob_speed**0.5

                    ## distance from ideal speed based on hidden risk
                    distance_risk_speed = 0.0
                    if risk.get("hidden"):
                        distance_risk_speed = log[frame_num-1].get("speed")/11.2
                    else:
                        distance_risk_speed = abs((11.2 - log[frame_num-1].get("speed"))/11.2)

                    ## accuracy
                    if policy == "REFERENCE":
                        correct_pred += 1
                    else:
                        correct_pred += (risk.get("hidden") == risk.get("pred"))

                    passed_target += 1


                    buf = pd.DataFrame([[policy, date, risk_num, int(risk.get("prob")*10)*0.1, risk.get("hidden"), risk.get("pred"), log[frame_num-1].get("speed"), distance_pred_speed, distance_prob_speed, distance_risk_speed, easy_rate]], columns=buf_prob_speed_count.columns)
                    buf_prob_speed_count = pd.concat([buf_prob_speed_count, buf], ignore_index=True)

            ## calculate reward
            reward.append(CalcReward(log[frame_num-1], log[frame_num], policy))

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

        buf_df = pd.DataFrame([[policy, date, risk_num, travel_time, total_fuel_consumption, mean_fuel_consumption, dev_accel, mean_speed, mean_risk_omission, mean_ambiguity_omission, request_time, total_reward, acc, easy_rate, request_count]], columns=["policy", "date", "risk_num", "travel_time", "total_fuel_consumption", "mean_fuel_consumption", "dev_accel", "mean_speed", "risk_omission", "ambiguity_omission", "request_time", "total_reward", "acc", "easy_rate", "request_count"])

        return buf_df, buf_prob_speed_count, buf_target_count, buf_recog_evaluation



    df = pd.DataFrame(columns = ["policy", "date", "risk_num", "travel_time", "total_fuel_consumption", "mean_fuel_consumption", "dev_accel", "mean_speed", "risk_omission", "ambiguity_omission", "request_time", "total_reward", "acc", "easy_rate", "request_count"])
    target_count = pd.DataFrame(columns = ["id", "prob", "is_requsted", "policy", "risk_num", "date"]) 
    # risk_prob_count = {"OURS":[0] * 10, "MYOPIC":[0]*10, "NOREQUEST":[0]*10, "REFERENCE":[0]*10}
    prob_speed_count = pd.DataFrame(columns = ["policy", "date", "risk_num", "prob", "hidden", "pred", "speed", "distance_pred_speed", "distance_prob_speed", "distance_risk_speed", "easy_rate"])
    recog_evaluation = pd.DataFrame(columns = ["hidden", "pred", "prob"])
    ray.init()
    processes = []
    for file in sys.argv[1:]:
        processes.append(process_raw_data.remote(file)) 

    results = ray.get(processes)
    for res in results:
        df = pd.concat([df, res[0]], ignore_index=True)
        prob_speed_count = pd.concat([prob_speed_count, res[1]], ignore_index=True)
        target_count = pd.concat([target_count, res[2]], ignore_index=True)
        recog_evaluation = pd.concat([recog_evaluation, res[3]], ignore_index=True)



    fig, ax = plt.subplots(1, 1, tight_layout=True)
    sns.lineplot(data=df, x="risk_num", y="travel_time", hue="policy", style="policy", markers=True, palette=palette)
    ax.set_ylim([0.0, 120.0])
    plt.savefig("travel_time.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="total_fuel_consumption", hue="policy", style="policy", markers=True, palette=palette)
    plt.savefig("total_fuel_consumption.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="mean_fuel_consumption", hue="policy", style="policy", markers=True, palette=palette)
    plt.savefig("mean_fuel_consumption.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="dev_accel", hue="policy", style="policy", markers=True, palette=palette)
    plt.ylim([0.0, 12.0])
    plt.savefig("accel.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="mean_speed", hue="policy", style="policy", markers=True, palette=palette)
    plt.ylim([0.0, 12.0])
    plt.savefig("mean_speed.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="ambiguity_omission", hue="policy", style="policy", markers=True, palette=palette)
    plt.savefig("ambiguity_omission.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="request_time", hue="policy", markers=True, style="policy", palette=palette)
    plt.ylim([0.0, 90.0])
    plt.savefig("request_time.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="request_count", hue="policy", markers=True, style="policy", palette=palette)
    plt.ylim([0.0, 30.0])
    plt.savefig("request_count.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="total_reward", hue="policy", style="policy", markers=True, palette=palette)
    plt.savefig("reward.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="risk_num", y="acc", hue="policy", style="policy", markers=True, palette=palette)
    plt.ylim([0.0, 1.0])
    plt.savefig("total_acc.svg", transparent=True)
    plt.clf()


    ## risk prob - intervention request
    request_rate = pd.DataFrame(columns=("policy", "risk_num", "prob", "rate"))
    for rn in target_count.risk_num.drop_duplicates():
        for pol in target_count.policy.drop_duplicates():
            for pro in target_count.prob.drop_duplicates():
                target = target_count[(target_count.policy==pol) & (target_count.risk_num==rn) & (target_count.prob==pro)]
                if len(target) > 0:
                    buf_request_rate = pd.DataFrame([[pol, rn, pro, target.is_requsted.sum()/len(target)]], columns=request_rate.columns) 
                    request_rate = pd.concat([request_rate, buf_request_rate], ignore_index=True)

    fig, ax = plt.subplots(tight_layout=True)
    sns.lineplot(x="policy", y="rate", data=request_rate, hue="risk_num", markers=True, ax=ax)
    ax.set_ylim(0.0, 1.0)
    ax.set_xlabel("risk probability")
    ax.set_ylabel("intervention request rate")

    plt.savefig("request_rate.svg", transparent=True, bbox_inches="tight")
    request_rate.to_csv("request_rate.csv")

    # speed - prob scatter plot
    fig, ax = plt.subplots(tight_layout = True)
    # sns.lineplot(data=prob_speed_count, x="policy", y="speed", hue="risk_num", style="hidden", markers=True, ax=ax, label="_nolegend_")
    sns.lineplot(data=prob_speed_count, x="policy", y="speed", hue="risk_num", style="hidden", markers=True, ax=ax)

    plt.savefig("pass_speed_ypolicy.svg", transparent=True)
    prob_speed_count.to_csv("prob_speed_count.csv")
    plt.clf()
    
    # speed - prediction distance plot
    fig, ax = plt.subplots(tight_layout = True)
    sns.lineplot(data=prob_speed_count, x="risk_num", y="distance_pred_speed", hue="policy", style="policy", markers=True, ax=ax, palette=palette)

    plt.savefig("speed_pred_distance.svg", transparent=True)

    # speed - probability distance plot
    fig, ax = plt.subplots(tight_layout = True)
    sns.lineplot(data=prob_speed_count, x="risk_num", y="distance_prob_speed", hue="policy", style="policy", markers=True, ax=ax, palette=palette)

    plt.savefig("speed_prob_distance.svg", transparent=True)

    # speed - hidden risk distance plot
    fig, ax = plt.subplots(tight_layout = True)
    sns.lineplot(data=prob_speed_count, x="risk_num", y="distance_risk_speed", hue="policy", style="policy", markers=True, ax=ax, palette=palette)

    plt.savefig("speed_risk_distance.svg", transparent=True)

    # speed - hidden risk speed plot
    fig, ax = plt.subplots(tight_layout = True)
    sns.lineplot(data=prob_speed_count, x="risk_num", y="speed", hue="policy", style="hidden", markers=True, ax=ax, palette=palette)
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

    ## horizontal axis = easy rate
    sns.lineplot(data=df, x="easy_rate", y="acc", hue="policy", style="policy", markers=True, palette=palette)
    plt.ylim([0.0, 1.0])
    plt.savefig(f"total_acc_easy_rate.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="easy_rate", y="request_time", hue="policy", style="policy", markers=True, palette=palette)
    ax.set_ylim(0.0, 90.0)
    plt.savefig(f"request_time_easy_rate.svg", transparent=True)
    plt.clf()

    sns.lineplot(data=df, x="easy_rate", y="request_count", hue="policy", style="policy", markers=True, palette=palette)
    ax.set_ylim(0.0, 30.0)
    plt.savefig(f"request_count_easy_rate.svg", transparent=True)
    plt.clf()

    fig, ax = plt.subplots(tight_layout = True)
    sns.lineplot(data=prob_speed_count, x="easy_rate", y="speed", hue="policy", style="hidden", markers=True, ax=ax, palette=palette)
    ax.set_ylim(0.0, 12.0)
    plt.savefig(f"pass_speed_easy_rate.svg", transparent=True)
    plt.clf()

    
    ## plot 3d 
    fig = plt.figure(figsize=(9, 9), facecolor="w")
    ax = fig.add_subplot(111, projection="3d")
    for policy, color in palette.items():
        buf_df = prob_speed_count.query("hidden == False & policy==@policy")
        x = []
        y = []
        for x_d in buf_df["risk_num"].drop_duplicates():
            x.append(x_d)
        for y_d in buf_df["easy_rate"].drop_duplicates():
            y.append(y_d)

        x.sort()
        y.sort()
        z = []
        for x_d in x:
            z_r = []
            for y_d in y:
                z_r.append(buf_df[(buf_df.risk_num == x_d) & (buf_df.easy_rate == y_d)].speed.mean())
            z.append(z_r)

        x, y = np.meshgrid(x, y)

        ax.plot_surface(np.array(x), np.array(y), np.array(z), color = color, alpha = 0.8)

    ax.set_zlim(0.0, 12.0)
    plt.savefig("pass_speed_true_3d.svg", transparent=True)
    plt.clf()

    fig = plt.figure(figsize=(9, 9), facecolor="w")
    ax = fig.add_subplot(111, projection="3d")
    for policy, color in palette.items():
        buf_df = prob_speed_count.query("hidden == True & policy==@policy")
        x = []
        y = []
        for x_d in buf_df["risk_num"].drop_duplicates():
            x.append(x_d)
        for y_d in buf_df["easy_rate"].drop_duplicates():
            y.append(y_d)

        x.sort()
        y.sort()
        z = []
        for x_d in x:
            z_r = []
            for y_d in y:
                z_r.append(buf_df[(buf_df.risk_num == x_d) & (buf_df.easy_rate == y_d)].speed.mean())
            z.append(z_r)

        x, y = np.meshgrid(x, y)

        ax.plot_surface(np.array(x), np.array(y), np.array(z), color = color, alpha = 0.8)

    ax.set_zlim(0.0, 12.0)
    plt.savefig("pass_speed_true_3d.svg", transparent=True)
    plt.clf()


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

    sns.lineplot(x=np.arange(0.0, 1.0, 0.1), y=correct/count, ax=ax[0], marker="o")
    sns.barplot(x=np.arange(0.0, 1.0, 0.1), y=count, ax=ax[1])
    ax[0].set_ylim(0.0, 1.0)
    ax[0].set_xticks([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
    ax[0].set_xticklabels([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])


elif sys.argv[1].endswith("csv"):
    
    for file in sys.argv[1:]:
        df = pd.read_csv(file)

        if "speed" in df.columns:
            # speed - prob scatter plot
            fig, ax = plt.subplots(tight_layout = True)
            sns.lineplot(data=df, x="scenario", y="speed", hue="risk_num", style="hidden", markers=True, ax=ax, label="_nolegend_")
            plt.savefig("pass_speed_ypolicy.svg", transparent=True)

        elif "rate" in df.columns:

            fig, ax = plt.subplots(tight_layout=True)
            sns.lineplot(x="scenario", y="rate", data=df, hue="risk_num", markers=True, ax=ax)
            ax.set_ylim(0.0, 1.0)
            ax.set_xlabel("risk probability")
            ax.set_ylabel("intervention request rate")

            plt.savefig("request_rate.svg", transparent=True, bbox_inches="tight")
