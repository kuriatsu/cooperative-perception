#! /usr/bin/python3
# -*- coding: utf-8 -*-

import pickle
import pandas as pd
import xml.etree.ElementTree as ET
import math
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
import csv
import glob
import os


sns.set(context='paper', style='whitegrid')
hue_order = ["traffic light", "crossing intention", "trajectory"]
eps=0.01
tl_black_list = [
"3_3_96tl",
"3_3_102tl",
"3_4_107tl",
"3_4_108tl",
"3_5_112tl",
"3_5_113tl",
"3_5_116tl",
"3_5_117tl",
"3_5_118tl",
"3_5_119tl",
"3_5_122tl",
"3_5_123tl",
"3_5_126tl",
"3_5_127tl",
"3_6_128tl",
"3_6_137tl",
"3_7_142tl",
"3_8_153tl",
"3_8_160tl",
"3_9_173tl",
"3_9_174tl",
"3_9_179tl",
"3_10_185tl",
"3_10_188tl",
"3_11_205tl",
"3_12_218tl",
"3_12_221tl",
"3_15_241tl",
"3_16_256tl",
"3_16_257tl",
]
opposite_anno_list = ["3_16_259tl", "3_16_258tl", "3_16_249tl"]

log_data = None
data_path = "/run/media/kuriatsu/KuriBuffaloPSM/PIE/experiment/PIE_202203/log202203"
for file in glob.glob(os.path.join(data_path, "log*.csv")):
    buf = pd.read_csv(file)
    filename =file.split("/")[-1]
    count = int(filename.replace("log_data_", "").split("_")[-1].replace(".csv", ""))
    print("{}".format(filename))

    if count in [0, 1, 2]:
        print("skipped")
        continue

    ## true positive, false positive, true negative, false negative
    trial = filename.split("_")[-1].replace(".csv", "")
    buf["subject"] = filename.replace("log_data_", "").split("_")[0]
    buf["task"] = filename.replace("log_data_", "").split("_")[1]
    correct_list = [] ## true state
    response_list = [] ## responce tp, fp, tn, fn
    for idx, row in buf.iterrows():
        ## when annotation of TL was wrong, flip evaluation
        if row.id in tl_black_list:
            row.last_state = -2
        if row.last_state == -1: # no intervention
            correct_list.append(-1)
            response_list.append(-1)

        elif int(row.last_state) == int(row.state):
            if row.id in opposite_anno_list:
                correct_list.append(1)
                if row.last_state == 1:
                    response_list.append(3)
                elif row.last_state == 0:
                    response_list.append(0)
                else:
                    print(f"last_state{row.last_state}, state{row.state}")
                    response_list.append(4) # ignored=4
            else:
                correct_list.append(0)
                if row.last_state == 1:
                    response_list.append(1)
                elif row.last_state == 0:
                    response_list.append(2)
                else:
                    print(f"last_state{row.last_state}, state{row.state}")
                    response_list.append(4) # ignored=4
        else:
            if row.id in opposite_anno_list:
                correct_list.append(0)
                if row.last_state == 1:
                    response_list.append(1)
                elif row.last_state == 0:
                    response_list.append(2)
                else:
                    print(f"last_state{row.last_state}, state{row.state}")
                    response_list.append(4) # ignored=4
            else:
                correct_list.append(1)
                if row.last_state == 1:
                    response_list.append(3)
                elif row.last_state == 0:
                    response_list.append(0)
                else:
                    print(f"last_state{row.last_state}, state{row.state}")
                    response_list.append(4) # ignored=4

    buf["correct"] = correct_list
    buf["response"] = response_list
    len(correct_list)
    if log_data is None:
        log_data = buf
    else:
        log_data = pd.concat([log_data, buf], ignore_index=True)

task_list = {"int": "crossing intention", "tl": "traffic light", "traj":"trajectory"}
subject_data = pd.DataFrame(columns=["subject", "task", "acc", "int_length", "missing"])
for subject in log_data.subject.drop_duplicates():
    for task in log_data.task.drop_duplicates():
        for length in log_data.int_length.drop_duplicates():
            target = log_data[(log_data.subject == subject) & (log_data.task == task) & (log_data.int_length == length)]
            # acc = len(target[target.correct == 1])/(len(target))
            acc = len(target[target.correct == 1])/(len(target[target.correct == 0]) + len(target[target.correct == 1])+eps)
            missing = len(target[target.correct == -1])/(len(target[target.correct != -2])+eps)
            buf = pd.DataFrame([(subject, task_list.get(task), acc, length, missing)], columns=subject_data.columns)
            subject_data = pd.concat([subject_data, buf])
            
# subject_data.acc = subject_data.acc * 100
# subject_data.missing = subject_data.missing * 100
# sns.barplot(x="task", y="acc", hue="int_length", data=subject_data, ci="sd")
# sns.barplot(x="task", y="acc", data=subject_data, ci="sd")

for task in subject_data.task.drop_duplicates():
    for length in subject_data.int_length.drop_duplicates():
        acc = subject_data[(subject_data.task==task) & (subject_data.int_length == length)].acc.mean()
        print(f"task : {task}, length : {length}, acc = {acc}")

fig, ax = plt.subplots()
print_data = subject_data[subject_data.task!="trajectory"]
sns.pointplot(x="int_length", y="acc", data=print_data, hue="task", ax=ax, capsize=0.1)
ax.set_ylim(0.0, 1.0)
ax.set_xlabel("Request time [s]", fontsize=14)
ax.set_ylabel("Accuracy [%]", fontsize=14)
ax.tick_params(labelsize=14)
ax.legend(fontsize=14)
plt.savefig("accuracy_pie_experiment.svg")
plt.clf()


def nd(x, u, si):
    return np.exp(-(x-u)**2/(2*si))/(2*np.pi*si)**0.5



fig, ax = plt.subplots(1, 2, tight_layout=True)
ax2 = ax[0].twinx()
ax3 = ax[1].twinx()

#pie_result = pd.read_csv("/run/media/kuriatsu/KuriBuffaloPSM/PIE/experiment/PIE_202203/pie_predict_result_valid.csv")
pie_result = pd.read_csv("/run/media/kuriatsu/KuriBuffaloPSM/PIE/experiment/PIE_202203/pie_predict.csv")
ax2.hist(pie_result["likelihood"], bins=50, alpha=0.5, label="prediction")
u = pie_result["likelihood"].mean()
si = pie_result["likelihood"].std()
print(f"pie result mean={u}, si={si}")
data = np.random.normal(u, si, size=10000)
ax[0].hist(data, bins=50, color="#ff7f00", alpha=0.5, label="simulation")

tlr_result = pd.read_csv("/run/media/kuriatsu/KuriBuffaloPSM/PIE/experiment/PIE_202203/tlr_result.csv")
ax3.hist(tlr_result["likelihood"], bins=50, alpha=0.5, label="prediction")
u = tlr_result["likelihood"].mean()
si = tlr_result["likelihood"].std()
print(f"tlr result mean={u}, si={si}")
data = np.random.normal(u, si, size=10000)
data = [i for i in data if i<=1.0] 
ax[1].hist(data, bins=50, color="#ff7f00", alpha=0.5, label="simulation")

fig.legend(fontsize=14)
ax[0].set_ylabel("count", fontsize=14)
ax[1].set_ylabel("count", fontsize=14)
ax[0].set_xlim([0.0, 1.0])
ax[1].set_xlim([0.0, 1.0])
ax[0].set_xlabel("likelihood", fontsize=14)
ax[1].set_xlabel("likelihood", fontsize=14)
plt.savefig("likelihood_hist.svg")
plt.clf()

def operator_model(time):
    min_time = 1.0
    min_acc = 0.65
    max_acc = 0.8
    slope = 0.03
    if time < min_time:
        return min_acc

    acc = (time-min_time) * slope + min_acc
    return acc if acc <= max_acc else max_acc

acc_data = []
for time in range(0, 10):
    acc_data.append(operator_model(time))

plt.plot(np.arange(0, 10), acc_data)
plt.xlabel("given time t_req")
plt.ylabel("accuracy")
plt.ylim([0.0, 1.0])
plt.savefig("future_model.svg")


