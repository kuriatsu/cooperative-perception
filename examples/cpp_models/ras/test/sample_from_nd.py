#!/bin/python3
# -*-coding:utf-8-*-

import matplotlib.pyplot as plt
import numpy as np
import random
import math

def nd(x, u, si):
    return np.exp(-(x-u)**2/(2*si))/(2*np.pi*si)**0.5


u = 0.7
si = 0.1
data = np.random.normal(u, si, size=10000)
plt.hist(data, bins=50, color="#ff7f00", alpha=0.5)
data = np.random.normal((1.0-u), si, size=10000)
plt.hist(data, bins=50, color="#ff7f00", alpha=0.5)
plt.xlim([0.0, 1.0])
plt.xlabel("likelihood")
plt.ylabel("count")
plt.show()

def operator_model(time):
    min_time = 3.0
    min_acc = 0.5
    max_acc = 0.9 
    slope = 0.2
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
plt.show()

