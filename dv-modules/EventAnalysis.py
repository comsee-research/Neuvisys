from dv import AedatFile

import numpy as np
import matplotlib.pyplot as plt
from numba import jit
# import cv2

tau = 50 # µs
t_tau = 400000 # µs
taus = np.arange(0, t_tau+1, tau)
size = (346, 260) # px
n = size[0] * size[1] # px
l = 20 # px
file_name = "/home/thomas/Vidéos/DVS_Recordings/air_hockey/air_hockey1.aedat4"

nb_event = 1000000000
freq_display = int(nb_event * 0.01) # %

def plot_spatial_correlation(spat_corr: np.array):
    fig, axes = plt.subplots(2, 2, sharex=True, figsize=(10, 8))
    fig.suptitle("Spatial correlation, \n probability of surrounding pixels firing given that a pixel will fire " + str(tau) + "µs later.", fontsize=16)
    axes[0, 0].set_title("P(On | On)")
    axes[0, 1].set_title("P(Off | On)")
    axes[1, 0].set_title("P(On | Off)")
    axes[1, 1].set_title("P(Off | Off)")
    
    i = 0
    for ax in axes.flat:
        ax.imshow(spat_corr[i])
        ax.set(xlabel='px', ylabel='px')
        ax.label_outer()
        i += 1
    plt.show()

def plot_temporal_correlation(temp_corr: np.array):
    fig, axes = plt.subplots(4, figsize=(50, 30))
    fig.suptitle("Temporal correlation, \n probability of a pixel firing given that this same pixel will fire in ranges of " + str(tau) + "µs.", fontsize=32)
    axes[0].set_title("P(On | On)", fontsize=26)
    axes[1].set_title("P(Off | On)", fontsize=26)
    axes[2].set_title("P(On | Off)", fontsize=26)
    axes[3].set_title("P(Off | Off)", fontsize=26)
    
    i = 0
    for ax in axes.flat:
        ax.plot(temp_corr[i])
        ax.set_xticks(np.linspace(0, temp_corr[0].size, 101, dtype=np.int32))
        ax.set_xticklabels(np.linspace(0, t_tau, 101, dtype=np.int32), rotation=45, fontsize=12)
        ax.set(xlabel='delay (µs)', ylabel='probability (%)')
        i += 1
    plt.show()
    
def plot_temporal_cumsum(temp_corr: np.array):
    cumsum_temp_corr = np.cumsum(temp_corr, axis=1)
    fig, axes = plt.subplots(4, figsize=(50, 30))
    fig.suptitle("Temporal correlation cumulative sum", fontsize=32)
    axes[0].set_title("P(On | On)", fontsize=26)
    axes[1].set_title("P(Off | On)", fontsize=26)
    axes[2].set_title("P(On | Off)", fontsize=26)
    axes[3].set_title("P(Off | Off)", fontsize=26)
    
    i = 0
    for ax in axes.flat:
        ax.plot(cumsum_temp_corr[i])
        ax.set_xticks(np.linspace(0, temp_corr[0].size, 101, dtype=np.int32))
        ax.set_xticklabels(np.linspace(0, t_tau, 101, dtype=np.int32), rotation=45, fontsize=12)
        ax.set(xlabel='delay (µs)', ylabel='probability (%)')
        i += 1
    plt.show()
    
@jit(nopython=True)
def spatial_correlation(x, y, polarity, timestamp, spat_corr, timestamps):
    if x > l and x < size[0] - l and y > l and y < size[1] - l:
        if polarity:
            spat_corr[0] += 1 * (timestamps[1, y-l:y+l+1, x-l:x+l+1] > timestamp - tau) # P(On | On)
            spat_corr[1] += 1 * (timestamps[0, y-l:y+l+1, x-l:x+l+1] > timestamp - tau) # P(Off | On)
        else:
            spat_corr[2] += 1 * (timestamps[1, y-l:y+l+1, x-l:x+l+1] > timestamp - tau) # P(On | Off)
            spat_corr[3] += 1 * (timestamps[0, y-l:y+l+1, x-l:x+l+1] > timestamp - tau) # P(Off | Off)
    timestamps[1*polarity, y, x] = timestamp

@jit(nopython=True)
def temporal_correlation(x, y, polarity, timestamp, temp_corr, timestamps):
    if polarity:
        for i, t in enumerate(taus):
            temp_corr[0, i] += 1 * timestamps[0, y, x] >= timestamp - t and timestamps[0, y, x] < timestamp - t + tau # P(On | On)
            temp_corr[1, i] += 1 * timestamps[1, y, x] >= timestamp - t and timestamps[1, y, x] < timestamp - t + tau # P(Off | On)
    else:
        for i, t in enumerate(taus):
            temp_corr[2, i] += 1 * timestamps[0, y, x] >= timestamp - t and timestamps[0, y, x] < timestamp - t + tau # P(On | Off)
            temp_corr[3, i] += 1 * timestamps[1, y, x] >= timestamp - t and timestamps[1, y, x] < timestamp - t + tau # P(Off | Off)
    timestamps[1*polarity, y, x] = timestamp
    

#%% Spatial Correlation computation
with AedatFile(file_name) as f:
    print("Spatial Correlation computation...")
    timestamps = np.zeros((2, size[1], size[0]))
    spat_corr = np.zeros((4, 2*l+1, 2*l+1))
    
    count = 0
    for e in f['events']:
        if count % freq_display == 0:
            print(100 * count / nb_event, "%")
            np.save("spatial_correlation", spat_corr)
            if count >= nb_event:
                break

        spatial_correlation(e.x, e.y, e.polarity, e.timestamp, spat_corr, timestamps)
        count += 1

for i in range(4):
    spat_corr[i] = spat_corr[i] / spat_corr[i].max()
    
#%% Plot
plot_spatial_correlation(spat_corr)

#%% Temporal Correlation computation
with AedatFile(file_name) as f:
    print("Temporal Correlation computation...")
    timestamps = np.zeros((2, size[1], size[0]))
    temp_corr = np.zeros((4, taus.size))
    
    count = 0
    for e in f['events']:
        if count % freq_display == 0:
            print(100 * count / nb_event, "%")
            np.save("temporal_correlation", temp_corr)
            if count >= nb_event:
                break
    
        temporal_correlation(e.x, e.y, e.polarity, e.timestamp, temp_corr, timestamps)
        count += 1
    
temp_corr = temp_corr / count

#%% Plot
plot_temporal_correlation(100*temp_corr)

#%% Plot
plot_temporal_cumsum(100*temp_corr)

#%% Loop frames
# with AedatFile(file_name) as f:    
#     for frame in f['frames']:
#         cv2.imshow('out', frame.image)
#         cv2.waitKey(100)
#     cv2.destroyAllWindows()