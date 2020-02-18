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
file_name = "/home/thomas/Vidéos/DVS_Recordings/sport_20_min_video.aedat4"

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
    
def display_weights(nb_display):
    for i in range(nb_display):
        fig, axs = plt.subplots(1, 2)
        axs[0].imshow(np.load("/home/thomas/neuvisys-dv/results/weights/neuron_" + str(i) + ".npy")[0])
        axs[1].imshow(np.load("/home/thomas/neuvisys-dv/results/weights/neuron_" + str(i) + ".npy")[1])
        plt.show()
            
#%% Spatial Correlation
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

#%% Temporal Correlation
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

#%% polarity frequency variation
with AedatFile(file_name) as f:
    print("Polarity frequency variation computation...")
    polarities = [[], []]
    on, off = 0, 0
    time1, time2 = 0, 0
    disp_freq = 200 # ms
    bins = 1000 # µs
    cumsum = np.zeros(2, dtype=np.int32)
    np.set_printoptions(formatter={'float': lambda x: "{0:0.1f}".format(x)})
    
    test = []
    
    for e in f['events']:
        time1 = e.timestamp
        time2 = e.timestamp
        break

    for e in f['events']:
        if e.polarity:
            on += 1
        else:
            off += 1
            
        if e.timestamp - time1 > bins:
            time1 = e.timestamp
            perc = on / (on + off)
            polarities[0].append(perc)
            polarities[1].append(1 - perc)
            cumsum += [on, off]
            test.append(on + off)
            plt.plot(test)
            plt.pause(0.01)
            on, off = 0, 0
            
        if e.timestamp - time2 > 1000 * disp_freq:
            time2 = e.timestamp
            plt.title("ratio of On and Off events by bins of " + str(bins) + " µs.")
            plt.stackplot(range(len(polarities[0])), polarities[0], polarities[1])
            plt.xticks(np.linspace(0, len(polarities[0]), 11, dtype=np.int32), np.linspace(0, disp_freq, 11, dtype=np.int32), rotation=45, fontsize=12)
            plt.ylabel("on/off ratio")
            plt.xlabel("time (ms)")
            plt.axhline(y=0.5, color='r', linestyle='-')
            plt.show()
            
            print("ratio on/off:", 100 * np.array([np.mean(polarities[0]), 1 - np.mean(polarities[0])]), "/ cumulative ratio:",  100 * cumsum / cumsum.sum())
            polarities = [[], []]

#%% Loop frames
# with AedatFile(file_name) as f:    
#     for frame in f['frames']:
#         cv2.imshow('out', frame.image)
#         cv2.waitKey(100)
#     cv2.destroyAllWindows()