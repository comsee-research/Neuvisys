from dv import AedatFile
import cv2
import numpy as np
import matplotlib.pyplot as plt

with AedatFile("/home/thomas/Vidéos/DVS_Recordings/sport_30_min_video.aedat4") as f:
    vec_eve_on = np.zeros(1000000)
    vec_eve_off = np.zeros(1000000)
    lock = True

    for e in f['events']:
        if lock:
            lock = False
            first = e.timestamp

        time = e.timestamp - first
        if time > 1000000:
            break

        if e.polarity:
            vec_eve_on[time] = 1
        else:
            vec_eve_off[time] = 1

    on_autocore = np.correlate(vec_eve_on, vec_eve_on, "same")
    # off_autocore = np.correlate(vec_eve_off, vec_eve_off, "same")

    plt.plot(on_autocore)