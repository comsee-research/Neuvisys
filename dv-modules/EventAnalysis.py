from dv import AedatFile

import numpy as np
import matplotlib.pyplot as plt
from scipy.sparse import vstack, dok_matrix, csr_matrix
from multiprocessing import Pool

tau = 1000

def mat_corr(mat1: csr_matrix, mat2: csr_matrix, ind: int):
    return correlation(mat1[:, ind], mat2[:, ind])

def correlation(mat1: csr_matrix, mat2: csr_matrix):
    
    corr = np.zeros(tau)
    for k in range(tau):
        corr[k] = (vstack([mat1[k:], csr_matrix((k, 1))]).transpose() * mat2)[0, 0]
    return corr

if __name__ == "__main__":
    with AedatFile("/home/thomas/Vidéos/DVS_Recordings/sport_30_min_video.aedat4") as f:
        duration = 100000
        size = (346, 260)
        n = size[0]*size[1]

        vecon = dok_matrix((duration, n), dtype=np.int8)
        vecoff = dok_matrix((duration, n), dtype=np.int8)
        # vecon = np.zeros(duration)
        # vecoff = np.zeros(duration)
        first = 0
        lock = True
        
        for e in f['events']:
            first = e.timestamp
            break
        
        print("Spatial Correlation computation...")
        mat = np.zeros((size[0], size[1]))
        proba = np.zeros((size[0], size[1]))
        for e in f['events']:
            time = e.timestamp - first
            if time > duration:
                break
        
            mat[e.x, e.y] = e.timestamp
            proba[e.x-1:e.x+2, e.y-1:e.y+2] += 1 * (mat[e.x-1:e.x+2, e.y-1:e.y+2] > e.timestamp - tau)

#%%
        print("Loading events...")
        for e in f['events']:
            time = e.timestamp - first
            if time > duration:
                break

            if e.polarity:
                vecon[time, e.x*size[1] + e.y] = 1
            else:
                vecoff[time, e.x*size[1] + e.y] = 1

#%%
        print("Spatial Correlation computation...")
        mat = np.zeros((size[0], size[1]))
        proba = np.zeros((size[0], size[1]))
        for e in f['events']:
            time = e.timestamp - first
            if time > duration:
                break
        
            mat[e.x, e.y] = e.timestamp
            proba[e.x-1:e.x+2, e.y-1:e.y+2] += 1 * (mat[e.x-1:e.x+2, e.y-1:e.y+2] > e.timestamp - tau)

#%%
        print("Temporal Correlation computation...")
        nb_proc = 8
        
        args_oo = [(vecon, vecon, i) for i in range(8)]        
        
        with Pool(nb_proc) as p:
            corr_oo = np.sum(p.starmap(mat_corr, args_oo), axis=0)

#%%
        print("Display")
        plt.plot(corr_oo)
        plt.show()
