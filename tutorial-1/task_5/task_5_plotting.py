#%% 
# imports
import pandas as pd
import matplotlib.pyplot as plt
import math
import os

#%%
# load data
filename = 'task_5_data_eight_coupling_no-perturbation.txt'
data = pd.read_csv(filename)

# extract file name without file extension
description, _ = os.path.splitext(filename)

# remove empty column
data.drop(data.columns[-1], axis=1, inplace=True)

#%%
# plotting all neurons
n_rows = math.ceil(len(data.columns)/2)
fig, axes = plt.subplots(nrows=n_rows, ncols=2, figsize=(20, 10), sharex=True)

#axes = axes.flatten()

for i, column in enumerate(data.columns):
    if i < n_rows:
        axes[i][0].plot(data.index*0.01, data[column], label=column)
        axes[i][0].set_ylabel(column)
    else:
        axes[i-n_rows][1].plot(data.index*0.01, data[column], label=column)
        axes[i-n_rows][1].set_ylabel(column)
axes[n_rows-1][0].set_xlabel('Time [s]')
axes[n_rows-1][1].set_xlabel('Time [s]')
plt.tight_layout()
plt.savefig(f'{description}.png')

# Show the plot
plt.show()

#%%
# plotting subtraction of horizontally aligned neurons
n_rows = math.ceil(len(data.columns)/2)
fig, axes = plt.subplots(nrows=n_rows, figsize=(20, 10), sharex=True)

for i in range(n_rows):
    osc = data.iloc[:,i] - data.iloc[:,i+n_rows]
    axes[i]. plot(data.index, osc)
    axes[i].set_ylabel('Motor '+str(i+1))
axes[-1].set_xlabel('Time')
plt.tight_layout()
plt.show()