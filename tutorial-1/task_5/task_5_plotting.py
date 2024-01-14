#%% 
# imports
import pandas as pd
import matplotlib.pyplot as plt
import math
import os

#%%
# load data
filename = 'task_5_data_eight_coupling.txt'
data = pd.read_csv(filename)

# extract file name without file extension
description, _ = os.path.splitext(filename)

# remove empty column
data.drop(data.columns[-1], axis=1, inplace=True)

#%%
# plotting
n_rows = math.ceil(len(data.columns)/2)
fig, axes = plt.subplots(nrows=n_rows, ncols=2, figsize=(20, 10), sharex=True)

axes = axes.flatten()

for i, column in enumerate(data.columns):
    axes[i].plot(data.index*0.01, data[column], label=column)
    axes[i].set_ylabel(column)
axes[-2].set_xlabel('Time [s]')
axes[-1].set_xlabel('Time [s]')
plt.tight_layout()
plt.savefig(f'{description}.png')

# Show the plot
plt.show()