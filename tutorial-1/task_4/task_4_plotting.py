#%% 
# imports
import pandas as pd
import matplotlib.pyplot as plt

#%%
# load data
data = pd.read_csv(r'task_4_four_neurons/four_neurons_case_1_data.txt')

#remove empty column
data.drop(data.columns[-1], axis=1, inplace=True)

#%%
# plotting
fig, axes = plt.subplots(nrows=len(data.columns), ncols=1, figsize=(20, 10), sharex=True)

for i, column in enumerate(data.columns):
    axes[i].plot(data.index*0.01, data[column], label=column)
    axes[i].set_ylabel(column)
axes[-1].set_xlabel('Time [s]')
plt.tight_layout()

# Show the plot
plt.show()