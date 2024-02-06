#%% 
# imports
import pandas as pd
import matplotlib.pyplot as plt
import math
import os

#%%
# load data
filename = 'ROM_reduced_amp_01.txt'
data = pd.read_csv(filename)

# extract file name without file extension
description, _ = os.path.splitext(filename)

# remove empty column
data.drop(data.columns[-1], axis=1, inplace=True)

goal_positions = data[['m01_gp', 'm02_gp', 'm03_gp', 'm04_gp', 'm05_gp', 'm06_gp', 'm07_gp']]
present_positions = data[['m01_pp', 'm02_pp', 'm03_pp', 'm04_pp', 'm05_pp', 'm06_pp', 'm07_pp']]
start_buffer = 6000 #ms

#%%
# plotting goal positions single plot
font_size = 16

fig, ax = plt.subplots(1, figsize=(20, 10))
ax.plot(data.timestamp[data.timestamp > start_buffer] - start_buffer, goal_positions[data.timestamp > start_buffer])
ax.axhline(511, c='grey', ls='--')
ax.set_xlim(0,10000)
ax.tick_params(axis='both', labelsize=font_size)
ax.set_xlabel('Time [ms]', fontsize=font_size)
ax.set_ylim(0,1023)
ax.set_ylabel('Motor Position',fontsize=font_size)
plt.tight_layout()
plt.savefig(f'{description}_goal-position.png')

# Show the plot
plt.show()


#%%
# plotting goal positions travelling waves
font_size = 50

fig, axes = plt.subplots(goal_positions.shape[1], figsize=(50, 25), sharex=True)
for i in range(goal_positions.shape[1]):
    axes[i].plot(data.timestamp[data.timestamp > start_buffer] - start_buffer, goal_positions[data.timestamp > start_buffer].iloc[:,i])
    axes[i].axhline(511, c='grey', ls='--')
    axes[i].set_xlim(0,10000)
    axes[i].tick_params(axis='y', labelsize=font_size)
    #axes[i].set_ylim(0,1023)
    axes[i].set_ylabel('Motor '+str(i+1),fontsize=font_size)
axes[-1].set_xlabel('Time [ms]', fontsize=font_size)
axes[-1].tick_params(axis='both', labelsize=font_size)
axes[-1].set_xlim(0,10000)

plt.tight_layout()
plt.savefig(f'{description}_goal-position_separate.png')

# Show the plot
plt.show()

#%%
# plot range of motion for 1 motor
font_size = 24
id = 0

fig, ax = plt.subplots(1, figsize=(20,10))
ax.plot(data.timestamp[data.timestamp > start_buffer] - start_buffer, goal_positions[data.timestamp > start_buffer].iloc[:,id], label='goal position')
ax.plot(data.timestamp[data.timestamp > start_buffer] - start_buffer, present_positions[data.timestamp > start_buffer].iloc[:,id], label='actual position')
ax.axhline(511, c='grey', ls='--')
ax.set_xlim(0,10000)
ax.tick_params(axis='both', labelsize=font_size)
ax.set_xlabel('Time [ms]', fontsize=font_size)
ax.set_ylim(0,1023)
ax.set_ylabel('Motor Position',fontsize=font_size)
plt.legend(fontsize=font_size)
plt.tight_layout()
plt.savefig(f'{description}_ROM.png')

# Show the plot
plt.show()

print('ROM goal position')
print('Min:', min(goal_positions[data.timestamp > start_buffer].iloc[:,id]))
print('Max:', max(goal_positions[data.timestamp > start_buffer].iloc[:,id]))

print('\nROM present position')
print('Min:', min(present_positions[data.timestamp > start_buffer].iloc[:,id]))
print('Max:', max(present_positions[data.timestamp > start_buffer].iloc[:,id]))