import numpy as np
import matplotlib.pyplot as plt

target = {'laser': 5.991, 'radar': 7.815}

with open('../build/nis_laser.txt') as f:
    lines = f.read().splitlines()
nis_laser = []
for line in lines:
    nis_laser.append(float(line))
    
nis_laser = np.array(nis_laser)
n_laser = nis_laser.shape[0]

with open('../build/nis_radar.txt') as f:
    lines = f.read().splitlines()
nis_radar = []
for line in lines:
    nis_radar.append(float(line))
    
nis_radar = np.array(nis_radar)
n_radar = nis_radar.shape[0]

fig, axes = plt.subplots(2,1, figsize=(18,10))
axes[0].set_title('NIS laser')
axes[0].plot(nis_laser, label='NIS laser')
target_line = np.array([target['laser']]*n_laser)
axes[0].plot(np.arange(n_laser), target_line, 'r--', label='95% line')
axes[0].legend(loc='best')
axes[1].set_title('NIS radar')
axes[1].plot(nis_radar, label='NIS radar')
target_line = np.array([target['radar']]*n_radar)
axes[1].plot(np.arange(n_radar), target_line, 'r--', label='95% line')
axes[1].legend(loc='best')
#plt.show()
plt.savefig('NIS_dataset_1.png')
