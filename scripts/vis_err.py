import numpy as np
import matplotlib.pyplot as plt

est = {'px':[], 'py':[], 'vx':[], 'vy':[]}
gt = {'px':[], 'py':[], 'vx':[], 'vy':[]}
n = 0
with open('../figures/dataset2.txt') as f:
    for line in f:
        tokens = line.split(' ')
        est['px'].append(float(tokens[0]))
        est['py'].append(float(tokens[1]))
        est['vx'].append(float(tokens[2]))
        est['vy'].append(float(tokens[3]))
        gt['px'].append(float(tokens[4]))
        gt['py'].append(float(tokens[5]))
        gt['vx'].append(float(tokens[6]))
        gt['vy'].append(float(tokens[7]))   
        n += 1

est['px'] = np.array(est['px'])
est['py'] = np.array(est['py'])
est['vx'] = np.array(est['vx'])
est['vy'] = np.array(est['vy'])
gt['px'] = np.array(gt['px'])
gt['py'] = np.array(gt['py'])
gt['vx'] = np.array(gt['vx'])
gt['vy'] = np.array(gt['vy'])
        
fig, axes = plt.subplots(2,2, figsize=(18,12))
axes[0,0].set_title('px')
axes[0,0].plot(est['px'], label='px estimate')
axes[0,0].plot(gt['px'], 'r--', label='px ground truth')
axes[0,0].legend(loc='best')
axes[0,1].set_title('py')
axes[0,1].plot(est['py'], label='py estimate')
axes[0,1].plot(gt['py'], 'r--', label='py ground truth')
axes[0,1].legend(loc='best')
axes[1,0].set_title('vx')
axes[1,0].plot(est['vx'], label='vx estimate')
axes[1,0].plot(gt['vx'], 'r--', label='vx ground truth')
axes[1,0].legend(loc='best')
axes[1,1].set_title('vy')
axes[1,1].plot(est['vy'], label='vy estimate')
axes[1,1].plot(gt['vy'], 'r--', label='vy ground truth')
axes[1,1].legend(loc='best')
#plt.show()
plt.savefig('dataset_2.png')
