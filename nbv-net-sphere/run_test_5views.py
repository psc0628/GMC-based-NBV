import sys
import os
from torch.nn.functional import dropout
from nbvnet import NBV_Net
import torch
import time
import numpy as np
import math

viewspace = []
with open("./pack.3.14.txt",'r') as fvs:
    num = 0
    coord = []
    for line in fvs:
        coord.append(float(line.strip('\n')))
        num = num +1
        if num == 3:
            viewspace.append(coord)
            num = 0
            coord = []
print(viewspace)
norm = math.sqrt( viewspace[0][0] * viewspace[0][0] + viewspace[0][1] * viewspace[0][1] + viewspace[0][2]*viewspace[0][2] )
#print(norm)

name_of_model = []
with open('./all_name.TXT', 'r') as f:
    for line in f:
        name_of_model.append(line.strip('\n'))
model = ''

first_view_ids = []
for i in range(0,5):
    first_view_ids.append(i)
view_id = -1

max_iteration = 20
iteration = -1

def get_single_view_point(path):
    net = NBV_Net(dropout_prob=0)
    checkpoint = torch.load('nbv_14_HB.pth.tar',map_location = torch.device('cpu'))
    net.load_state_dict(checkpoint['net'])
    grid = np.genfromtxt(path).reshape(1, 1, 32, 32, 32)
    grid = torch.tensor(grid, dtype=torch.float32)
    startTime = time.time()
    ids = net(grid)
    endTime = time.time()
    print('run time is ' + str(endTime-startTime))
    np.savetxt('./run_time/'+model+'_v'+str(view_id)+'_'+str(iteration)+'.txt',np.asarray([endTime-startTime]))
    return ids


for model in name_of_model:
    print('testing '+ model)
    for view_id in first_view_ids:
        print('max_iteration is '+str(max_iteration))
        iteration = 0
        while iteration<max_iteration:
            print('./data/'+model+'_v'+str(view_id)+'_'+str(iteration)+'.txt')
            while os.path.isfile('./data/'+model+'_v'+str(view_id)+'_'+str(iteration)+'.txt')==False:
                pass
            time.sleep(1)
            ids = get_single_view_point('./data/'+model+'_v'+str(view_id)+'_'+str(iteration)+'.txt')
            ids = ids.argmax(dim=1)
            print('next view is ' + str(ids))
            scale = 0.0
            with open("./viewspace/"+model+".txt",'r') as fvs:
                for line in fvs:
                    scale = float(line.strip('\n'))
            outview = [viewspace[ids][0]/norm*scale,viewspace[ids][1]/norm*scale,viewspace[ids][2]/norm*scale]
            np.savetxt('./log/'+model+'_v'+str(view_id)+'_'+str(iteration)+'.txt',outview,fmt='%f')
            f = open('./log/ready.txt','a')
            f.close()
            iteration += 1
        print('testing '+ model + '_v'+ str(view_id) + ' over.')
