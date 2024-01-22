import os
import json

def init():
    path = '/home/isabel/Desktop/athena_ard_new/'
    os.chdir(path)
    
    initial_cond = {'gait': 1, 'dir': 1, 'time': 2.06}
    
    fn = path + 'parameters.json'
    with open(fn, 'w') as outfile:
        json.dump(initial_cond, outfile)
        
if __name__ == '__main__':
    init()