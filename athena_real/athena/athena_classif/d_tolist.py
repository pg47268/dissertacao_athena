

def dict_to_list(arg):
    for i in range(6):
        for j in arg[str(i)].keys():
            pos_list.append(arg[str(i)][j])
        
    return pos_list
    
if __name__ == '__main__':
    pos= {}
    pos_list = []
    for i in range(6):
        pos[str(i)] = {'TC': 0, 'CTr': 0, 'FTi': -90}
        
    pos_list = dict_to_list(pos)
    print(pos_list)