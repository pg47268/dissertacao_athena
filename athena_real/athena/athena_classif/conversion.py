def real_to_sim(angles):
    for i in range(6):
        angles[str(i)]['TC']-= 35
        # CTr
        '''if i in [0, 3]:
            angles[str(i)]['CTr']-= 75
        elif i in [1, 4]:
            angles[str(i)]['CTr']-= 85
        elif i == 5:
            angles[str(i)]['CTr']-= 90
        else:
            angles[str(i)]['CTr']-= 80'''
        if i in [1, 5]:
            angles[str(i)]['CTr']-= 90
        elif i in [0, 2, 3]:
            angles[str(i)]['CTr']-= 80
        else:
            angles[str(i)]['CTr']-= 85
        angles[str(i)]['CTr']*= -1
        
        # FTi
        '''if i == 1:
            angles[str(i)]['FTi']-= 15
        elif i == 2:
            angles[str(i)]['FTi']-= 0
        elif i == 3:
            angles[str(i)]['FTi']-= 10
        elif i == 5:
            angles[str(i)]['FTi']-= 25
        else:
            angles[str(i)]['FTi']-= 5'''
        if i == 1:
            angles[str(i)]['FTi']-= 15
        elif i == 3:
            angles[str(i)]['FTi']-= 10    
        elif i == 5:
            angles[str(i)]['FTi']-= 25
        else:
            angles[str(i)]['FTi']-= 5
        angles[str(i)]['FTi']*=-1   
            
    return angles


def sim_to_real(angles):
    for i in range(6):
        angles[str(i)]['TC']+= 35
        angles[str(i)]['CTr']*= -1
        # CTr        
        '''if i in [0, 3]:
            angles[str(i)]['CTr']+= 75
        elif i in [1, 4]:
            angles[str(i)]['CTr']+= 85
        elif i == 5:
            angles[str(i)]['CTr']+= 90
        else:
            angles[str(i)]['CTr']+= 80'''
        if i in [1, 5]:
            angles[str(i)]['CTr']+= 90
        elif i in [0, 2, 3]:
            angles[str(i)]['CTr']+= 80
        else:
            angles[str(i)]['CTr']+= 85
        # FTi      
        '''if i == 1:
            angles[str(i)]['FTi']-= 15
        elif i == 2:
            angles[str(i)]['FTi']-= 0
        elif i == 3:
            angles[str(i)]['FTi']-= 10
        elif i == 5:
            angles[str(i)]['FTi']-= 25
        else:
            angles[str(i)]['FTi']-= 5
        angles[str(i)]['FTi']*=-1'''

        if i == 1:
            angles[str(i)]['FTi']-= 15
        elif i == 3:
            angles[str(i)]['FTi']-= 10    
        elif i == 5:
            angles[str(i)]['FTi']-= 25
        else:
            angles[str(i)]['FTi']-= 5
        angles[str(i)]['FTi']*=-1
        
    return angles



