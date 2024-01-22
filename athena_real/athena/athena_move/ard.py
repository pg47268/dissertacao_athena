import time
import serial
import numpy as np
from struct import *
from pySerialTransfer import pySerialTransfer as txfer
from serial.tools.list_ports import comports

class Arduino:
    
    def __init__(self):
    #----------arduino comunication----------
        self.ard = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
        self.link = txfer.SerialTransfer('/dev/ttyACM0')
        self.link.open()
        time.sleep(2)

    #----------encoders----------
        self.encoders = np.zeros(18)
    #----------sensors-----------
        self.force = np.zeros(6)
        self.dist = []
        self.vel_x = []
        self.accel_x = []
        self.imu = np.zeros(2)
        self.h = []
        
    def port(self):
        usb_port = []
        com_ports_list = list(comports())
        for port in com_ports_list:
            if port[1].find("Arduino") != -1: #for arduino uno-> find("ttyACM")
                usb_port.append(port[0])
        return usb_port
        
        
    def write1(self, data):
        #convert to list of floats instead of list of ints
        data_f = [float(x) for x in data]
        send_size = 0

        list_size = self.link.tx_obj(data_f)
        send_size += list_size  
        self.link.send(send_size)

        while not self.link.available():
            if self.link.status < 0:
                if self.link.status == txfer.CRC_ERROR:
                    print('ERROR: CRC_ERROR')
                elif self.link.status == txfer.PAYLOAD_ERROR:
                    print('ERROR: PAYLOAD_ERROR')
                elif self.link.status == txfer.STOP_BYTE_ERROR:
                    print('ERROR: STOP_BYTE_ERROR')
                else:
                    print('ERROR: {}'.format(self.link.status))
               

        # Receive and print the force sensor data and pos array from Arduino
        received_data = self.link.rx_obj(obj_type=list,
                                         obj_byte_size=112,
                                         list_format='f')

        
        #print('SENT: {}'.format(data))
        print(' ')
        print('Position Data: {}'.format(received_data))

        self.encoders = received_data[:18]
        self.force = received_data[18:24]
        print(self.force)
        self.vel_x = received_data[24]
        self.accel_x = received_data[25]
        self.h = received_data[26]

        
        
    def readInfo(self, data):
        read = False
        while read is False:
            line = self.ard.readline().decode('utf-8').rstrip()
            if line == data:
                read = True
                self.ard.reset_input_buffer()    
        

    def contact(self, id):
        '''if id == 1 or id == 2:
            if self.force[id] > 0:
                #print("Contact", self.force[id-1])
                return True
            else:
                #print(" No Contact", self.force[id-1])
                return False'''
        if self.force[id] >= 0:
            #print("Contact", self.force[id-1])
            return True
        else:
            #print(" No Contact", self.force[id-1])
            return False
            
    def all_contact(self):
        self.count_contact = []
        for id in range(6):
            if self.force[id] >= 0:
                self.count_contact.append(id)
        return self.count_contact
    
    def foot_contact(self, id):
        if id == 1 or id == 2:
            if self.force[id] >= 150:
                return True
            else:
                return False

    
'''if __name__ == '__main__':
    
    ard = Arduino()
    
    angle = 0
    pos = np.zeros(18)
    for i in range(9):
        angle += 5
        for j in range(18):
            pos[j] = angle
        print(angle)
        ard.write(1.0, pos)
        time.sleep(0.1)'''
        
