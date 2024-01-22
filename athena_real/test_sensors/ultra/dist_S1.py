# Medir distância com sensor de ultrassons arduino com o python
import serial
import time
import numpy as np
import matplotlib.pyplot as plt

def Porta():
# Encontra portas serie disponíveis.
    USB_port = []
    from serial.tools.list_ports import comports
    com_ports_list = list(comports())
    for port in com_ports_list:
        USB_port.append(port[0])
    return USB_port

if __name__ == "__main__":
    USB_port = Porta()
    if USB_port == []:
        print('Sem COM')
    else:
        usbp = USB_port[0]
        print('A usar a porta: ', usbp)
            
    ard = serial.Serial(port=usbp, baudrate= 115200, timeout=1)

    value = ard.readline() #inicializa a comunicação
    plt.xlabel('tempo [s]')
    plt.ylabel('Distância [cm]')
    plt.title('Sensor distância ultrassons arduino')
    plt.grid()
    #plt.ion()
    dist = []
    temp = []
    time0 = time.time()
    while True:
        value = ard.readline()
        timec = time.time() - time0
        strval = value.decode('ASCII')
        try:
            dist.append(float(strval))
            temp.append(timec)
            plt.plot(temp, dist, 'b-')
            plt.pause(0.025)
        except:
            continue

    plt.show()
              

