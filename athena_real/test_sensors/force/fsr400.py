# Medir com sensor de força arduino com o python
import serial
import time
import csv
import numpy as np

def Porta():
# Encontra portas serie disponíveis.
    USB_port = []
    from serial.tools.list_ports import comports
    com_ports_list = list(comports())
    for port in com_ports_list:
        print(port)
        USB_port.append(port[0])
    return USB_port

if __name__ == "__main__":
    USB_port = Porta()
    if USB_port == []:
        print('Sem COM')
    else:
        usbp = USB_port[0]
        print('A usar a porta: ', usbp)

    Lista = []
    cabec = ["Analog reading", "Voltage reading in mV", "FSR resistance in ohms", "Conductance in microMhos", "Force in Newtons: "]
    Lista.append(cabec)
            
    ard = serial.Serial(port=usbp, baudrate= 9600, timeout=1)

    value = ard.readline() #inicializa a comunicação
    try:
        while True:
            value = ard.readline()
            strval = value.decode('ASCII').replace('\r\n', '').split(',')
            if strval[0] != '':
                Lista.append(strval)
                print(strval)
                time.sleep(2) #para 3 segundos
    except KeyboardInterrupt:    
        # Open file to write data
        with open('f_10k_1017.csv', 'w', newline='') as file:
            writer = csv.writer(file)

            # Write each inner list as a new row
            writer.writerows(Lista)

              

