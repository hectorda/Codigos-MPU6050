# -*- coding: utf-8 -*-
import sys

def iniciar():
	from time import time, strftime
	import numpy as np
	import serial
	import sys
	import os
	import glob

	def serial_ports():
	    if sys.platform.startswith('win'):
	        ports = ['COM' + str(i + 1) for i in range(256)]

	    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
	        ports = glob.glob('/dev/tty[A-Za-z]*')
	    elif sys.platform.startswith('darwin'):
	        ports = glob.glob('/dev/tty.*')

	    else:
	        raise EnvironmentError('Unsupported platform')

	    result = []
	    for port in ports:
	        try:
	            s = serial.Serial(port)
	            s.close()
	            result.append(port)
	        except (OSError, serial.SerialException):
	            pass
	    return result
            
	puertos=serial_ports();
	print puertos
	if(len(puertos)==0):
		print("Dispositivo no conectado")
	s = serial.Serial(str(puertos[0]),250000)

	elapsed_time = 0;
	listx = []
	listy = []
	listtime = []

	tiempo = 40
	start_time = time()
	while (elapsed_time<tiempo):
	  try:
	    d = s.readline()
	    d = d.split()
	    if len(d)!=6:
	    	print "dato invalido"
	    print d
	    #X = float(d[0])
	    #Y = float(d[1])
	    if len(listtime)==0:
	    	start_time = time()                    
	    elapsed_time = time() - start_time
	    #listx.append(X)
	    #listy.append(Y)
	    listtime.append(elapsed_time)
	  except:
	      pass
	print "Tiempo",elapsed_time,"muestras",len(listtime),	

        
if __name__ == "__main__":
	iniciar()
