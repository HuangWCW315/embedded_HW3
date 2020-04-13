import matplotlib.pyplot as plt
import numpy as np
import serial
import time

serdev = '/dev/ttyACM0'
s = serial.Serial(serdev, 115200)

data_num_tem = s.readline()
data_num = int(data_num_tem)
print(data_num)
#create vector
t = np.arange(0,1,1/data_num)
x_p = np.arange(0,1,1/data_num)
y_p = np.arange(0,1,1/data_num)
z_p = np.arange(0,1,1/data_num)
tilt = np.arange(0,1,1/data_num)

#read data
for x in range(0, data_num):
    t_tem=s.readline() 
    t[x] = float(t_tem)

    x_tem=s.readline() 
    x_p[x]=float(x_tem)   

    y_tem=s.readline() 
    y_p[x]=float(y_tem)     

    z_tem=s.readline() 
    z_p[x]=float(z_tem)  

    tilt_tem = s.readline() 
    tilt[x] = int(tilt_tem)

#plot data
fig, ax = plt.subplots(2, 1)
ax[0].plot(t,x_p,color="blue", linewidth=2.5,linestyle="-",label="x")
ax[0].plot(t,y_p,color="red", linewidth=2.5,linestyle="-",label="y")
ax[0].plot(t,z_p,color="green", linewidth=2.5,linestyle="-",label="z")
ax[0].set_xlabel('Time')
ax[0].set_ylabel('ACC vector')
ax[0].legend(loc='lower left', frameon=True)

ax[1].stem(t, tilt) # plotting the spectrum
ax[1].set_xlabel('Time')
ax[1].set_ylabel('Tilt')

plt.show()
s.close()