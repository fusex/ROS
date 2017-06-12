import serial
import array


ser = serial.Serial('/dev/ttyACM3',115200)

gx = 0.0
gy = 0.0
gz = 0.0

for i in range(0,2000):
   line = ser.readline().strip()[1:]

   if len(line) == 48:
      floatsArray = array.array('f',line) 
      gx += floatsArray[3] * 0.0174533
      gy += floatsArray[4] * 0.0174533
      gz += floatsArray[5] * 0.0174533

   
print('gx {0} \n gy {1} \n gz {2}'.format(gx,gy,gz))
