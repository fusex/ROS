import serial
import array


ser = serial.Serial('/dev/ttyACM3',115200)

mx = 0.0
my = 0.0
mz = 0.0
nbSample = 2000

for i in range(0,nbSample):
   line = ser.readline().strip()[1:]

   if len(line) == 48:
      floatsArray = array.array('f',line) 
      mx += floatsArray[6] * 1e-7    
      my += floatsArray[7] * 1e-7
      mz += floatsArray[8] * 1e-7

   
print('mx {0} \n my {1} \n mz {2}'.format(mx / nbSample, my / nbSample , mz / nbSample))
