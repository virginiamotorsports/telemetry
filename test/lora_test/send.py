from serial import Serial, SerialBase
import time, random
import numpy as np
ser = Serial("/dev/ttyUSB0", 115200)
   
# Send character 'S' to start the program
# ser.write(bytearray('S','ascii'))
random_bytes = lambda n:bytearray(map(random.getrandbits,(8,)*n))

# Read line   
while True:
    # n = 199
    
    bytesarr = np.array(random_bytes(199), dtype=np.uint8)
    # print(bytesarr)
    bytesarr[bytesarr == 255] = 0
    bytesarr = np.append(bytesarr, 255)
    # print(bytearray(bytesarr.tolist()))
    # print(len(bytesarr))
    ser.write(bytearray(bytesarr.tolist()))
    time.sleep(.0975)
    print("sent")