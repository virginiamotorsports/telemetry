from serial import Serial, SerialBase
import time, random
import numpy as np
import helloWorldTest_pb2
ser = Serial("/dev/ttyUSB1", 115200)
   
# Send character 'S' to start the program
# ser.write(bytearray('S','ascii'))
#random_bytes = lambda n:bytearray(map(random.getrandbits,(8,)*n))
# message = helloWorldTest_pb2.Test1()
# message.text = "Hello World!"
# message.text.encode
# Read line   
while True:
    # n = 199
    # print("Hello World!")
    #bytesarr = np.array(random_bytes(199), dtype=np.uint8)
    # print(bytesarr)
    #bytesarr[bytesarr == 255] = 0
    #bytesarr = np.append(bytesarr, 255)
    # print(bytearray(bytesarr.tolist()))
    # print(len(bytesarr))
    #ser.write(bytearray(bytesarr.tolist()))
    
    ser.write(bytearray("Hello World!", 'ascii'))
    
    time.sleep(.0975)