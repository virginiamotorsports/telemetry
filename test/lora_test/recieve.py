from serial import Serial, SerialBase
import time
ser = Serial("/dev/ttyUSB1", 115200)
   
# Send character 'S' to start the program
# ser.read_all(bytearray('S','ascii'))
before = 0
# Read line   
while True:
    
    data = ser.read_until(expected=b'\xFF')
    hzes = 1/(time.time() - before)
    if hzes > 20:
        hzes = 20.0
    print(hzes)
    before = time.time()
    
    # time.sleep(0.025)