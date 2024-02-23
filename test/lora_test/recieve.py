from serial import Serial, SerialBase
import time
import helloWorldTest_pb2
ser = Serial("/dev/ttyUSB0", 115200)
   

# Send character 'S' to start the program
# ser.read_all(bytearray('S','ascii'))
before = 0
# Read line   
while True:
    
    message = ser.read_until(expected=b'\x21')
    # message = ser.read()
    # print(message)
    output = message.decode()
    # hzes = 1/(time.time() - before)
    # if hzes > 20:
    #     hzes = 20.0
    # print(hzes)
    #message.ParseFromString()
    #print(message)
    # print("recieved")
    print(output)
    before = time.time()
    
    time.sleep(0.025)
    
# to run, cd into /test/lora_test
# then run this:
# python3 recieve.py &
# python3 send.py &
# currently only sending not recieving :(q