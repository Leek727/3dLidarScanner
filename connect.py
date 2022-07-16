import serial
import time

with open('data.csv', 'w') as f:
    f.write("")

def write_port(data):
    port.write(data.encode())

print("Looking for a device...")
while True:
    try:
        port = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=0)
        time.sleep(2)
        write_port('connected')

        print("Connected!")
        break

    except Exception as e:
        print(e)
        time.sleep(.5)

while True:
    #time.sleep(.2)
    buffer = ""
    data = port.read(10000)
    if len(data) > 0:
        data = data.decode().strip()#.split(",")
        if "Done" in data:
            break
        
        #print(data,end="")
        with open("data.csv", "a") as f:
            f.write(data.replace(":","\n"))           