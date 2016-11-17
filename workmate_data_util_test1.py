from time import sleep
import serial.tools.list_ports

print "Detecting your computer's Operating System"
from sys import platform
if platform == "linux" or platform == "linux2":
    print "OS detected: Linux"
elif platform == "darwin":
    print "OS detected: MacOS"
elif platform == "win32":
    print "OS detected: Windows"
else:
    print "Failed to detect OS. Please try some other Mac,Linux or Windows system. Check website for compatible OS versions."
sleep(1)
    

ser = serial.tools.list_ports.comports()

detected = False

for i in range(1000):
    if detected:
        break
    ser = serial.tools.list_ports.comports()
    for port in ser:
        print 'port name '+port.name
        print 'port description '+port.description
        if port.description=='Arduino Leonardo':
            detected=True
            print "device found"
            break
        sleep(.001)
if not detected:
    print "cube not found. Did you press the connect button ?"

with open('log.txt','wb') as log_file:
    sleep_signal = 't'
    ser = serial.tools.list_ports.comports()
    for port in ser:
        print port.description
        if port.description=='Arduino Leonardo':
            sleep(.2)
            cube_port = port.name
            try:
                if platform == "linux" or platform == "linux2" or platform == "darwin":
                    cube_ser = serial.Serial('/dev/'+cube_port)
                elif platform == "win32":
                    cube_ser = serial.Serial(cube_port)
            except:
                print "something went wrong... please run this again"

            cube_ser.timeout = 15
            cube_ser.baudrate = 9600           
    #         cube_ser.write(u'a'.encode())
            ser_in = None
            sleep_counter = 0
            print "waiting for cube to start data transfer..."
            break_var = None
            while(True):
                sleep(1)
                sleep_counter+=1
                if sleep_counter>5:
                    print "cube didn't send transfer signal. Please try again :|"
                    break
                try:
                    ser_in = cube_ser.readline()
                    #print ser_in
                    if ser_in=='t\r\n':
                        if sleep_signal=='s':
                            #print 'sending sleep signal'
                            cube_ser.write(u's'.encode())
                        elif sleep_signal=='a':
                            #print 'sending awake signal'
                            cube_ser.write(u'a'.encode())
                        elif sleep_signal=='t':
                            print 'getting data'
                            cube_ser.write(u't'.encode())
                            transfer_status="started"
                            print "starting data transfer..."
                            MPU_data = 0
                            while not(transfer_status=="transfer complete\r\n" or transfer_status=="couldn't open file\r\n" ):
                                ser_in = cube_ser.readline()
                                #print ser_in
                                if MPU_data==1:    
                                    log_file.write(ser_in)
                                elif ser_in=="logfile opened\r\n":
                                    MPU_data = 1
                                transfer_status = ser_in
                            if transfer_status=="transfer complete\r\n": 
                                print "Awesome! data fetched. Go ahead, upload the file and see how you performed all this while :)"
                            elif transfer_status=="couldn't open file\r\n":
                                print "data file on the device couldn't be opened."
                            break_var=1
                            break
                except Exception,e:
                    pass
                    #print e
                    #print "seomthing went wrong. please give it another try"
                if break_var:
                    break

