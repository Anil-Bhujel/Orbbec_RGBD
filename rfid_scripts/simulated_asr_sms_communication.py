import socket
import sys
import random
import time
from datetime import datetime as dt

'''

simulated_asr_trigger_mode_specify_rfid_rajtext

Loads a binary file with the reads and waits for multi id request trigger to respond

_specify version was designed to allow specify a specific IP address to respond from

This was done so that we could multiple clients with different ip addressese
from different ethernet interfaces in the SAME PC. At one point we used
one PC and three ethernet ports (one on board + two via usb dongle)
to simulated 3 different clients from three different ip
addresses connecting.

'''


def log(msg):
    
    with open("log-simulated_asr.txt","a") as fp:
        ts = dt.strftime(dt.now(),"%Y-%m-%d %H:%M:%S")
        fp.write(f"\n---@ {ts} --------\n")
        fp.write(msg)



data = None 
debug=0

if(debug):
    print("*"*100)
    print("This is debug mode. No SOCKETS will be created for transfer. \nThis is just to read binary file and simulate sending frames")
    print("*"*100)
    p = input("Enter any key + press enter to continue: ") 

def load_data(file):
    global data
    try:
        fp = open(file,"rb")
        data= fp.read()
        fp.close()
    except:
        log(f"Could not open/read {file}")
        data=None



def extract_reads(chunk):

    db_data = []

    numFrames = int(len(chunk)/29)

    ts= dt.now()  #all the reads are part of the same inquiry

    for i in range(0,numFrames):
        stInd = i*29
        frame = chunk[stInd:stInd+29]
        rfid = frame[5:21].decode("utf-8")  #decode byte array to string
        antenna = int(chr(frame[23])) #number to ascii to number again
        print("RFID {}   at antenna  {}". format(rfid,antenna))

        db_data.append((1,'1A',antenna,rfid,ts))

    #Only choose unique tuples
    '''
    db_uniq = list(set(db_data))

    for d in db_data:
        print(d)

    print("v"*20)

    for d in db_uniq:
        print(d)
    
    print("\n\n" + "-"*20)
    '''    



if(len(sys.argv) !=5 ):
    
    msg = "incorrect Usage. Usage:  script_name.py  host.ip.address   host.portNo   randomFrameCount  client.ip.address.to.use"
    msg = msg + "\n\nExample 1:\t\t script_name.py  192.168.0.100  12345  5  192.168.0.52"
    msg += "\t\t\t(tcp server at 192.168.0.100, port=12345, send upto 5 reads, and respond from 192.168.0.52)"
    msg = msg + "\n"
    msg += "\n\nExample 1:\t\t script_name.py  127.0.0.1  12345  3  127.0.0. 1"
    msg += "\t\t\t(tcp server at localhost, port=12345, send upto 5 reads, and respond from localhost)"
    log(msg)
    
    #sys.exit(msg)
    host="192.168.0.100"
    port=12345
    maxFrame=5
    clientIP="192.168.0.101"
else:

    host=sys.argv[1]
    port=int(sys.argv[2])
    maxFrame = int(sys.argv[3])
    clientIP = sys.argv[4]

#maxSleep = float(sys.argv[4])


if (not debug):
    ClientSocket = socket.socket()
    ClientSocket.bind((clientIP,0))  #THIS IS HOW AS A CLIENT YOU BIND YOURSEF TO SPECIFIC IP ADDRESS

    print('Waiting for SERVER {} @ port# {} to allow connection'.format(host,port))

    try:
        ClientSocket.connect((host, port))
        print("Connected (as ASR 650) to the Server ")
    except socket.error as e:
        print(str(e))
        log(f"Socket could not connect to host({host},{port})\n")
        sys.exit("Socker error could not connect to Server")

    #Waits for a response before starting loop
    Response = ClientSocket.recv(1024)
    #10/25/2024 - the sms hub first responds with the "02 FF AA WELCOME TO THE *****" message. 
    #We need to ignore this and then start listening

#read the binary file
#load_data("rfidTest2.bin")
load_data("rfidTest2_updated.bin")

if(not data):
    sys.exit("No data was found. Either binary file is missing or is empty")
    

numFrames = len(data)/29  #This is total number of frames available in the binary file

sentFrames = 0 #Track how many frames have we sent back so that we can loop back to the first frame

while True:

    try:
        #Wait for the trigger request, then respond
        print(">>>>>>Waiting for trigger request from the hub")
        Response = ClientSocket.recv(64) #blocking call
        #print(Response)

        #Check the 4th byte if its hex=0x21 i.e. dec=33, we have received a trigger, else its something else (set mux speed, set trigger mode etc. etc.)
        if(Response[3] != 33):

            #print("Not a trigger request")

            if(Response[3]==40):  #i.e. 0x28 (set trigger mode)
                print("<<<<<<<<<<<< RX: set operating mode request... sending an ack")
                #send something for now - the hub is expecting an ack
                ClientSocket.send(bytes.fromhex("02F001062110A203"))  #ack for something else but should do the job

            elif (Response[3]==195):  #i.e. c3 (set mux speed = 1)
                print("<<<<<<<<<<<< RX: set mux speed request... sending an ack")
                #send something for now - the hub is expecting an ack
                ClientSocket.send(bytes.fromhex("02F001062110A203"))  #ack for something else but should do the job 

            elif(Response[3]==1):  #i.e. 0x01 (connection request message)
                print("<<<<<<<<<<<< RX: connection request message to ASR650... sending an ack")
                #send something for now - the hub is expecting an ack
                ClientSocket.send(bytes.fromhex("02F001062110A203"))  #ack for something else but should do the job

            else:
                print(f"<<<<<<<<<<<< RX: [ERROR]...unknown 4th byte hex={hex(Response[3])}")
                print(f"[ERROR]...dont know how to handle, so exiting.")
                break

            #if did not break in the above checks, continue listening to the next request
            continue

        print("\n--------- MULTI_ID REQUEST RECEIVED ---------------Going to respond now -----")

        #Choose a random number N in (1,Max) to read N chunks from the binary file
        howMany = random.randint(1,maxFrame)
        print("***RandomNumFramesToSend={} ***".format(howMany))
        
        #While we still have frames left to send
        if(howMany+sentFrames <= numFrames):

            startIndex = sentFrames*29
            endIndex = startIndex+29*howMany
            sentFrames = sentFrames + howMany 
        else:
            startIndex = sentFrames*29
            endIndex = len(data)

            sentFrames = 0
        
        #Get the chunk of binary data at the right indices
        chunk = data[startIndex:endIndex]
        
        #Display the chunk about to be sent
        if(debug):
            print(chunk.hex())
            print("-"*20)

        if(not debug):
            extract_reads(chunk)  #Visually readable data

        if(not debug):

            try:
                numBytesToSend = len(chunk) #This is how many totral bytes we need to send
                bytesSent = 0 #this is how many bytes we have sent so far  
                #we are going to break up the total bytes into smaller chunks to simulate real tcp transfer
                #If 4 frames i.e 4*29 = 116 bytes need to be sent, we will send then in random sequences
                #such as 20 bytes, then 59 bytes, and so so.

                #Send the ack first
                ClientSocket.send(bytes.fromhex("02F001062110A203"))

                #Then send the rest of the bytes
                while(bytesSent < numBytesToSend):
                
                    random_size = random.randint(1,20) #send anywhere from 1 to 20 bytes
                    
                    sub_chunk = chunk[bytesSent: bytesSent + random_size]

                    if(debug):
                        print(sub_chunk)
                    
                    #Track bytes sent for this request so far
                    bytesSent = bytesSent + len(sub_chunk)
                    ClientSocket.send(sub_chunk)

                    if(debug):
                        print("#Bytes={}, So far sent = {}".format(len(sub_chunk),bytesSent))

                    #wait a few milliseconds before sending the next bytes in the sequence    
                    time.sleep(0.04)

                
            except Exception as e:
                log(f"(Area1) Exception occured. sg = {str(e)} . Exiting while loop\n")
                break
    except Exception as e:
                log(f"(Main) Exception occured. sg = {str(e)} .Exiting while loop\n")
                break    
    
    
print("Main while loop exited, so closing the socket")
ClientSocket.close()
