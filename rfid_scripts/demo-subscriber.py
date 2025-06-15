import tkinter as tk
from tkinter import Label
import  msu_rfid_reader_publisher_v2 as reader

from pubsub import pub

from datetime import datetime as dt


class DemoApp:
       
    def __init__(self,master):
        self.master = master
        master.title("DEMO APP - Subscribes to Topic published by RFIDReaderApp")

        self.label = Label(master, text="Label that will change when RFID read is brodcasted by RFID Reader App")
        self.label.pack(pady=100)

        #call method to subcribe to topic (named topic_rfid)
        self.subscribe_to_topic()

    def subscribe_to_topic(self):
        pub.subscribe(self.receive_message,"topic_rfid")

    def receive_message(self, data):
        #print("Message received...")
        self.label.config(text=f"Message Recceived at {dt.strftime(dt.now(),'%Y-%m-%d %H:%M:%S')},  Content = {data}")


    
if __name__ == "__main__":

    #This main gui owns the root window
    #This main gui can be the image capture GUI
    winMain = tk.Tk()
    winMain.geometry('1200x400')

    mainApp = DemoApp(winMain)

    #the second GUI is the RFID HUB
    width=1270
    height=360

    win2 = tk.Toplevel(winMain)
    win2.title("RFID Data Acquisition System - UNL+MSU")
    win2.geometry(f"{width}x{height}") #keep the width and height of the RFID App
    reader_app = reader.RFIDReaderApp(win2)

    winMain.mainloop()


