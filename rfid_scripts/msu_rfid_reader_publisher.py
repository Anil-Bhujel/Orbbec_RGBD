from ast import Pass
from re import I
from tkinter import Label, Button,Frame,Tk,Text,END,Canvas,Menu,filedialog,IntVar,Entry,Checkbutton,StringVar,Radiobutton
from tkinter.messagebox import showinfo,showerror
from tkinter.ttk import Style,Combobox 
import time
import tkinter 
from tkinter.ttk import Treeview,Scrollbar,Notebook
import json 

import socket
import os
import sys 
from _thread import *
from threading import current_thread,Thread, Event

from datetime import datetime as dt
import queue
import traceback 
from collections import OrderedDict
import random

import logging
from turtle import bgcolor 

from tktooltip import ToolTip
from tkinter import messagebox
from PIL import ImageTk, Image

from pubsub import pub



VERSION = 1  #Start at V1 for MSU (based on hub_v18)

divider = "\n"+"-"*40+"\n"


VERSION_MESSAGE = "Version:"+ str(VERSION)
VERSION_MESSAGE += "\nV1 for MSU, based off V18 for USMARC - removed image capture related items"
VERSION_MESSAGE += "\nAdded ASR650 commands to set mode to trigger + fast mux speed"
VERSION_MESSAGE += "\nLoging Tx/Rx traffic via json setting + Via checkbox added 'LogTxRx'"


logging.basicConfig(level=logging.DEBUG,filename="log-communications.log",
        format='%(asctime)s:%(levelname)s:%(message)s')

logger = logging.getLogger(__name__)
logging.getLogger('mysql.connector').setLevel(logging.ERROR) #only log error or higher for mysql



MAIN_GUI_WIDTH=1270
MAIN_GUI_HEIGHT=360 #450 works as standalone but when called inside TopLevel things got cut off

STATUS_BAR_HEIGHT=20
GUI_WIDTH=MAIN_GUI_WIDTH
GUI_HEIGHT = MAIN_GUI_HEIGHT-STATUS_BAR_HEIGHT

TITLE_SIZE=16
TREEVIEW_CONTENT_SIZE=12
TREEVIEW_HEADING_SIZE=12


import db_handler_sqlite3 as mdbh


#Part of CRC generating routine
def reflect_data(x, width):
    # See: https://stackoverflow.com/a/20918545
    if width == 8:
        x = ((x & 0x55) << 1) | ((x & 0xAA) >> 1)
        x = ((x & 0x33) << 2) | ((x & 0xCC) >> 2)
        x = ((x & 0x0F) << 4) | ((x & 0xF0) >> 4)
    elif width == 16:
        x = ((x & 0x5555) << 1) | ((x & 0xAAAA) >> 1)
        x = ((x & 0x3333) << 2) | ((x & 0xCCCC) >> 2)
        x = ((x & 0x0F0F) << 4) | ((x & 0xF0F0) >> 4)
        x = ((x & 0x00FF) << 8) | ((x & 0xFF00) >> 8)
    elif width == 32:
        x = ((x & 0x55555555) << 1) | ((x & 0xAAAAAAAA) >> 1)
        x = ((x & 0x33333333) << 2) | ((x & 0xCCCCCCCC) >> 2)
        x = ((x & 0x0F0F0F0F) << 4) | ((x & 0xF0F0F0F0) >> 4)
        x = ((x & 0x00FF00FF) << 8) | ((x & 0xFF00FF00) >> 8)
        x = ((x & 0x0000FFFF) << 16) | ((x & 0xFFFF0000) >> 16)
    else:
        raise ValueError('Unsupported width')
    return x

def crc_poly(data, n, poly, crc=0, ref_in=False, ref_out=False, xor_out=0):
    g = 1 << n | poly  # Generator polynomial

    # Loop over the data
    for d in data:
        # Reverse the input byte if the flag is true
        if ref_in:
            d = reflect_data(d, 8)

        # XOR the top byte in the CRC with the input byte
        crc ^= d << (n - 8)

        # Loop over all the bits in the byte
        for _ in range(8):
            # Start by shifting the CRC, so we can check for the top bit
            crc <<= 1

            # XOR the CRC if the top bit is 1
            if crc & (1 << n):
                crc ^= g

    # Reverse the output if the flag is true
    if ref_out:
        crc = reflect_data(crc, n)

    # Return the CRC value
    return crc ^ xor_out

#end CRC generating routine



'''
Main Class handling RFID connect/read/write

'''


class RFIDReaderApp(Frame):

    def __init__(self, master):
        
        #Make two frame - main_master (which will have the notebook) and status_frame
        self.frameHeader = Frame(master,bg="red",height=100)
        self.frameMain = Frame(master,bg="red",height=GUI_HEIGHT)
        frameStatus = Frame(master,height=STATUS_BAR_HEIGHT,bg="coral1")

        self.frameHeader.pack(side="top")
        self.frameMain.pack(side="top")
        frameStatus.pack(side="bottom",fill="x")
        
        self.main_master = master

        super().__init__(self.main_master)
        self.initParameters()

        # create a notebook
        #self.notebook = Notebook(self.main_master)
        self.notebook = Notebook(self.frameMain)
        self.notebook.pack(pady=0, expand=True)

        # create frames
        #TAB_HEIGHT = 300 for 450 MAIN HEIGHT
        TAB_HEIGHT = 200
        self.tab1 = Frame(self.notebook,width=GUI_WIDTH,bg="LightSkyBlue1",height=TAB_HEIGHT)
        self.tabMessage = Frame(self.notebook, width=GUI_WIDTH,height=TAB_HEIGHT)
        self.tab2 = Frame(self.notebook,width=GUI_WIDTH,height=TAB_HEIGHT)
        self.tab3 = Frame(self.notebook,width=GUI_WIDTH,height=TAB_HEIGHT)
        self.tabASRInfo = Frame(self.notebook,width=GUI_WIDTH,height=TAB_HEIGHT)
       
        self.tabVersionInfo=Frame(self.notebook,width=GUI_WIDTH,height=TAB_HEIGHT)
        
        

        self.tab1.pack(fill='both', expand=True, anchor="nw")
        self.tabMessage.pack(fill="both",expand=True,anchor="nw")
        self.tab2.pack(fill='both', expand=True,anchor="nw")
        self.tab3.pack(fill='both', expand=True,anchor="nw")
        self.tabASRInfo.pack(fill='both', expand=True,anchor="nw")
      
        self.tabVersionInfo.pack(fill='both', expand=True,anchor="nw")



        self.tab1.pack_propagate(False)
        self.tabMessage.propagate(False)
        self.tab2.pack_propagate(False)
        self.tab2.pack_propagate(False)
        self.tabASRInfo.pack_propagate(False)
      
        
        self.tabVersionInfo.pack_propagate(False)


         
        # add frames to notebook
        self.notebook.add(self.tab1, text='Main')
        self.notebook.add(self.tabMessage,text="MessageWindow")
        self.notebook.add(self.tab2, text='Threads')
        self.notebook.add(self.tab3, text='Read Info')
        self.notebook.add(self.tabASRInfo, text='ASR Info')
   
        self.notebook.add(self.tabVersionInfo, text='Version Info')


        #Event listener on notebook
        self.notebook.bind('<<NotebookTabChanged>>',self.notebookTabChanged)

        #self.master = self.main_master 
        #menubar = Menu(self.master, tearoff=0)
        self.master = self.tab1
        menubar = Menu(self.main_master, tearoff=0)


        settingsmenu = Menu(menubar,tearoff=0)
        settingsmenu.add_command(label="Load Settings.JSON", command=self.prompt_json)
        #settingsmenu.add_command(label="Reload", command=self.load_json)
        
        dbmenu = Menu(menubar)
        dbmenu.add_command(label="DELETE ALL feeder_data_new", command=self.emptyFeederData)
        dbmenu.add_command(label="DELETE today's feeder_data_new", command=self.emptyFeederDataToday)
          

        #menubar.add_cascade(label='Settings', menu=settingsmenu)
        menubar.add_cascade(label='Database', menu=dbmenu)
               
        self.main_master.config(menu=menubar)

        #Top bar with persistent informaion
        self.frameDebug = Frame(self.frameHeader,bg="#800000",bd=2,pady=5,padx=5)
        self.frameDebug.pack(fill="x")

        l0=Label(self.frameDebug,text='Total Threads:')
        l0.pack(side="left",padx=(20,0))
        self.LabelThreads = Label(self.frameDebug, text='0')
        self.LabelThreads.pack(side="left")

        lTh2=Label(self.frameDebug,text='Active Threads:')
        lTh2.pack(side="left",padx=(20,0))
        self.LabelThreadsActive = Label(self.frameDebug, text='0')
        self.LabelThreadsActive.pack(side="left")


        l1=Label(self.frameDebug,text=' || Deque Size:', padx=20)
        l1.pack(side="left")
        self.LabelDequeSize = Label(self.frameDebug, text='0')
        self.LabelDequeSize.pack(side="left")

        l2=Label(self.frameDebug,text='|| Trigger Timeout:', padx=20)
        l2.pack(side="left")
        self.LabelTimeout = Label(self.frameDebug, textvariable=str(self.timeoutTrigger_ms))
        self.LabelTimeout.pack(side="left")

        l3=Label(self.frameDebug,text='|| TriggerDelay:', padx=20)
        l3.pack(side="left")
        self.LabelDelay = Label(self.frameDebug, textvariable=str(self.triggerInterval_ms))
        self.LabelDelay.pack(side="left")

        self.btnPause = Button(self.frameDebug, text="Pause", command=self.pauseCommunications)
        self.btnPause.pack(side="right")

        self.btnReloadParams = Button(self.frameDebug, text="Reload Parameters", command=self.reloadSettingsParameters)
        self.btnReloadParams.pack(side="right",padx=(10,0))

  
        self.master.update()

        #Frame#1 - Right below the tabs
        self.frame1 = Frame(self.master,bg="#002000",bd=2,pady=0,padx=0) #padx was 5 before
        #self.frame1.pack(fill="x")
        self.frame1.pack()

        #RFID System Label
        labelRFID = Label(self.frame1,text="RFID\nSystem",font=(0,30))
        labelRFID.pack(side="left")

        img1 = ImageTk.PhotoImage(Image.open("logo-msu-64.png"))
        self.img1 = img1

        labelUNL = Label(self.frame1,image=self.img1)
        labelUNL.pack(side="left", padx=10 )
        
        img2 = ImageTk.PhotoImage(Image.open("logo-nebraska-no-fill-64.jpg"))
        self.img2 = img2

        labelUNL = Label(self.frame1,image=self.img2)
        labelUNL.pack(side="left", padx=10 )


        imgAMILAB = ImageTk.PhotoImage(Image.open("sms-amilab-logo-128-2.jpg"))
        self.imgAMILAB = imgAMILAB

        labelAMILAB = Label(self.frame1,image=self.imgAMILAB)
        labelAMILAB.pack(side="left", padx=(10,40) )



        frameL = Frame(self.frame1,padx=10)
        frameL.pack(side="left")

        l1=Label(frameL,text="SERVER STATUS",pady=0,bg='#338080',fg='#FFFFFF',font=(0,TITLE_SIZE))
        l1.pack(fill="x")

        self.treeServer = Treeview(frameL,height=1,selectmode="none") #selectmode = none makes it unselectable and does not change the highlights
        self.treeServer['columns']=('server','port','status_server')
        self.treeServer.column('#0',width=0,stretch="no")
        self.treeServer.column('server',width=125,anchor="center")
        self.treeServer.column('port',width=80,anchor="center")
        self.treeServer.column('status_server',width=120,anchor="center")
        self.treeServer.heading("server",text="Server IP",anchor="center")
        self.treeServer.heading("port",text="PORT",anchor="center")
        self.treeServer.heading("status_server",text="Status",anchor="center")
       
        self.treeServer.tag_configure("not-running", background="#dc3545",foreground="white")
        self.treeServer.tag_configure("running", background="pale green",foreground="black")

        self.treeServer.pack()

        
        ######## Database Status ####################
        self.frameDB = Frame(self.frame1,padx=1) #padx=5 before
        self.frameDB.pack(side="left")
       
        l1=Label(self.frameDB,text="DATABASE",pady=0,bg='#3300FF',fg='#FFFFFF',font=(0,TITLE_SIZE))
        l1.pack(fill="x")

        self.treeDB = Treeview(self.frameDB,height=1,selectmode="none")
     
        self.treeDB['columns']=('status_server')
        self.treeDB.column('#0',width=0,stretch="no")
        self.treeDB.column('status_server',width=125,anchor="center")
        self.treeDB.heading("status_server",text="Status",anchor="center")
        self.treeDB.tag_configure("db-error", background="#dc3545",foreground="white")
        self.treeDB.tag_configure("db-good", background="pale green",foreground="black")
        self.treeDB.pack()

        #======== Todays Count ==========
        frameCountToday = Frame(self.frame1)
        frameCountToday.pack(side="right", padx=(5,5))

        self.labelCntLine1= Label(frameCountToday,font=("Arial",18,'bold'),text="Todays Count", bg='#9ACD32')
        self.labelCntLine1.pack(side="top",fill="x")

        self.labelCntValue = Label(frameCountToday,font=("Arial",18,'bold'),text="00000", bg='#9ACD32')
        self.labelCntValue.pack(side="bottom",fill="x")


        #===== REAL TIME CLOCK =======
        frameClock = Frame(self.frame1)
        frameClock.pack(side="right", padx=(10,10))

        self.labelTime = Label(frameClock,font=("Arial",18,'bold'),bg='yellow')
        self.labelTime.pack(fill="both")
        self.run_clock()

        

        #========= CLIENTS FRAME ============
        
        self.frameClients = Frame(self.master)
        self.frameClients.pack(fill="x")

        lTitle=Label(self.frameClients,text="RFID READER STATUS",pady=0,bg='#008888',fg='#FFFFFF',font=(0,TITLE_SIZE))
        lTitle.pack(fill="x",side="top")

        
        self.frame2 = Frame(self.frameClients,bg='#600000',padx=20,pady=0)  #hide this frame by setting width =0 
        #self.frame2.pack(side="left") #DONT PACK

        l2=Label(self.frame2,text="CLIENTS STATUS - WITH CAMERAS",pady=0,bg='#008888',fg='#FFFFFF',font=(0,TITLE_SIZE))
        l2.pack(fill="x")

        self.treeClientsWithCameras = Treeview(self.frame2,height=1,selectmode="none")
        self.treeClientsWithCameras['columns']=('pen','reader','status_asr','asr','camera','status_camera')
        
        self.treeClientsWithCameras.column('#0',width=0,stretch="no")
        self.treeClientsWithCameras.column('pen',width=50,anchor="e")
        self.treeClientsWithCameras.column('reader',width=80,anchor="e")

        self.treeClientsWithCameras.column('status_asr',width=120,anchor="center")
        self.treeClientsWithCameras.column('asr',width=150,anchor="center")
        self.treeClientsWithCameras.column('camera',width=150,anchor="center")
        self.treeClientsWithCameras.column('status_camera',width=120,anchor="center")
        
        self.treeClientsWithCameras.heading("pen",text="Pen#",anchor="center")
        self.treeClientsWithCameras.heading("reader",text="Reader#",anchor="center")
        
        self.treeClientsWithCameras.heading("status_asr",text="Connected",anchor="center")
        self.treeClientsWithCameras.heading("asr",text="IP ASR650",anchor="center")
        self.treeClientsWithCameras.heading("camera",text="IP Azure Kinect",anchor="center")
        self.treeClientsWithCameras.heading("status_camera",text="Connected",anchor="center")


        self.treeClientsWithCameras.pack()


             
        self.frameStandalones = Frame(self.frameClients,bg='lavender',padx=20,pady=0)
        self.frameStandalones.pack(fill="x")

        self.frameStandalonesConnectionStatus = Frame(self.frameStandalones)
        self.frameStandalonesConnectionStatus.pack(side="left", expand=True, fill="x",padx=10)

        labelConnStatus=Label(self.frameStandalonesConnectionStatus,text="Connection Status",pady=5,bg='RoyalBlue4',fg='#FFFFFF', font=(0,TITLE_SIZE))
        labelConnStatus.pack(fill="x")
        
        #self.treeStandAlones = Treeview(self.frameStandalones,height=10)
        self.treeStandAlones = Treeview(self.frameStandalonesConnectionStatus,height=1,selectmode="none")
        
        self.treeStandAlones['columns']=('pen','reader','asr','status_asr')
        
        self.treeStandAlones.column('#0',width=0,stretch="no")
        self.treeStandAlones.column('pen',width=20,anchor="e")
        self.treeStandAlones.column('reader',width=30,anchor="e")
      
        self.treeStandAlones.column('asr',width=100,anchor="center")
        self.treeStandAlones.column('status_asr',width=100,anchor="center")
        
        
        self.treeStandAlones.heading("pen",text="Pen #",anchor="center")
        self.treeStandAlones.heading("reader",text="Reader #",anchor="center")

        self.treeStandAlones.heading("status_asr",text="Status",anchor="center")
        self.treeStandAlones.heading("asr",text="IP ASR650",anchor="center")
       

        #####self.treeStandAlones.pack(fill="x")
        self.treeStandAlones.pack(side="left", expand=True, fill="x")  #So we can add antenna hits on the right

        #update tree background colors for tags
        self.treeStandAlones.tag_configure("not-connected", background="#dc3545",foreground="white")
        self.treeStandAlones.tag_configure("connected", background="pale green",foreground="black")
        
        '''
        FOR MSU - add the antenna hit counter in the main window
        '''
        self.frameAntennaReadsMAIN = Frame(self.frameStandalones, pady=10)
        self.frameAntennaReadsMAIN.pack(side="left", expand=True, fill="x")

        labelAreadsMAIN=Label(self.frameAntennaReadsMAIN,text="HITS Recorded/Reader/Pen/Antenna",pady=5,bg='RoyalBlue4',fg='#FFFFFF', font=(0,TITLE_SIZE))
        labelAreadsMAIN.pack(fill="x")

        self.treeAntennaMAIN = Treeview(self.frameAntennaReadsMAIN,height=1)
        #self.treeAntennaMAIN['columns']=('reader','pen','ant0', 'ant1','ant2','ant3','ant4','ant5','ant6','ant7','ant8')
        self.treeAntennaMAIN['columns']=('reader','pen','ant0', 'ant1','ant2','ant3','ant4')  #MSU only 4 antennas
        
        self.treeAntennaMAIN.column('#0',width=0,stretch="no")
        self.treeAntennaMAIN.column('reader',width=80,anchor="center")
        self.treeAntennaMAIN.column('pen',width=50,anchor="center")
        self.treeAntennaMAIN.column('ant0',width=100,anchor="center")
        self.treeAntennaMAIN.column('ant1',width=100,anchor="center")
        self.treeAntennaMAIN.column('ant2',width=100,anchor="center")
        self.treeAntennaMAIN.column('ant3',width=100,anchor="center")
        self.treeAntennaMAIN.column('ant4',width=100,anchor="center")
        ''' FOR MSU - no need for antennas 5-8
        self.treeAntennaMAIN.column('ant5',width=100,anchor="center")
        self.treeAntennaMAIN.column('ant6',width=100,anchor="center")
        self.treeAntennaMAIN.column('ant7',width=100,anchor="center")
        self.treeAntennaMAIN.column('ant8',width=100,anchor="center")
        '''


        self.treeAntennaMAIN.heading("reader",text="Reader",anchor="center")
        self.treeAntennaMAIN.heading("pen",text="Pen",anchor="center")
        self.treeAntennaMAIN.heading("ant0",text="Ant Single",anchor="center")
        self.treeAntennaMAIN.heading("ant1",text="Ant #1",anchor="center")
        self.treeAntennaMAIN.heading("ant2",text="Ant #2",anchor="center")
        self.treeAntennaMAIN.heading("ant3",text="Ant #3",anchor="center")
        self.treeAntennaMAIN.heading("ant4",text="Ant #4",anchor="center")
        '''
        self.treeAntennaMAIN.heading("ant5",text="Ant #5",anchor="center")
        self.treeAntennaMAIN.heading("ant6",text="Ant #6",anchor="center")
        self.treeAntennaMAIN.heading("ant7",text="Ant #7",anchor="center")
        self.treeAntennaMAIN.heading("ant8",text="Ant #8",anchor="center")
        '''

        self.treeAntennaMAIN.pack(fill="x")

        #update tree background colors for tags
        self.treeAntennaMAIN.tag_configure("active", background="green")
        self.treeAntennaMAIN.tag_configure("passive", background="white")




        self.treeClientsWithCameras.tag_configure("not-connected", background="#dc3545",foreground="white")
        self.treeClientsWithCameras.tag_configure("connected", background="pale green",foreground="black")
        self.treeClientsWithCameras.tag_configure("partially-connected", background="yellow",foreground="black")

        self.update()

        #TreeView for localhosts
        self.frameLocalhostClients = Frame(self.master,bg='#600000')
        self.frameLocalhostClients.pack(fill="x")

              

       
      

        #Next row will be some buttons/checkboxes to handle some simple things like reloading json or enabling messages to be looged
        frameTestButtons = Frame(self.master,bg="coral1", pady=5)
        frameTestButtons.pack()
        
        #Add a reset button to set antenna counts to 0
        btnResetAntennaCounts = Button(frameTestButtons,text='Reset Antenna Counts',command=self.resetAntennaHits)
        btnResetAntennaCounts.pack(side="right",padx=10)

        #8/29/2024 - Add checkbox for logging check Rx/Tx on demand
        checkboxLogTxRx = Checkbutton(frameTestButtons,text='LogTxRx', 
                        variable=self.varCheckBoxLogTxRx,
                        onvalue=1,offvalue=0,
                        )
        checkboxLogTxRx.pack(side="right")
        #end add 8/29/2024

        
        checkboxDetails = Checkbutton(frameTestButtons,text='Show Details', 
                        variable=self.varCheckboxDetails,
                        onvalue=1,offvalue=0,
                        )
        checkboxDetails.pack(side="right")

        checkboxQuietTerminal = Checkbutton(frameTestButtons,text='Quiet Terminal', 
                        variable=self.varCheckboxQuiet,
                        onvalue=1,offvalue=0,
                        )
        checkboxQuietTerminal.pack(side="right")

        self.update()

        #status bar
        self.statusbar = Label(frameStatus,text="Load the settings.json file to begin", bd=1, relief=tkinter.RIDGE, anchor="se") #relief=tkinter.SUNKEN,
        self.statusbar.pack(fill="x")

        ##################  MESSAGE WINDOW TAB ################
        frameStatus = Frame(self.tabMessage,highlightbackground="white", highlightthickness=4)
        frameStatus.pack(pady=(5,5))
        frameInfo = Label(frameStatus, text='Message Window', font=('Arial',14))
        frameInfo.pack(side="top")
        self.text_widget= Text(frameStatus,width=480) #msu - no gui_height
        self.text_widget.pack()

        tex_scroll = Scrollbar(orient="vertical",)
    
        tex_scroll.config(command=self.text_widget.yview, )
        self.text_widget["yscrollcommand"] = tex_scroll.set
    

        self.update()

        
        ##################  THREADS TAB ##################
        
        self.frameThreads = Frame(self.tab2,bg='#600000',width=750)  #remove height=750
        self.frameThreads.pack(fill="both")

       
        labelThreads=Label(self.frameThreads,text="Active Threads",pady=0,bg='#008888',fg='#FFFFFF')
        labelThreads.pack(fill="x")

        self.treeThreads = Treeview(self.frameThreads)
        self.treeThreads['columns']=('threadNo','IP','Port','Created','Tx','Rx')
        
        self.treeThreads.column('#0',width=0,stretch="no")
        self.treeThreads.column('threadNo',width=100,anchor="e")
        self.treeThreads.column('IP',width=120,anchor="e")
        self.treeThreads.column('Port',width=60,anchor="e")
        self.treeThreads.column('Created',width=220,anchor="center")
        self.treeThreads.column('Tx',width=250,anchor="center")
        self.treeThreads.column('Rx',width=250,anchor="center")


        self.treeThreads.heading("threadNo",text="Thread#",anchor="center")
        self.treeThreads.heading("IP",text="IP",anchor="center")
        self.treeThreads.heading("Port",text="Port",anchor="center")
        self.treeThreads.heading("Created",text="Created@",anchor="center")
        self.treeThreads.heading("Tx",text="Tx(Bytes)",anchor="center")
        self.treeThreads.heading("Rx",text="Rx(Bytes)",anchor="center")
        
        self.treeThreads.pack()
        
        #################### Antenna Reads tab ################################
        self.frameAntennaReads = Frame(self.tab3)
        self.frameAntennaReads.pack(fill="x")

       
        labelAreads=Label(self.frameAntennaReads,text="HITS recorded by individual antenna",pady=5,bg='#008888',fg='#FFFFFF', font=(0,30))
        labelAreads.pack(fill="x")

        self.treeAntenna = Treeview(self.frameAntennaReads,height=9)  #MSU - this is controlling the height that write over status bar
        self.treeAntenna['columns']=('reader','pen','ant0', 'ant1','ant2','ant3','ant4','ant5','ant6','ant7','ant8')
        
        self.treeAntenna.column('#0',width=0,stretch="no")
        self.treeAntenna.column('reader',width=80,anchor="center")
        self.treeAntenna.column('pen',width=50,anchor="center")
        self.treeAntenna.column('ant0',width=100,anchor="center")
        self.treeAntenna.column('ant1',width=100,anchor="center")
        self.treeAntenna.column('ant2',width=100,anchor="center")
        self.treeAntenna.column('ant3',width=100,anchor="center")
        self.treeAntenna.column('ant4',width=100,anchor="center")
        self.treeAntenna.column('ant5',width=100,anchor="center")
        self.treeAntenna.column('ant6',width=100,anchor="center")
        self.treeAntenna.column('ant7',width=100,anchor="center")
        self.treeAntenna.column('ant8',width=100,anchor="center")


        self.treeAntenna.heading("reader",text="Reader",anchor="center")
        self.treeAntenna.heading("pen",text="Pen",anchor="center")
        self.treeAntenna.heading("ant0",text="#0",anchor="center")
        self.treeAntenna.heading("ant1",text="#1",anchor="center")
        self.treeAntenna.heading("ant2",text="#2",anchor="center")
        self.treeAntenna.heading("ant3",text="#3",anchor="center")
        self.treeAntenna.heading("ant4",text="#4",anchor="center")
        self.treeAntenna.heading("ant5",text="#5",anchor="center")
        self.treeAntenna.heading("ant6",text="#6",anchor="center")
        self.treeAntenna.heading("ant7",text="#7",anchor="center")
        self.treeAntenna.heading("ant8",text="#8",anchor="center")

        self.treeAntenna.pack(side="left")

        #update tree background colors for tags
        self.treeAntenna.tag_configure("active", background="green")
        self.treeAntenna.tag_configure("passive", background="white")

        btnTest = Button(self.frameAntennaReads,text='Reset',command=self.resetAntennaHits)
        btnTest.pack(side="right")

        self.update()



        ################ ASR Info Tab ##################
        fntFrames=10
        
        self.TxFrame=Frame(self.tabASRInfo,pady=10,highlightbackground="blue", highlightthickness=4)
     
           
        self.checkbox = Checkbutton(self.TxFrame,text='Enable', 
                        variable=self.varCheckbox,
                        onvalue=1,offvalue=0,command=self.enableASRInfo,
                        font=(0,fntFrames))
        self.checkbox.pack(side="left")
        ToolTip(self.checkbox,msg="Need to explicitly enable this to see the list of ASR's.\n\nNote the communication will be paused - don't forget to press CONTINUE to resume communications")

      

        ButtonSendAll = Button(self.TxFrame,text='Send To All', command = self.sendMessageAll)
        ButtonSendAll.pack(side="right")
        ToolTip(ButtonSendAll,"Send the selected commands sequentially to all ASRS650's listed")

        self.btnReloadCommands = Button(self.TxFrame, text="Reload_Commands",font=(0,fntFrames),command=self.loadASRCommands)
        self.btnReloadCommands.pack(side="right")

        self.btnCRC= Button(self.TxFrame, text="CRC",font=(0,fntFrames),command=self.appendCRC)
        self.btnCRC.pack(side="right")
        ToolTip(self.btnCRC,msg="CRC's for the stored messages are already included - no need to click this.\n\nIf you are trying a new manual command, click on the CRC button to compute and include the CRC")

        self.entryMSG = Entry(self.TxFrame,bd=5, width=50,font=(0,fntFrames))
        self.entryMSG.pack(side="right")
        ToolTip(self.entryMSG,msg="Automatically filled for pre-defined messages. If you are typing your own - please make sure the CRC and ETX is included.")


            
        #combobox
        self.stringVarCombo = StringVar()

        self.combo=Combobox(self.TxFrame,values=list(self.commands.keys()), justify="center", textvariable=self.stringVarCombo, width=30)
        #Highlight the first one
        self.combo.current(0)
        self.updateCommand([])#to put the command in the entry box
        self.combo.bind('<<ComboboxSelected>>', self.updateCommand)
        self.combo.pack(side="right")

        L1 = Label(self.TxFrame, text="Message To Send",font=(0,fntFrames,"bold"))
        L1.pack(side="right")

        self.TxFrame.pack(fill="x")
        #self.RxFrame.pack()
        
        #this frame is filled with table later on after the ASR's are connected
        self.ASRInfoFrame=Frame(self.tabASRInfo)
        self.ASRInfoFrame.pack()
        self.ASRInfoFrame.pack()



        #DEBUG TEST FRAME
        self.frameTest = Frame(self.tabASRInfo)
        self.frameTest.pack()


        self.btnsASRSend=[]
        self.btnsASRClear=[]

        #Debug frame
        self.hexToDecFrame=Frame(self.tabASRInfo)

        self.LabelDec = Label(self.hexToDecFrame, text="---")
        self.LabelDec.pack(side="right")

        self.textVarHexToDec = StringVar()
        self.hexVal = Entry(self.hexToDecFrame,textvariable=self.textVarHexToDec)
        self.hexVal.pack(side="left")

        self.btnConvert= Button(self.hexToDecFrame, text="Convert",font=(0,12),command=self.convertHexToDec)
        self.btnConvert.pack(side="right")

        self.hexToDecFrame.pack(side="bottom")

       ###### Version Info Tab
        
        tabs = self.notebook.tabs()
        cntTabs = len(tabs)
        self.notebook.tab(cntTabs-1,text="Version: "+str(VERSION))
        txtVersion = Text(self.tabVersionInfo)
        txtVersion.insert("end",VERSION_MESSAGE)
        txtVersion.pack(fill="both")
        

        #For now load the json without having to open a dialog
        #self.pauseComm=True
        try:
            self.json_file = "settings.json"
            print("JSON FILE = ".format(self.json_file))
            self.load_json()

            #JSON Worked
            jsonWorked = True

        except Exception as e:
            print(repr(e))
            logger.error(f"JSON LOAD ERROR => {repr(e)}")
            self.prompt_json()

        #IF JSON is OK, start the main routine
        self.startMainRoutine()

    def run_clock(self):
        now = dt.now()
        time_string = dt.strftime(now, '%I:%M:%S %p\n%b %d %Y')
        self.labelTime.config(text=time_string)
        self.labelTime.after(1000,self.run_clock)

        #at midnight, log midnight
        if((now.hour==0) and (now.minute==0) and (now.second==45)):
            pass

 
    #Clear ASR650 Rx entry box
    def clearRXEntry(self,entry):
        entry.delete(0,"end")
        entry.insert(0,".....")
    
    
    #Certain commands to be sent to RFID Reader(ASR650) can be loaded from a json file
    #and sent through this App.
    def loadASRCommands(self):
        try:
            with open("commands.json") as fp2:
                    json_commands = json.load(fp2)

                    self.commands = {}
                    
                    for cmd in json_commands["commands"]:
                        ks = list(cmd.keys())
                        vs = list(cmd.values())

                        self.commands[ks[0]]=vs[0]

                    #update the combobox
                    self.combo.config(values=list(self.commands.keys()))
        except Exception as ex:
            messagebox.showerror(title="List of Commands For ASR 650", message = "Either could not load commands.json or something is incorrect in the json file. Exception Message="+repr(ex))
            #Could not open the commands.json file or may be it was incorrect - so manually add at least one command
       
    #simple hex to decimal converter for debugging
    def convertHexToDec(self):
        self.LabelDec.config(text="") #clear output
        hx = self.hexVal.get() #get hex
        hx = hx.strip() #strip out spaces before after
        if(hx != ""):
            self.LabelDec.config(text=int(hx,16))

    #When sending non catalog messages to ASR650, we need to generate and append the CRC followed by ETX
    #this function handles the CRC generation and appending.
    def appendCRC(self):
        msg_incomplete = self.entryMSG.get()
        msg_incomplete = msg_incomplete.strip()  #remove leading/trailing spaces
        read_bytes=bytes.fromhex(msg_incomplete)
        crc = crc_poly(read_bytes, 8, 0x1D, 0, ref_in=True, ref_out=True) #crc returned is INT
        crc_hex = hex(crc).upper()  #hex string has 0x padded
        crc_hex=crc_hex[2:] #get rid of 0x
        #crc_bytes=crc.to_bytes(1,'little')
        final_hex_string = msg_incomplete + " " + crc_hex + " 03"
        self.entryMSG.delete(0,"end")
        self.entryMSG.insert(0,final_hex_string)
        pass

    #Display the selected ASR650 Tx message and display it
    def updateCommand(self,event):
        #self.RxEntry.delete(0,"end") #Clear RX field
        self.entryMSG.delete(0,"end") #delete current TX mesasge
        self.entryMSG.insert(0,self.commands[self.stringVarCombo.get()])
        self.update()

    #ASR Info tab - checkbox needs to be checked to enable other TX to ASR650
    #This is so that we can pause the read collection.
    def enableASRInfo(self):

        #This enables the table that lets you send messages to specific asr's.
        #Note: This also pauses the main communication to all ASR's.
        if self.varCheckbox.get()==1:

            if(not self.pauseComm):
                self.pauseCommunications() #pause the main communications for now
            
          
            #first delete all chidren items from the frame
            if(self.frameTest):
                for child in self.frameTest.winfo_children():
                    child.destroy()

            #get the keys
            keys_info = self.threadsMoreInfo.keys()
            keys_info_sorted = sorted(keys_info)

            #for i,key in enumerate(self.threadsMoreInfo):
            for i,key in enumerate(keys_info_sorted):

                k=i+1

                ip=key[0]
                penNo=self.getPenNo((ip,None)) #4/5/2023
                info = self.threadsMoreInfo[key]
                conn = info[1]
                type = info[2]

                #If this is a camera, do not bother as we are only interested in sending to ASRs
                if(self.isThisACamera(ip)):
                    continue

                #Create new frame to hold the CRUD rows    
                frame = Frame(self.frameTest)
                frame.pack()

                fntFrames = 10
                LabelPen = Label(frame,text="Pen#"+str(penNo),font=("Arial",fntFrames),fg="blue",borderwidth=1, relief="sunken",padx=5)
                LabelIP = Label(frame,text=ip,font=(0,fntFrames),fg="blue")
                LabelSt = Label(frame,text="...",font=(0,fntFrames),fg="blue",width="20")
                EntryResp = Entry(frame, width=50)
                ButtonSend = Button(frame,text='Send', command = lambda i=i, k=k, l=LabelSt, e=EntryResp,ip=ip,c=conn: self.sendMessage2(i,l,ip,c,e),font=(0,fntFrames))
                ButtonClear = Button(frame,text='Clear', command = lambda e=EntryResp: self.clearRXEntry(e),font=(0,fntFrames))
                self.btnsASRSend.append(ButtonSend)
                self.btnsASRClear.append(ButtonClear)
                LabelPen.pack(side="left")
                LabelIP.pack(side="left")
                
                EntryResp.pack(side="right")
                LabelSt.pack(side="right")
                ButtonSend.pack(side="right")

            #Update the width to 100%
            #==>THIS WRITES OVER THE statusbar self.tabASRInfo.config(height=GUI_HEIGHT-STATUS_BAR_HEIGHT)

        else:
            #first delete all chidren items from the frame
            
            if(self.frameTest):
                for child in self.frameTest.winfo_children():
                    child.destroy()

            self.statusbar.config(text="NOTE Communication Still Paused, Restart IT if needed")
            
 
    def sendMessage(self,ip,conn):
        
        self.textVarRX.set("------")
        self.update()
        #showinfo("Test",f"Going to send message {self.entryMSG.get()} to {ip}")
        msb_bytes = bytes.fromhex(self.entryMSG.get())
        conn.sendall(msb_bytes)
        time.sleep(2)
        recv = conn.recv(1024)
        #self.Rx.config(text=recv.hex())
        msg_recv = recv.hex()
        n=2
        msg_new = [(msg_recv[i:i+n]) for i in range(0, len(msg_recv), n)]
        msg_with_spaces = " ".join(msg_new)

        self.textVarRX.set(msg_with_spaces)

    
    #Send the selected message to all RFID readers
    def sendMessageAll(self):

        for btn in self.btnsASRSend:
            btn.invoke()

        pass

    #Just show the sending... text
    def sendMessage2(self,i,label, ip,conn,entry):

        label.config(text="Sending..")
        entry.delete(0,"end")
        entry.insert(0,".........")
        self.update()

    #Return the index corresponding to the reader
    #based on locattion in the antennaHits list
    def indexReader(self,reader):
        for indx,readerMain in enumerate(self.antennaHits):
            if reader==readerMain:
                return indx

        return -1 #return not found -1 by default

    #When RESET antenna hits is pressed, reset all values to 0.
    #This makes it easier to see if the antennas are working when you start from 0
    def resetAntennaHits(self):

        for key in self.antennaHits:

            cur = self.antennaHits[key]  #should be [penNo, N1, N2, N3, N4, N5, N6, N7, N8]
            hitsZero = [0 for x in cur[1:]]  #make the hits 0
            cur[1:]=hitsZero  #note this assignment alters the original self.antennaHits[key] to have 0's
                              #had we done cur=[0,0,0,0,0], it would not affect the original list

            indx = self.indexReader(key)
            lst = []
            lst.append(key)
            lst.extend(cur)

            self.treeAntenna.item(indx,text='',values=tuple(lst),tag=('passive'))
            self.treeAntennaMAIN.item(indx,text='',values=tuple(lst[0:-4]),tag=('passive'))

        pass

    #Just to         
    def testAntennaHits(self):

        ''''
            Everytime this button is pushed, a random reader's random antenna hit bumps up by 1.   
        
        '''

        #randomReader = random.randint(0,len(self.antennaHits))
        readers =[reader for reader in self.antennaHits]
        random.shuffle(readers)
        randomReader = readers[0]
        print(randomReader)
        #Choose a random antenna
        randomAntenna = random.randint(1,5)
        print(randomAntenna)
        
        #index of the reader
        indx = self.indexReader(randomReader)

        #Current antanna value -> bump up by 1
        hits = self.antennaHits[randomReader]
        hits[randomAntenna+1]+=1
        #self.antennaHits[randomReader]=hits #not needed
        
        lst = []
        lst.append(randomReader)
        lst.extend(hits)
        self.treeAntenna.item(indx,text='',values=tuple(lst),tag=('active'))
        self.treeAntennaMAIN.item(indx,text='',values=tuple(lst[0:-4]),tag=('active'))
        self.update()
        time.sleep(2)
        self.treeAntenna.item(indx,text='',values=tuple(lst),tag=('passive'))
        self.treeAntennaMAIN.item(indx,text='',values=tuple(lst[0:-4]),tag=('passive'))
        self.update()


    #Handle when user changes tabs
    def notebookTabChanged(self,event):

        index = self.notebook.index("current") #Which tab is pressed

        '''
        As of 8/7/2022 - 
        0 => Main
        1 => Message Window
        2 => Threads
        3 => Read Info
        4 => ASR Info
        5 => CameraPC
        6 => Version
        '''

        #if not in the ASR INFo tab, delete any frames used in the ASR info list of buttons
        if(index != 4):

            #If in any other tab but ASR Info, clear the ASR info content

            #first delete all chidren items from the frame
            if(self.frameTest):
                for child in self.frameTest.winfo_children():
                    child.destroy()
                #also clear the list of the buttons that we had added
                self.btnsASRSend = []
            
            #if enable is check disable
            self.varCheckbox.set(False)
            #self.enableASRInfo()

        #if in the antenna reads tab
        if(index==3):
            self.threadsTabActive=True 
            start_new_thread(self.updateThreadsTabActively,())
        else:
            self.threadsTabActive=False




    def updateThreadsTabActively(self):
        logger.debug(f"[New Thread][updateActiveThreads]")
        self.prnt("Starting update threads actively")

        while True:
            
            if(self.threadsTabActive == False):
                break;        

            try:
                self.clearTableThreads()

                #Update threads
                iidxT = 0
                for thread in self.threads:
                    try:
                        ip=thread[0]
                        port=thread[1]

                        more_info = self.threadsMoreInfo[thread]
                        tx_rx = self.RxTx[ip]
                        self.treeThreads.insert(parent='',index='end',iid=iidxT,text='',values=(iidxT+1,thread[0],thread[1],more_info[0],tx_rx[0],tx_rx[1]))     
                        iidxT=iidxT+1
                        
                    except KeyError:
                        logger.error(f"update_threads .. keyError on thread {thread}")
                    except Exception as e:
                        logger.error(f"update_threads .. general error str(e)={str(e)}")
            except Exception as e:
                logger.error(f"update_threads .. general error str(e)={str(e)}")      

            finally:
                 Event().wait(2.0)

    def pauseCommunications(self):
        if(self.pauseComm):
            self.pauseComm = False
            self.btnPause.config(text='Pause',bg='#FFE589',highlightbackground='#FFE589',fg='#000000')

            self.statusbar.config(text='Communication Resumed ...')
            self.statusbar.config(bg='#F0F0F0',fg='black')

            self.update()
        else:
            self.pauseComm = True
            self.btnPause.config(text='Continue',highlightbackground='#f44336',bg='#f44336',fg='#000000')
            self.statusbar.config(text='Communication Paused ...')
            self.statusbar.config(bg='red',fg='white')
            self.update()



    def emptyFeederData(self):
        #{NOT IN SQLITE}sql="DELETE FROM {}.{} WHERE id>0".format(mdbh.database,"feeder_data_new")
        sql="DELETE FROM {} WHERE id>0".format("feeder_data_new")
        tpl = ("DELETE_ALL",sql)
        self.dbqueueDeleteRecords.put(tpl) #add to the queue so that the db thread can execute it
        
        #mdbh.cursor.execute(sql)
        #mdbh.db.commit()
        #cntDeleted = mdbh.cursor.rowcount

        #msg = "Delete all {} records from feeder_data_new".format(cntDeleted)
        #showinfo(title="Database", message=msg)
    
    def emptyFeederDataToday(self):

        today_str = dt.strftime(dt.now(),"%Y-%m-%d")
        #[SQLITE-WONT WORK] sql="DELETE FROM {}.{} WHERE date(readtime) = '{}' and id>0".format(mdbh.database,"feeder_data_new",today_str)
        sql="DELETE FROM {} WHERE date(readtime) = '{}' and id>0".format("feeder_data_new",today_str)
        tpl=("DELETE_TODAY",sql)
        self.dbqueueDeleteRecords.put(tpl) #add to the queue so that the db thread can execute it
        
        #mdbh.cursor.execute(sql)
        #mdbh.db.commit()
        #cntDeleted = mdbh.cursor.rowcount

        #msg = "Deleted {} records from feeder_data_new that belonged to today {}".format(cntDeleted, today_str)
        #showinfo(title="Database", message=msg)

    def initParameters(self):

        self.host=''
        self.port=0
        self.matlab=0
        self.json_file = None
        self.sockets = {}
        self.threads = []
        self.threadsMoreInfo={}
        self.LocalhostClients = []  #[ (type, ip, port),("matlab","127.0.0.1",62345), ("asr","192.168.0.101",62222)   ]     ]
        self.localhostCounts = OrderedDict()

        self.ThreadCount = 0
        self.asr_camera_status = []
        self.asr_standalone_status=[]
        self.ServerSocket = None
        self.pauseComm = False
        self.dbqueue = queue.Queue(maxsize=100000) #incase db does down and comes backup
        self.dbqueueCaptureStatus = queue.Queue(maxsize=1000) #incase db does down and comes backup
        self.dbqueueDeleteRecords = queue.Queue(maxsize=10) #incase db does down and comes backup
        
        
        self.writingToDBInProgress = False
        self.writingToDBCaptureStatusInProgress = False
        
        self.standalone_readers=[]
        self.treeStandAlones=[]
        

        self.fakePens=[(2,"1A"),(3,"1B"),(4,"1C"),(5,"2A"),(6,"2B"),(7,"2C"),(2,"1D"),(3,"1E"),(3,"1F"),(4,"1G"),(4,"1H"),(5,"2D"),(6,"2E"),(6,"2F"),(7,"2G"),(7,"2H")]
        self.fakePenIndex=0

        self.timeoutTrigger_ms = IntVar()
        self.timeoutTrigger_ms.set(1000)
        self.triggerInterval_ms = IntVar()
        self.triggerInterval_ms.set(8000)


        #debug related
        self.debug_displayParsedData = False
        self.debug_displayProcessedMessage = False
        self.debug_threadedClient = False
        self.debug_updateQueue= False

        #cleanup related
        self.cleanupQueue=[]

        #Unknown ip related
        self.unknownIPs={}

        #Rx/Tx status on ips
        self.RxTx = dict()
        self.threadsTabActive=False

        #Antenna hits counts
        self.antennaHits = {}

        #Commands for ASR
        self.commands={"get_serial_number":"02 FF F0 08 2F 03"} #4/4/2023
        self.btnsASRSend=[]
        self.btnsASRClear=[]
        self.frameTest = None
        self.varCheckbox = IntVar()

        self.framePCControl=None
        self.btnsPCSend=[]
        self.framesPCList = []

        self.varCheckboxDetails = IntVar()
        self.varCheckboxDetails.set(0)

        self.varCheckboxQuiet=IntVar()
        self.varCheckboxQuiet.set(1)

        #8/29/2021
        self.varCheckBoxLogTxRx = IntVar()
        self.varCheckBoxLogTxRx.set(0)
        #end new addition 8/29/2024

        self.LabelTodaysCount=""

      

        #10/20
        self.cntToday = 0 #make todays count 0, it will be updated by calling SELECT COUNT(*) every M minues
                          #in between queries, it will be updated based cntToday + numRecordsInsertedSInce
        self.numDBWrites = 0   #since it runs at 2 second intervals, 10 minutes would be 300 attempts that would take 600 seconds i.e 10 minutes
        self.numRecordsInsertedSinceLastRead = 0 


        #04/04/2023
        self.trigger_response_count_tx = None
        self.trigger_response_count_rx = None        

        
    def startMainRoutine(self):     

        self.statusbar.config(text='.................',fg='black')

        for (asr,camera,penNo,readerNo) in self.asr_camera_pairs:
            
            self.sockets[asr]=(0,penNo) #meaning no forwarding client has been assigned yet
            self.asr_camera_status.append([0,0])
            self.antennaHits[readerNo]=[penNo,0,0,0,0,0,0,0,0,0]   #[pen A0 A1-A8], A0 for single anntennas


        for (asr,camera,penNo,readerNo) in self.standalone_readers:
            
            self.asr_standalone_status.append(0)
            self.antennaHits[readerNo]=[penNo,0,0,0,0,0,0,0,0,0]

           

        print("Sockets initialized as:")
        print(self.sockets)
        print("\n\nStatuses initialized as:")    
        print(self.asr_camera_status)

        self.updateTable()

        for indx,key in enumerate(self.antennaHits):
            lst = []
            lst.append(key)
            lst.extend(self.antennaHits[key])
            self.treeAntenna.insert(parent='',index='end',iid=indx,text='',values=tuple(lst))
            #FOR MSU - show only 4 antenna (dont use the last 4 entries corresponding to antennas 5-8)
            self.treeAntennaMAIN.insert(parent='',index='end',iid=indx,text='',values=tuple(lst[0:-4]))

        #self.treeAntenna.config(height=len(self.antennaHits)+10)  #changed 0 to 10


        #Run the tcp/ip loop
        self.statusbar.config(text='Status: .......Starting thread to listen for tcp clients',fg='black')
        start_new_thread(self.communicate,())
        #time.sleep(1.5)

        #Start the thread that writes to the the database
        self.statusbar.config(text='Status: .......Starting thread that writes to database',fg='black')
        start_new_thread(self.db_write, ())

        #time.sleep(1.5)
        self.statusbar.config(text='Status: .......Done with initialization. Hub running..',fg='black')


    def prompt_json(self):
        self.json_file = filedialog.askopenfilename()
        print("JSON FILE = ".format(self.json_file))
        self.load_json()

    #By default, full update not a quick update but can be triggered for quick updates of some simple settings
    def load_json(self,quickUpdate=0):
        
        #Load the settings json file
        if self.json_file:

            with open(self.json_file,"r") as fp:
                json_obj = json.load(fp)
                #print(json.dumps(json_obj,indent=4))

            if(not quickUpdate):   
                self.asr_camera_pairs = []

                for pen in json_obj["pens"]:

                    penNo = pen["no"]
                    readers = pen["readers"]

                    for reader in readers:
                        if reader["camera"] != "0":
                            self.asr_camera_pairs.append((reader["ip"],reader["camera"], penNo, reader["no"]))
                        else:
                            self.standalone_readers.append([reader["ip"], reader["camera"], penNo, reader["no"]])  #-1 means not selected from fakelist, once selected assign index of fakePens


                self.host=json_obj["tcpip_server"]["ip"]
                self.port=json_obj["tcpip_server"]["port"]

                self.treeServer.insert(parent='',index='end',iid=0,text='',values=(self.host,self.port,"Not Running"),tag=('not-running'))

            #some other settings
            self.triggerInterval_ms.set(json_obj["asr_settings"]["request_interval_ms"])
            self.timeoutTrigger_ms.set(json_obj["asr_settings"]["timeout_ms"])

            
            #8/29/2024: Added json setting to turn on/off TX/RX messages
            if "log_txrx" in json_obj:
                #self.logTxRx = json_obj["log_txrx"]
                self.varCheckBoxLogTxRx.set(json_obj["log_txrx"])
            else:
                #self.logTxRx = 0
                self.varCheckBoxLogTxRx.set(0)
        
        #Load the ASR commands from another json file
        self.loadASRCommands()

    def reloadSettingsParameters(self):
        try:
            self.status_msg("Reloading JSON")
            self.load_json(quickUpdate=1)
            self.status_msg("JSON reloaded..")
        except: 
            self.status_msg("Exception while reloading json. Check LOG file")
            pass 

    def status_msg(self,msg):
        self.statusbar.config(text=msg)

    def communicate(self):

        self.ServerSocket = socket.socket()
        self.ServerSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #so socket can be re-used

        try:
            #self.text_widget.insert(END,"\n\nAttempting to bind to {} at port# {}\n".format(self.host,self.port))
            msgAttempt = "Attempting to bind to {} at port# {}".format(self.host,self.port)
            self.prntForce(msgAttempt)

            #Actual bind call
            self.ServerSocket.bind((self.host, self.port))

            #print('Waiting for a Connection..')
            #self.text_widget.insert(END,"\nBinding worked.Waiting for connection from clients")
            self.prntForce("Binding worked.Waiting for connection from clients")

            for child in self.treeServer.get_children():
                self.treeServer.delete(child)
            self.treeServer.insert(parent='',index='end',iid=0,text='',values=(self.host,self.port,"Running"),tag=('running'))  
            
            self.ServerSocket.listen(5)
            
          
            #Start accepting connections
            self.acceptConnections()

            
        except socket.error as e:
            #print(str(e))
            msg=str(e)
            #self.text_widget.insert(END,msg)
            self.prntForce(msg)
            logger.error(msg)
            #showerror(title="TCP/IP Server",message="Could not run tcp/ip server at {} and port# {}.\nCheck the ip address of the computer and restart the program".format(self.host,self.port))



    def printToTerminal(self,msg):
        #04/25/2023 - make it quiter
        if(not self.varCheckboxQuiet.get()):  #if checked means wants quiet mode, if unchecked means want verbose mode
            print(msg)

        pass 

    def prnt(self,msg):

        if self.varCheckboxDetails.get()==1:
            #self.text_widget.insert(END,"\n\n"+msg)
            tm = dt.strftime(dt.now(),"%Y-%m-%d %H:%M:%S")
            msg = "\n" + "--"*10 + tm + "--"*10 + "\n" + msg
            msg += "\n"+"--"*30+"\n" 
            self.text_widget.insert("1.0","\n"+msg)
            print(msg)
    
    def prntForce(self,msg):

        #self.text_widget.insert(END,"\n\n"+msg)
        tm = dt.strftime(dt.now(),"%Y-%m-%d %H:%M:%S")
        msg = "\n" + "--"*10 + tm + "--"*10 + "\n" + msg
        msg += "\n"+"--"*30+"\n" 
        self.text_widget.insert("1.0",msg)
        print(msg)

    def updateStatus(self,address):
       
        #is the connected client part of asr/camera pair
        for indx,pair in enumerate(self.asr_camera_pairs):

            asr = pair[0]
            camera=pair[1]

            if asr==address[0]:
                self.asr_camera_status[indx][0]=1
                break
            
            if camera==address[0]:
                self.asr_camera_status[indx][1]=1
                break
        
        #is the client just a standalone reader
        for indx,pair in enumerate(self.standalone_readers):

            asr = pair[0]
            camera=pair[1]
            pen=pair[2]
            reader=pair[3]

            if asr==address[0]:
                self.asr_standalone_status[indx]=1
                break

           
        #update the table based on status
        self.updateTable()


    def updateTable(self):

        #first delete all chidren of the tree
        try:
            self.clearTableClients()
        except:
            logger.error('exception thrown while clearTableclients()')

        try:    
            self.clearTableClientsStandAlone()
        except:
            logger.error('exception thrown while clearTableclientsstandAlone()')
    
       

        try:
            self.clearTableThreads()
        except:
            logger.error('exception thrown while clearTableThreads()')
    
        

        for indx,pair in enumerate(self.asr_camera_pairs):
            st_asr = "No"
            st_camera="No"
            
            if(self.asr_camera_status[indx][0] == 1):
                st_asr = "Yes"
            if(self.asr_camera_status[indx][1] == 1):
                st_camera = "Yes"
            
            penNo=pair[2]
            readerNo=pair[3]
            #july 14,2022: show a ** in not coneected status
            if st_asr == "No":
                st_asr = "**"+ st_asr
            if st_camera == "No":
                st_camera = "**"+ st_camera


            self.treeClientsWithCameras.insert(parent='',index='end',iid=indx,text='',values=(penNo, readerNo, st_asr,pair[0],pair[1],st_camera))    
           
            if(st_asr=="Yes") and (st_camera=="Yes"):
                self.treeClientsWithCameras.item(indx,tag=('connected'))
            else:
                if(st_asr=="**No") and (st_camera=="**No"):
                    self.treeClientsWithCameras.item(indx,tag=('not-connected'))
                else:
                    self.treeClientsWithCameras.item(indx,tag=('partially-connected'))
       

        for indx,pair in enumerate(self.standalone_readers):
           
            st_asr = "No"
           
            if(self.asr_standalone_status[indx] == 1):
                #st_asr = "Yes"
                st_asr = "CONNECTED"
            
            if st_asr == "No":
                #st_asr = "**"+ st_asr
                st_asr = "NOT-CONNECTED"
            
            penNo=pair[2]
            readerNo=pair[3] 
            self.treeStandAlones.insert(parent='',index='end',iid=indx,text='',values=(penNo, readerNo, pair[0],st_asr))
            #if(st_asr=="**No"):
            if(st_asr=="NOT-CONNECTED"):
                self.treeStandAlones.item(indx,tag=('not-connected'))
            else:
                self.treeStandAlones.item(indx,tag=('connected'))      
        
        #Adjust tehe height of the two columns to be the same
        ##col_ht = max(len(self.asr_camera_pairs),len(self.standalone_readers))+0
        ##self.treeClientsWithCameras.config(height=col_ht)      
        ###self.treeStandAlones.config(height=col_ht) 

        #Localhost Clientss
        iidx = 0
        for client in self.LocalhostClients:
            if client[0] != "matlab":
                #format treeLocalhostClients: ("asr",ip,port,pen,reader)
                cnt = self.localhostCounts[client[4]]
                self.treeLocalhostClients.insert(parent='',index='end',iid=iidx,text='',values=(client[3], client[4], client[1],client[2],cnt))      
                iidx=iidx+1
                pass
       

        #Update threads
        iidxT = 0
        for thread in self.threads:
            try:
                more_info = self.threadsMoreInfo[thread]
                self.treeThreads.insert(parent='',index='end',iid=iidxT,text='',values=(iidxT+1,thread[0],thread[1],more_info[0]))     
                iidxT=iidxT+1
            except KeyError:
                logger.error(f"update_threads .. keyError on thread {thread}")
            except Exception as e:
                logger.error(f"update_threads .. general error str(e)={str(e)}")
        
        self.treeThreads.config(height= len(self.threads)+2) 

        self.update()
        
        #Update the db table
        if(mdbh.db):
            self.clearTableDB()
            self.treeDB.insert(parent='',index='end',iid=0,text='',values=("Running"),tag=('db-good'))
        else:
            self.clearTableDB()
            if(mdbh.status ==-1):
                #self.treeDB.insert(parent='',index='end',iid=0,text='',values=('localhost',"Settings.json Error"),tag=('db-error'))
                self.treeDB.insert(parent='',index='end',iid=0,text='',values=("Error(json)"),tag=('db-error'))
            elif (mdbh.status ==-2):
                #self.treeDB.insert(parent='',index='end',iid=0,text='',values=('localhost',"Connection Error"),tag=('db-error'))
                self.treeDB.insert(parent='',index='end',iid=0,text='',values=("Error(Conn)"),tag=('db-error'))
            else:
                #self.treeDB.insert(parent='',index='end',iid=0,text='',values=('localhost',"Error Status="+str(mdbh.status)),tag=('db-error'))
                self.treeDB.insert(parent='',index='end',iid=0,text='',values=("Error("+str(mdbh.status)+")"),tag=('db-error'))


            #showerror(title="MySQL Server",message="MySQL Connection Error\n\nClose the program. Check MYSQL Server is running, settings.json has correct settings and run the program again.")       
            msgDB="Database Issues. Could not establish connection. Reads will not be saved in database.."
            self.statusbar.config(text=msgDB,fg='red')
            #Also write to the message window 
            #self.text_widget.insert(END,"\n"+msgDB)
            self.prntForce(msgDB)
            
        

        

    def clearTableThreads(self):
        for item in self.treeThreads.get_children():
            self.treeThreads.delete(item)

    def clearTableClients(self):
        for item in self.treeClientsWithCameras.get_children():
            self.treeClientsWithCameras.delete(item)
    
    def clearTableClientsStandAlone(self):
        for item in self.treeStandAlones.get_children():
            self.treeStandAlones.delete(item)

    def clearTableClientsLocalhosts(self):
        for item in self.treeLocalhostClients.get_children():
            self.treeLocalhostClients.delete(item)
    
    

    def clearTableDB(self):
        for item in self.treeDB.get_children():
            self.treeDB.delete(item)
    
    def clientExists(self,address):


        pass

    def acceptConnections(self):
        #self.text_widget.insert(END,"\nStill waiting")

        while True:
            
            try:
                
                Client, address = self.ServerSocket.accept()

                #Are we expecting it?
                if( not self.isThisExpectedDevice(address)):
                  
                    #add to dict, the unknown ip
                    if(address[0] in self.unknownIPs):
                        self.unknownIPs[address[0]] += 1
                    else: 
                        self.unknownIPs[address[0]]=1

                    #if for the first time or if 1000th time
                    numAttempts = self.unknownIPs[address[0]]

                    if( (numAttempts== 1) or (numAttempts%1000 == 0) ):    
                        self.prnt("Unknown ip "+address[0]+". Closing connection")
                        logger.warning(f"[THREADED CLIENT]. Unwanted connection from {address[0]}, Num Attemps = {numAttempts}")
                #All expected devices will be allowed to create a thread to communicate    
                else:
                    '''
                    msg = 'Connected to Client: ' + address[0] + '@ port: ' + str(address[1])
                    print(msg)
                    self.prnt(msg)

                    #self.updateStatus(address)

                    start_new_thread(self.threaded_client, (Client, address))
                    self.ThreadCount += 1
                    msg='Thread Number: ' + str(self.ThreadCount)
                    self.prnt(msg)
                    self.LabelThreads.config(text=str(self.ThreadCount))
                    '''

                    threadCreated = False

                    if len(self.threads) == 0:  #if no threads yet, add to thread
                        ip_port=(address[0],address[1])
                        self.threads.append(ip_port)
                        self.threadsMoreInfo[ip_port]=[self.nowStr(),None,None]  #None,None to be replaced by socket and type of device
                        start_new_thread(self.threaded_client, (Client, address))
                        threadCreated = True
                    else:
                        #if this is not the first thread, check if the same ip/port exists??
                        cntCheckPrevThreads=0

                        for item in self.threads:

                            ip=item[0]
                            prt = item[1]

                            if((ip == address[0]) and (prt == address[1])):
                                
                                #msg="{} @ {} already has a socket. Not creating a new one".format(address[0],address[1])
                                msg="{} @ {} already has a socket. Cleaning up old socket and creating new one".format(address[0],address[1])
                                print(msg)
                                self.prnt(msg)
                                time.sleep(2)  #wait for 2 seconds in case the thread detected bad connection and started clearning it up.
                                self.cleanup_connection(address)
                                #Prepare to add the client again
                                ip_port = (address[0],address[1])
                                self.threads.append(ip_port)
                                self.threadsMoreInfo[ip_port]=[self.nowStr(),None,None]

                                start_new_thread(self.threaded_client, (Client, address))
                                threadCreated = True

                                break

                            else:
                                cntCheckPrevThreads = cntCheckPrevThreads + 1

                        #If all previous threads have been checked and no match found, create a new one
                        if(cntCheckPrevThreads == len(self.threads)):
                            
                            ip_port=(address[0],address[1])
                            self.threads.append(ip_port)
                            self.threadsMoreInfo[ip_port]=[self.nowStr(),None,None]

                            start_new_thread(self.threaded_client, (Client, address))
                            threadCreated = True    
                                


                    if threadCreated:

                        self.ThreadCount += 1
                        msg=f"Thread Number: {self.ThreadCount} for {address[0],address[1]})"
                        print(msg)
                        self.prnt(msg)
                        self.LabelThreads.config(text=str(self.ThreadCount))
                        self.LabelThreadsActive.config(text=str(len(self.threads)))

                        
                        
                        #Wait for a second to allow connections to stabilize before accepting another connection
                        time.sleep(1) #wait for 1 second for trhead creation to go through
                    
                    
                    for item in self.asr_camera_pairs:
                        asr = item[0]
                        camera = item[1]
                        penNo = item[2]

                        debug=False
                        if(debug):
                            msg="Checking asr={}, camera{} pair to check if address={} just connected".format(asr,camera,address[0])
                            print(msg)
                            self.prnt(msg)
                        
                        if camera == address[0]:
                            self.sockets[asr]=(Client,penNo)
                            msg="*** Camera/PC1 {} has connected. Assigning its Client into to sokets dictionary".format(address[0])
                            print(msg)
                            self.prnt(msg)
                            
                            if(debug):
                                print("########Updating sockets....")
                                print(self.sockets)
                
            except KeyboardInterrupt:
                logger.error(f"SERVER EXCEPTION--> keyboard inteerupt")
                break
            except Exception as e:
                logger.error(f"SERVER EXCEPTION --> General --> {e}")
        
        #self.treeServer.insert(parent='',index='end',iid=0,text='',values=(self.host,self.port,"Not Connected"))          
        self.ServerSocket.close()
        logger.error(f"Closing Server")

    def nowStr(self):
        return dt.strftime(dt.now(),"%Y-%m-%d %H:%M:%S")

    def infoIP(self,address):
        ip=address[0]
        port=address[1]

        if(len(self.asr_camera_pairs) > 0):
            for pair in self.asr_camera_pairs:
                if(ip==pair[0]):
                    return ("asr",ip,port,pair[2],pair[3])
                elif(ip==pair[1]):
                    return ("camera",ip,port,pair[2],"n/a")        

        if(len(self.standalone_readers) > 0):
            for asr in self.standalone_readers:
                if(ip==asr[0]):
                    return ("asr",ip,port,asr[2],asr[3])

        #if here, that means the above two checks did not return anything
        #and we dont know what this IP address is.
        return ("unknown",ip,port,"n/a","n/a")          


    def isThisACamera(self,ip):

        for indx,pair in enumerate(self.asr_camera_pairs):
            #if the second item in the pair matches the ip being checked, it is indeed a camera, break out
            if pair[1]==ip:
                return True
                break 

        #By default return False
        return False
            
    def isThisExpectedDevice(self,address):
        ip = address[0]

        if(ip != "127.0.0.1"): #localhosts are ok.
            for pair in self.asr_camera_pairs:
                if ip == pair[0] or ip==pair[1]:
                    return True
        
            for alone in self.standalone_readers:
                if ip==alone[0]:
                    return True
            #if neither in the pair or standalone, reject it
            return False 
        else:
            return True

    #1/30/2023 - return the pen number corresponding to the IP address
    def getPenNo(self,address):

        ip = address[0] #the ip address

        if(ip != "127.0.0.1"): #localhosts are ok.

            for pair in self.asr_camera_pairs:
                if ip == pair[0] or ip==pair[1]:
                    return pair[2] #(ip_asr,ip_pc,penNo,readerNo)
        
            for alone in self.standalone_readers:
                if ip==alone[0]:
                    return alone[2] #(ip_asr,0,penNo,readerNo)
                    
            #if neither in the pair or standalone, reject it
            return -1 
        else:
            return 0      
        

    #Added: 8/29/2024 --> copied from the jpl hub for soc students 10/5/2023 
    def readableRX(self,response):
        recvHexStr = response.hex() #will return something like 02aa0115d0108a0302f0010601101603
        recvReadable = [recvHexStr[i:i+2] for i in range(0,len(recvHexStr),2)] #return list with 2 character chunks
        recvReadableStr = " ".join(recvReadable)
        
        return recvReadableStr 
    


    #THIS FUNCTION IS HEART AND SOUL OF THE COMMUNICATIONS HUB
    #It creates a thread to talk to the client         
    def threaded_client(self,connection,address):

        data_buffer = []
        lastNAntennaReads = []
        byteReceived = 0

        #Add to the RxTx dict
        if address[0] not in self.RxTx:
            self.RxTx[address[0]] = [0,0]

        logger.debug(f"[CONNECTED][THREAD STARTED] ip={address[0]}, port={address[1]}")    

        #Send a message - ASR's wont care and will not respond to this. MATLAB will responsd with I AM MATLAB
        whatIsMyPen = self.getPenNo(address) #ADDED ON 1/30/2023
        welcome_msg="WELCOME TO THE HUB "+address[0]+"@"+str(whatIsMyPen)
        if(self.varCheckBoxLogTxRx.get()):
            logger.debug("[TX]" + welcome_msg)

        #8/29/2024 - string message to bytes
        msg_bytes = welcome_msg.encode("ascii")

        #msg_bytes_hdr=b'\x02\xAA\x00\xD0'+msg_bytes  #D0=>SEND HELLO TO PC/CAMERA/DASHBOARD - asr ignores it
        msg_bytes_hdr=b'\x02\xFF\xAA\xD0'+msg_bytes  #D0=>SEND HELLO TO PC/CAMERA/DASHBOARD - asr ignores it
        
        #Generate CRC
        crc = crc_poly(msg_bytes_hdr, 8, 0x1D, 0, ref_in=True, ref_out=True) #crc returned is INT
        crc_hex = hex(crc)
        crc_bytes=crc.to_bytes(1,'little')

        #Append the CRC and ETX
        msg_final = msg_bytes_hdr+crc_bytes+b'\x03'
        
        #Byte count tx/rx update
        tx_rx = self.RxTx[address[0]] #current tx, rx count
        tx_rx[0] += len(msg_final)  #update count to increment Tx

        #SEND message
        connection.send(msg_final)
        print(f"{'>'*90}  TX --> {welcome_msg}")

        #Send ASR's their own connect request message- ASR's will respond to this
        connection.send(bytes.fromhex("02FFF001F803"))  #ASR. Are you connected??
        print(f"{'>'*90}  TX --> 02 FF F0 01 F8 03")

        #Waiting 500ms to gather any respones from the ASR650
        time.sleep(0.5) #wait for 500ms

        try:
            #Receive the connection request message that comes from the reader
            recv=connection.recv(32) #blocking call

            if not recv: #client has disconnected
                msg="[THREADED CLIENT][NOT RECV] ASR650 RFID Reader has ended the connection"
                logger.warning(msg)
                self.cleanup_connection(address)    
                connection.close()
                self.prnt(msg)
                return
            else:
                '''
                Update: 7/29/2024
                NEW - send two additional commands to set trigger mode and antenna speed fastest
                '''
                
                connection.send(bytes.fromhex("02FFF028311002C203"))
                msg_triggermode = f"{'>'*90}  TX --> SET OPERATING MODE = TRIGGER:  02 FF F0 28 31 10 02 C2 03"
                print(msg_triggermode)

                if(self.varCheckBoxLogTxRx.get()):
                    logger.debug("[TX][ASR650] SET OPERATING MODE = TRIGGER:  02 FF F0 28 31 10 02 C2 03")

                #Give the ASR650 time to respond
                time.sleep(0.5) #wait 500ms to get a response
                recv=connection.recv(32) #blocking call  <== THIS IS THE RESPONSE FOR SET OPERATING MODE
                            
                if not recv: #client has disconnected
                    msg="[THREADED CLIENT][NOT RECV] ASR650 RFID Reader has ended the connection"
                    logger.warning(msg)
                    self.cleanup_connection(address)    
                    connection.close()
                    self.prnt(msg)
                    return
                else:
                    ######(FOR SOC Only - To display received messages) print(f"{'>'*90}  RX --> {self.readableRX(recv)}")
                    msg_rx_triggermode = f"{'>'*90}  RX --> {self.readableRX(recv)}"

                    if(self.varCheckBoxLogTxRx.get()):
                        logger.debug(f"[RX][ASR650] {self.readableRX(recv)} ")


                    #Then send SET MUX SPEED = 1  (fastest)   command
                    connection.send(bytes.fromhex("02FFF0C3017603"))
                    msg_muxspeed = f"{'>'*90}  TX --> SET MUX SPEED (1 FAST): 02 FF F0 C3 01 76 03"
                    print(msg_muxspeed)

                    if(self.varCheckBoxLogTxRx.get()):
                        logger.debug("[TX][ASR650] SET MUX SPEED (1 FAST): 02 FF F0 C3 01 76 03")

                    time.sleep(0.5) #wait 500ms to get a response
                    recv=connection.recv(32) #blocking call  <== THIS IS THE RESPONSE FOR SET OPERATING MODE
                                
                    if not recv: #client has disconnected
                        msg="[THREADED CLIENT][NOT RECV] ASR650 RFID Reader has ended the connection"
                        logger.warning(msg)
                        self.cleanup_connection(address)    
                        connection.close()
                        self.prnt(msg)
                        return
                    else:
                        #### (FOR SOC Only - To display received messages) print(f"{'>'*90}  RX --> {self.readableRX(recv)}")

                        if(self.varCheckBoxLogTxRx.get()):
                            logger.debug(f"[RX][ASR650] {self.readableRX(recv)} ")

        except Exception as e:
            msg="[THREADED CLIENT] [EXCEPTION while RECV] " + str(e)
            logger.warning(msg)
            self.cleanup_connection(address)    
            connection.close()
            self.prnt(msg)
            return 

        #If the above worked and no exception, the thread is ready to send/receive rfid trigger requests
        #Get the type of device from the IP address - if not an expected IP, return immediately. Only certain ip's are allowed.                 
        (type,ip,port,pen,reader)=self.infoIP(address)  #this is the real/simulated asr responding to multi-id trigger request
        tx_rx = self.RxTx[address[0]] #current tx, rx count
        tx_rx[1] += len(recv)  #update count to increment Tx

        if(type=="unknown"):
            self.prnt("Unknown ip "+address[0]+". Closing connection")
            connection.close()
            #Always return
            return


        #Update MoreInfo
        curList = self.threadsMoreInfo[(address[0],address[1])] #can this lead to key error
        curList[1]=connection #this was causing error sometimes  
        curList[2]=type


        #update the status here, tables are updated within this function 
        self.updateStatus(address)

        #Form the trigger request message - sent after predefined timeout in the json file
        hex_timeout = format(self.timeoutTrigger_ms.get(),'x')  #convert 500 to hex 1f4
                    
        if(len(hex_timeout)%2 == 1):
            hex_timeout = "0" + hex_timeout  #pad 0 to make it 01f4 instead of 1f4
        
        tokens= [hex_timeout[i:i+2] for i in range(0,len(hex_timeout),2)]  #group as 2 letters
        
        #if token is either 02 03 or 10 add a dle 10 before them
        new_tokens = []
        for token in tokens:
            if token=="02" or token=="03" or token=="10":
                new_tokens.append("10")

            new_tokens.append(token)

        #TRIGGER Mode - send multi id request
        #Example:   02 FF F0 21 10 03 E8 00 22 1F F1 03
        #           02 FF F0 21(trigger) (10 03 E8)=>1000ms  (00 22)=>reserved  1F(antenna mask) CRC ETX

        #Form an int array of messages so that we can call bytes on them to generate CRC
        msg_trigger_request=["02","FF","F0","21"]
        msg_trigger_request.extend(new_tokens)
        msg_trigger_request.extend(["00","22","0F"])  #00 22 FF => FF means all 8 antennas, 1F means 5 antennas, MSU = 0F only 4 antennas

        #Generate crc
        msg_trigger_request_int = [int(t,16) for t in msg_trigger_request]  #need to make list of ints first (or call  byte.fromhex(hex_string))
        crc =  crc_poly(bytes(msg_trigger_request_int), 8, 0x1D, 0, ref_in=True, ref_out=True)
        #Append crc (convert from int to hex)
        crc_hex = format(crc,'x').upper()
        msg_trigger_request.append(crc_hex)
        msg_trigger_request.append("03")
        #Convert the hex message to int
        msg_trigger_request_int_final = [int(t,16) for t in msg_trigger_request]
       
        
       
        #Once connection is established, this while loop runs forever.
        while True:
            
            try:
                #if comm is paused, just continue, to next iteration while loop
                if(self.pauseComm):
                    time.sleep(2)
                    continue    

                #connection.send("test".encode("ascii"))
                if(type=="asr" or type=="asr_localhost"):

                    startTime = time.time() #track time taken
                    
                    #connection.send(bytes.fromhex("01F400221F6303")) #Trigger mode - multi id request - 500ms timeout
                    connection.sendall(bytes(msg_trigger_request_int_final))
                    self.printToTerminal(f"{'>'* 90} TX --> {' '.join(msg_trigger_request)}")
                    tx_rx = self.RxTx[address[0]] #current tx, rx count
                    tx_rx[0] += len(msg_trigger_request_int_final)  #update count to increment Tx

                    #wait 250 more milliseconds than timeout to let the ASR650 respond
                    time.sleep(self.timeoutTrigger_ms.get()/1000.0 + 0.25)
                    #asr would have responded in weird chunks. If we wait long enough - we will receive the full data

                    #blocking call
                    data = connection.recv(512)  #This is going to run multiple times if the data
                    #comes in broken segments such as 29 byte read coming
                    #in as 1-24-5

                    lenNewData = len(data)
                    lenOldData = len(data_buffer)

                    #data_buffer.append([d for d in data])
                    #Bytes to Int and add the the data_buffer. This will be emptied by calls to parse()
                    data_buffer.extend([d for d in data])
                    
                    #How many new bytes were received (may be small numbers and not the entire message all at once)
                    byteReceived = byteReceived + len(data)
                    
                    #Instead of converting to string to display - just pass on the raw bytes to PC
                    #reply = 'Server Says: ' + data.decode('utf-8')
                    
                    if not data: #client has disconnected
                        break
                    else:
                        tx_rx = self.RxTx[address[0]] #current tx, rx count
                        tx_rx[1] += len(data)  #update count to increment Tx
                        
                        #There is data, collect and then parse
                        if (self.debug_threadedClient):
                            print(f"Data Len={len(data)}")
                        
                        #parse messages that are fully formed (could be ACK, NACK, EID etc)
                        #result = self.parse_data(data_buffer)
                        result = self.parse_data_V1(data_buffer)
                        
                        reads=[]  #reads will be saved here

                        goodToCollectReads = False #This variable is set to true once we receive an ack for 0x21

                        for indx,chunk in enumerate(result["chunks"]):
                            
                            #Get the message between the indices
                            #msg = data_buffer[chunk[0]:chunk[1]+1]
                            msg = chunk  #in parse_data_v1, it returns actual chunks

                            #It is possible that someleft over empty messages were not parsed by parse_data_V1.
                            #in that case chunk will be returned as empty and we can skip
                            if len(msg) == 0:
                                continue

                            if(self.debug_displayProcessedMessage):
                                print(f".......Processing msg #{indx} of {len(result['chunks'])}")
                                print(f"msg ====> {msg}")

                            #Parse it figure out what type of message
                            msg_info = self.parse_message(msg) #has keys type,valid, 

                            if( (msg_info["valid"]) and (msg_info["type"]=="ACK")):
                                if(msg_info["code"]!='0x21'):
                                    print(f"ACK..Expected for code 0x21, received {msg_info}. Figure out how to handle it later")
                                    
                                else:
                                    goodToCollectReads = True 
                                
                                #whether ack for 01 or 21, move on to next segment
                                continue

                            elif((msg_info["valid"]) and (msg_info["type"]=="NACK")):

                                if(msg_info["code"]!='0x21'):
                                    print(f"NACK...Expected code 0x21, received {msg_info}. Figure out how to handle it later")
                                    continue # not sure what the ACK is for but we dont need to know until its 0x21
                                else:
                                    #since we received a nack for multi id request, may be we dont have trigger mode enabled.
                                    print("NACK.  Trigger mode does not seems to be enabled.") 
                                    break   #Are we sure we dont want to continue and rather break???????
                            elif((msg_info["valid"]) and (msg_info["type"]=="EID")):

                                #If we have the ACK, treat the rest as reads        
                                if(goodToCollectReads):
                                    
                                    if(msg_info["valid"]) and (msg_info["type"]=="EID"):           
                                            
                                        eid = msg_info["eid"]
                                        antenna=msg_info["antenna"]

                                        #appends to reads    
                                        reads.append([msg_info["eid"],msg_info["antenna"],tuple(msg)])

                        #Did we get any reads
                        if(len(reads)>0):

                            #Keep just he unique reads
                            unique_reads = [list(x) for x in set(tuple(x) for x in reads)]

                            #sort it based on antenna number which is the second element in the [eid,antenna] list
                            sorted_unique_reads = sorted(unique_reads,key=lambda x:x[1])
                            #print it
                            if(not self.varCheckboxQuiet.get()):
                                [print(read,end="") for read in sorted_unique_reads]

                            #One more check: if we get the same reads within the second, ignore them
                            #not expecting two sets of reads in  within the second in the trigger mode but just to be sage.

                            #is this eid, antenna and time already in the list of the last 20 inserts
                            now = dt.now()
                            now = now.replace(microsecond=0) #get rid of .fff

                            filtered_reads = []

                            for read in sorted_unique_reads:

                                eid = read[0]
                                antenna=read[1]
                                fullMessage=read[2]

                                if(self.isEIDUnique(lastNAntennaReads,eid,antenna,now)):
                                    
                                    filtered_reads.append([eid,antenna,fullMessage])

                                    lastNAntennaReads.append((eid,antenna,now)) #add as a unique
                                    
                                    #just keep the list of last 20 reads
                                    if(len(lastNAntennaReads)==21):
                                        lastNAntennaReads = lastNAntennaReads[1:]
                                        self.printToTerminal("Keeping last 20 reads to avoid duplicate")
                                else:
                                    self.printToTerminal(f"DUPlICATE {pen}-{reader}-{eid}-{antenna}-{now}--->Not adding to queue")
                            
                            #09/03/2022 --> After getting unique and sorted reads, do one last thing
                            #if the same rfid shows up in multiple antennas, keep the left most one
                            final_reads = []
                            rfids = [read[0] for read in filtered_reads]
                            uniq_rfids = set(rfids)
                            #10/29/2024 - reads to publish
                            published_reads = []

                            for rfid in uniq_rfids:
                                matches = [read for read in filtered_reads if read[0]==rfid]
                                final_reads.append(matches[0])

                                #For publishing
                                pub_dict = {"rfid":matches[0][0], "antennaNo": matches[0][1]}
                                published_reads.append(pub_dict)  #0th index is rfid, 1st index is antenna

                            
                            #Once the final reads are there - send to Camera Capture + Send to DB queue
                            if(len(final_reads) > 0):
                                identifier="P"+str(pen)+"-R"+reader+"-"+dt.strftime(now,"%Y%m%d%H%M%S")

                                '''
                                #
                                #
                                #
                                #
                                    BROADCAST HERE IF YOU NEED ANOTHER CLIENT/THREAD TO HAVE ACCESS TO THE READS
                                #
                                #
                                #
                                #
                                '''

                                '''
                                    10/29/2024 - Broadcasting/Publishing(pubsub module) using topic named "topic_rfid"
                                                 Publish TOPIC here

                                '''
                             
                                pub.sendMessage("topic_rfid",data={"time_of_read": dt.strftime(dt.now(),"%Y-%m-%d %H:%M:%S"), "reads": published_reads})

                                #
                                #
                                #
                                #
                                #
                                #
       



                                #Sending to queue keep identifier
                                self.sendToQueue(now,pen,reader,final_reads,identifier)
                              
                        #Update the data_buffer = note result["buffer"] remove the valid chunks and returns the rest
                        data_buffer = result["buffer"]


                #if it's an asr, compute the time taken to process the data
                if(type=="asr" or type=="asr_localhost"):
                    endTime = time.time()
                    timeDiff = endTime - startTime #going to be in seconds

                    msg_timeTaken = f"Time taken: {timeDiff:.2f}"
                    self.printToTerminal(msg_timeTaken+"\n\n")

                    #self.statusbar.config(text=msg_timeTaken)
                    
                    waitTime = self.triggerInterval_ms.get()/1000.0 #seconds

                    if(timeDiff < waitTime):
                        time.sleep(waitTime - timeDiff)

            except Exception as e:
                print(e)
                hdr=f"{'*'*20}\nClient:({address[0]},{address[1]})\n\n"
                self.log(hdr)
                self.log(str(e))
                traceback.print_exc()
                break
                        

        #If the while loop was broken, it was probably because client got disconnected.
        #Do some cleanup and update any conneciton information before closing the connection
        self.cleanup_connection(address)    
        connection.close()
    
    def log(self,msg):
    
        with open("log_exceptions.txt","a") as fp:
            fp.write(msg)
  
    #This sends the reads to the database queue where another thread is bulk inserting N records every 2 seconds
    def sendToQueue(self, now,pen,reader,final_reads,identifier):

        msg = ""
        for read in final_reads:

            eid = read[0]
            antenna=read[1]
            self.printToTerminal(f"Adding unique read  {identifier}---{pen}-{reader}-{eid}-{antenna}-{now} to queue") 
            #self.update_queue(pen,reader,eid,antenna,now) #update the deque
            self.update_queue(pen,reader,eid,antenna,now,identifier) #update the deque
            
            msg = msg+f"RECEIVED - {pen}-{reader}-|| {eid}  @ {antenna}||  @ {now}\n"
            #self.prnt(msg)

            #Also update the antenna read count
            #Current antanna value -
            cur_hits = self.antennaHits[reader]
            #Antenna Number
            antennaNo = int(antenna)  #read to ascii to int (will be 0 if single antenna is used)
            cur_hits[antennaNo+1] +=1  #if antenna is 1, update the 3rd value in the list. 1st val=pen No, 2nd val = Ant #0, 3rd Val=Ante #1 ......
            #??????? what is this self.antennaHits[antennaNo]=cur_hits
            #create a list to send to treeview
            lst = []
            lst.append(reader)
            lst.extend(cur_hits)

            #which index is this reader
            indx=self.indexReader(reader)
            self.treeAntenna.item(indx,text='',values=tuple(lst))
            self.treeAntennaMAIN.item(indx,text='',values=tuple(lst[0:-4])) #MSU exclude the last 4 items
        #Print the reads to the messagebox
        self.prnt(msg)
                         
        pass

  

       
    
    def generate_custom_identifier_message(self,identifier):
        
        custom = b'\x02\xCC\xAA\xDD'+bytes(identifier,"ascii")
        crc = crc_poly(custom, 8, 0x1D, 0, ref_in=True, ref_out=True) #crc returned is INT
        
     
        #INT to hex and then upper case 
        crc_hex = hex(crc).upper()  #hex string has 0x padded - could return single character like 0xE or 0x3 
        crc_hex=crc_hex[2:] #get rid of 0x
        #Handle single character
        if(len(crc_hex)==1):
            crc_hex = "0"+crc_hex  #prepend 0 for single character crc's 
            
        #Prepend DLE is either 02 or 03 or 10
        if (crc==2) or (crc==3) or (crc==16):  
            crc_hex = "10"+crc_hex 
           
        #print(f"\n\n\ncrc_hex={crc_hex}\n\n")
        
        custom_final = custom + bytes.fromhex(crc_hex) + b'\x03'
        
        return custom_final
    
        pass 
    
    def handleLocalhosts(self,connection, address):
        
        fakeInfo = self.fakePens[self.fakePenIndex]

        local = ("asr_localhost",address[0],address[1], fakeInfo[0],fakeInfo[1]) #("ars",ip,port,pen,reader)

        self.localhostCounts[fakeInfo[1]]=0  #save reader number and initial count of 0 reads

        self.fakePenIndex = self.fakePenIndex + 1  #next index where localhost is saved

        self.LocalhostClients.append(local)

        return local
        

    def isEIDUnique(self,lastEntries,eid,antenna,ts):

        debug = False

        if(debug):
            print(f"\n======Checking if unique or already in the list (current list size={len(lastEntries)})=========")
            [print(entry) for entry in lastEntries]
        
            if(len(lastEntries) == 0):
                print("The list is empty, must be unique")

        
        for entry in lastEntries:
        
            if( eid==entry[0] and antenna==entry[1] and ts==entry[2] ):
                self.printToTerminal(".......Not unique")
                return False 
            

        #if none of the entries matched it, then must be unique
        self.printToTerminal(".......YES UNIQUE")
        return True     

    def findIndex(self,lst,item):
        try:
            return lst.index(item)
        except ValueError:
            return -1

    def findETX(self,data):

            indxETX=-1
            #data = [d for d in data_orig] #this makes a list of int's now

            while(1):
                #indxETX = remaining.find(b'\x03')
                indxETX = self.findIndex(data,3)

                if(indxETX !=-1):
                    if data[indxETX-1] == 16:  #This is DLE and 03 is not an ETX
                        data[indxETX]=255 #make the 03 255 so its omitted
                        #nextIndex = startIndex+indxETX + 1
                        indxETX = self.findETX(data)
                    else:
                        break
                else:
                    break #break because there is no \x03

            return indxETX

    def findSTX(self,data):

            indxSTX=-1
            #data is now expected to be an int array
            #data = [d for d in data_orig] #this makes a list of int's now

            while(1):
                #indxETX = remaining.find(b'\x03')
                indxSTX = self.findIndex(data,2)
                #if the 1st index has 02, we found it, just return
                if(indxSTX == 0):
                    break

                if(indxSTX !=-1):
                    if data[indxSTX-1] == 16:  #This is DLE and 02 is not an STX
                        data[indxSTX]=255 #make the 02 255 so its omitted
                        #nextIndex = startIndex+indxETX + 1
                        indxSTX = self.findSTX(data)
                    else:
                        break
                else:
                    break #break because there is no \x03

            return indxSTX                      


    def parse_data_V1(self,data_buffer):
        
        '''
        The V0 parse data would return 
        result.buffer = [', . , . remaining data, .....]
        result.chunks [ [0,7] ,[0,28] , [0,28] ,[] , [] ]

        where the chunk indices were based on 0 and was confusing.

        In V1, return the actual chunk
        '''
        #print("Loop Parse Data")
        results = {}

        results['buffer']=data_buffer  #return remaining data buffer
        results['chunks']=[]           #return actual chunks of data

        while 1:
            #print("Looop infinite 1, within parse data")
            iSt = self.findSTX(data_buffer)

            if(iSt !=-1):
                iEnd = self.findETX(data_buffer)

                if(iEnd != -1):
                    
                    if(self.debug_displayParsedData):
                        print(f"Found chunk from {iSt} to {iEnd}")

                    thisChunk = data_buffer[iSt:iEnd+1]

                    if(self.debug_displayParsedData):
                        print(thisChunk)

                    #convert from int to hex list    
                    hx =[ hex(d) for d in thisChunk]

                    if(self.debug_displayParsedData):
                        print(hx)
                        print()

                    chunk = [iSt,iEnd]

                    #results["chunks"].append(chunk)
                    results["chunks"].append(thisChunk)
                    

                    #Move the data buffer ahead if full mesage is found
                    data_buffer = data_buffer[iEnd+1:] #Remove the m essage from data buffer
                    
                else:
                    break
            else:
                break

        results["buffer"] = data_buffer     

        return results


    def crcCheck(self,msg,msg_info):

        expected_crc = msg[-2]
        bytes_for_crc = bytes(msg[0:-2])
        crc = crc_poly(bytes_for_crc, 8, 0x1D, 0, ref_in=True, ref_out=True)
        if(crc == expected_crc):
            msg_info["valid"]=1
        else:
            msg_info["valid"]=0
            msg_info["other"]=f"crc mismatch, expected={expected_crc}, calculated={crc}"

    #parse_messages takes the message block 0x02 DST SRC ......  CRC ETX and returns expected messages or discards the pile
    def parse_message(self,msg):

        msg_info = {}
        msg_info["valid"]=0
        msg_info["type"]="unknown"
        msg_info["other"]="n/a"
    
        #if msg starts with   0x02 0xF0 0x01 - must be from ASR650
        if msg[0]==2 and msg[1]==240 and msg[2]==1:

            if(msg[3] == 6):  #i.e. 0x06
               
                msg_info["type"]="ACK"
                msg_info["code"]="0x"+format(msg[4],"02x").upper()  #C2 from 192
                self.crcCheck(msg,msg_info)
               

            elif msg[3]==21:  #i.e 0x15
                #msg_info["valid"]=1
                msg_info["type"]="NACK"

                msg_info["code"]="0x"+format(msg[4],"02x").upper()  #0xC2 from 192
                self.crcCheck(msg,msg_info)

            elif msg[3]==35:  #i.e 0x23 (EID)
                msg_info["valid"]=1
                msg_info["type"]="EID"
                (eid,antenna)=self.extract_eid(msg)
                msg_info["eid"]=eid 
                msg_info["antenna"]=antenna
        
  
        #Return message info (could be unknown by default)        
        return msg_info



    def extract_trigger(self,msg):

        chunk =  msg[4:-3]  #02 FA 01 A1  timeout  DLE  delay  ....  (Get main chunk up until right before the last DLE)

        timeout = -1
        delay = -1

        #is there a dle 10, it separates the timeout and delay
        indx = self.findIndex(chunk, 16)  #0x10 = 16 Decimal
        if indx==-1:

            return(-1,-1)
        else:
            timeoutArr = chunk[0:indx]
            delayArr = chunk[indx+1:]

            #convert to char list
            timeout = self.charToNum(timeoutArr)
            delay = self.charToNum(delayArr)

            return (timeout,delay)  


    def change_font_size(app,frames,direction):
    
        #for frame in frames:
        for frame in frames.winfo_children():

            num = 0
            
            for child in frame.winfo_children():
                num = num + 1 

                print("I am child #" + str(num))
                
                font = child.cget('font')
                #size = font.cget('size')
                #size = child.cget('size')
                #print("Font size = " + str(size))

                print("Font = " + font)
                tokens = font.split(" ")
                print(tokens)

                family = tokens[0]
                size = int(tokens[1])

                if(direction == "big"):
                    fontNum = size+1
                else: 
                    fontNum = size-1
                    
                
                child.config(font=(family,fontNum))

                
            print("\n\n")

    #Convert int array like [49 48 49 48] to ascii based number 1010 
    def charToNum(self,arr):

        arrC = [chr(x) for x in arr]  #char array from array of ints 
        arrC_join = ''.join(arrC)  #join char array to form a string
        return int(arrC_join)  #convert string to number


        
        
    def extract_eid(self,chunk):
        rfidList =  chunk[5:21]
        rfidChr = [chr(n) for n in rfidList]
        rfidStr = ''.join(rfidChr)
        antenna = chr(chunk[23])
        return (rfidStr,antenna) 



    def cleanup_connection(self,address):
        ip_port = (address[0],address[1])

        if(ip_port in self.cleanupQueue):
            self.prnt(f"Cleanup of {ip_port} already in progress")
            return 

        #if not in cleanup queue already, add it.
        self.cleanupQueue.append(ip_port)

        #self.prnt("*"*40)
        msg1=f"Cleaning Connection {address[0]} @ {address[1]}"
        self.prnt(msg1)
        logger.debug("Attempting to cleanup " + msg1)
        #self.prnt("*"*40)

        #Remove it from self.threads
        ip_port=((address[0],address[1]))

        try:
            self.threads.remove(ip_port)
        except:
            logger.error(f"removing item from thread list failed. Item = ({ip_port[0]},{ip_port[1]})")

        try:    
            self.threadsMoreInfo.pop(ip_port)
        except:
            logger.error(f"threads.pop() failed. Item = ({ip_port[0]},{ip_port[1]})")

        

        #if asr disconnects, just update the connection status pair
        #if camera disconnects, if it's socket was assigned to an asr, need to reset it to 0

        for indx,pair in enumerate(self.asr_camera_pairs):
            
            asr = pair[0]
            camera=pair[1]

            if asr==address[0]:
                self.asr_camera_status[indx][0]=0
                break
            
            if camera==address[0]:
                self.asr_camera_status[indx][1]=0
                #find the asr ip and clear its sockets
                affected = [asr1 for asr1,camera1,pen1,reader1 in self.asr_camera_pairs if camera1==camera]
                affected_ip_asr = affected[0]
                self.sockets[affected_ip_asr]=(0,0)  #(connection_to_pc=0, pen = 0)
                break

           
        for indx,pair in enumerate(self.standalone_readers):
                
            asr = pair[0]
            camera=pair[1]

            if asr==address[0]:
                self.asr_standalone_status[indx]=0
                break
                
       

        #Remove from cleanup queue
        self.cleanupQueue.remove(ip_port)

        #Update the tables            
        self.updateTable()
        #Update active thread count
        self.LabelThreadsActive.config(text=str(len(self.threads)))
        pass     


    #extract reads from data chunk (possibly same antenna read multiple times)
    def extract_reads(self,chunk):

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
        db_uniq = list(set(db_data))

        for d in db_data:
            print(d)

        print("v"*20)

        for d in db_uniq:
            print(d)
        
        print("\n\n" + "-"*20)

        return db_uniq       

    #This function creates a new thread and dumps reads out of the queue
    #to the the database at fixed intervals (currently 2.0 seconds)

    def db_write(self):

        print("Thread for writing to db (Feeder + CaptureStatus) has started.")

        while(1):

            #Wait the previous writing operation to finish
            if(not self.writingToDBInProgress):

                try:

                    #Get todays count - just to keep the connection alive
                    #15 means 30 seconds
                    #30 means every minute
                    #150 means 300 seconds i.e. 5 minutes
                    #300 means 600 seconds same as 10 minutes
                    if self.numDBWrites%150 == 0:
                        
                        #self.labelCntValue.config(bg='yelllow')
                        #self.update()
                        
                        if(mdbh.db):
                            self.cntToday=mdbh.getTodaysRecordCount() #Get the count
                        else:
                            self.cntToday+=self.numRecordsInsertedSinceLastRead 

                        numDBWrites = 0   #Reset this counter to 0
                        #numRecordsSinceLastRead = 0 
                        self.numRecordsInsertedSinceLastRead = 0
                        #self.labelCntValue.config(bg='#9ACD32')
                        #self.update()

                    #self.LabelTodaysCount.config(text="Todays Count: "+str(cntToday))
                    ####10/20==> self.labelCntValue.config(text=str(cntToday))
                    self.labelCntValue.config(text=str(self.cntToday + self.numRecordsInsertedSinceLastRead))

                    #The deque size
                    self.LabelDequeSize.config(text=str(self.dbqueue.qsize())) #get the size of the deque

                    # ---------- FEEDER DATA ------------------
                    #if deque is not empty 
                    if(not self.dbqueue.empty()):

                        len_queue = self.dbqueue.qsize() #queue size
                        num_items_to_write = min(len_queue,1000) #write no more 1000 records at a time
                        

                        #Records to insert
                        records=[]
                        for k in range(0,num_items_to_write):
                            records.append(self.dbqueue.get())
                        
                        #block the db write
                        self.writingToDBInProgress=True

                        if(mdbh.db):  #if there is a database to write to will be NONE

                            try:
                                cnt = mdbh.insertDataReal(records) #
                                self.writingToDBInProgress=False   #

                                #update count since last SELECTED
                                self.numRecordsInsertedSinceLastRead = self.numRecordsInsertedSinceLastRead + cnt
                                #####3/23/2023 - NO NEED TO BUMP IT UP HERE ===>>> self.numDBWrites = self.numDBWrites + 1


                                self.printToTerminal("Done writing")
                                self.printToTerminal("-"*20)

                                
                            except:
                                logger.error(f"Exception while inserting FEEDER_DATA records. May be db is not connected.")
                                self.numRecordsInsertedSinceLastRead += self.dump_to_backup_data_file(records)
                                self.writingToDBInProgress=False

                        else:
                            self.statusbar.config(text=f"Getting reads but cannot save to database. Writing to text file instead. Current queue size={len_queue}",fg='red')
                            self.dump_to_backup_data_file(records)
                            self.writingToDBInProgress=False

                    #BUMP number of db write attempts here instead 3/23/2013
                    self.numDBWrites += 1 #even if nothing is written we want it incremented, so we can call the databse and keep the connection alive every N attempts
                    #if in the quiet terminal mode, print some status on the same line, so we know its working
                    if(self.varCheckboxQuiet.get()):
                        print("", end=f"\r[Quiet Mode Active]: Database Records Inserted Today: {self.labelCntValue.cget('text')}")
                    #print(self.numDBWrites)
                    
             
                    #----------- DELETE Queries -------------------
                    #if deque is not empty 
                    
                    if(not self.dbqueueDeleteRecords.empty()):

                        len_queue_DEL = self.dbqueueDeleteRecords.qsize() #queue size
                        num_del_query_to_run = 1

                        #Records to insert
                        (type,sql) = self.dbqueueDeleteRecords.get()
                        
                        #block the db write
                        self.writingToDBInProgress=True

                        if(mdbh.db):  #if there is a database to write to will be NONE

                            try:
                                mdbh.cursor.execute(sql)
                                mdbh.db.commit()
                                cntDeleted = mdbh.cursor.rowcount

                                self.writingToDBInProgress=False 

                                if(type=="DELETE_TODAY"):
                                    msg = "Deleted {} records from feeder_data_new that belonged to today".format(cntDeleted)
                                else:
                                    msg = "Deleted ALL {} records from feeder_data_new".format(cntDeleted)
                                
                                #8/29/2024 - make self.cnt_today = 0 (reset)
                                self.cntToday = 0

                                showinfo(title="Database", message=msg)

                            except:
                                logger.error(f"Exception while calling DELETE query. May be db is not connected.")
                                self.writingToDBInProgress=False

                        else:
                            #self.statusbar.config(text=f"Getting reads but cannot save to database. Writing to text file instead. Current queue size={len_queue}",fg='red')
                            #self.dump_to_backup_data_file(records)
                            self.writingToDBInProgress=False

                    

                    
                except Exception as e:

                    print("Error in db_write thread")
                    logger.error(f"[DB_WRITE]. {str(e)}")
                    self.writingToDBInProgress=False


                finally:
                    Event().wait(2.0)
                    #time.sleep(10.0)

    
    def dump_to_backup_data_file(self,records):

        with open("communications-backup-data.txt","a") as fp:
            for record in records:
                #make comma separeted list
                record_str_list = [str(record[0]), record[1], record[2],record[3],dt.strftime(record[4],"%Y-%m-%d %H:%M:%S"),record[5]]
                record_csl = ",".join(record_str_list)
                #add new line
                fp.write(record_csl+"\n") 

            #Return number of lines
            return len(records)    


    
    def update_queue(self,pen,reader,rfid,antenna,ts,identifier):
        
        data = (pen,reader,antenna,rfid,ts,identifier)

        if(not self.dbqueue.full()):
            self.dbqueue.put(data)
        else:
            self.statusbar.config(text=f"Max queue size of {self.dbqueue.qsize()} reached. Not adding any new reads")
            logger.debug("Queue size limit reached")

        #Print
        if(self.debug_updateQueue):
            print("\n*******In update_queue()************")
            print("Pen #{}.  Added to queue...Current queue size: {}".format(pen,self.dbqueue.qsize()))
        
        self.printToTerminal(data)

    def update_queue_captureStatus(self,data):
        
        if(not self.dbqueueCaptureStatus.full()):
            self.dbqueueCaptureStatus.put(data)
        else:
            self.statusbar.config(text=f"Max queue size of {self.dbqueueCaptureStatus.qsize()} reached. Not adding any new reads")
            logger.debug("Queue size limit reached")
 
        print(data)


def exit_program():
    logger.warning(f"[WINDOW DELETED] Program Exit")
    root.destroy()

if __name__ == "__main__":

    logger.debug("[PROGRAM-START]")

    root = Tk()
    root.protocol("WM_DELETE_WINDOW", exit_program)
    root.title("Swine Managment System - Communications Hub")
    #root.iconbitmap(r"pig.ico")
    root.geometry(f"{MAIN_GUI_WIDTH}x{MAIN_GUI_HEIGHT}+0+0")

    style = Style()
    style.configure("Treeview", font=(None,TREEVIEW_CONTENT_SIZE), rowheight=25)
    style.configure("Treeview.Heading", font=(None,TREEVIEW_HEADING_SIZE))
   
    '''
    print("reader app root-->")
    print(root)
    print("rfid readers's toplevel window (main root which is calling this)")
    print(root.winfo_toplevel())

    print(root==root.winfo_toplevel())
    print("Printing child of root's winfo_children")
    for child in root.winfo_children():
        print(child)
        #print_children(child)

    sys.exit("Done RFID App")
    '''
  
    #Create the main object
    obj = RFIDReaderApp(root)

    try:
        print("*"*40)
        print(f"VERSION {VERSION}")
        print("*"*40)
        #Start main loop
        root.mainloop()
    except Exception as e:
        logger.warning(f"[MAINLOOP EXITNG] {str(e)}")