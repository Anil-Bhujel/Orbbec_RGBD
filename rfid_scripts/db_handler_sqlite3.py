import sqlite3
from datetime import datetime as dt
import logging 

logging.basicConfig(level=logging.DEBUG,filename="log-communications.log",
        format='%(asctime)s:%(levelname)s:%(message)s')

logger = logging.getLogger(__name__)

#These variables are available to others importing this module
db=None
cursor=None
#databasename="sms1_usmarc"
databasename="rfid_reads.db"
table = "feeder_rfid_data"


def connectToDB():
   
    
    global db
    global cursor
    global status

    try:
        db = sqlite3.connect(databasename,check_same_thread=False)  #if not db_write results in error 
        #2024-07-11 13:27:45,797:ERROR:[DB_WRITE]. SQLite objects created in a thread can only be used in that same thread. The object was created in thread id 140704389951744 and this is thread id 123145530912768.
        #Cursor to the connection
        cursor = db.cursor()

        #If no exception, next check if table exists and if not create it
        try:
            cursor.execute(f"SELECT COUNT(*) as cnt FROM {table}")

        except sqlite3.OperationalError:
            
            print(f"Table {table} does not exist.")

            if(sqlite3.OperationalError): # if this error occurs
                try:
                    print(f"Attempting to create the table {table}")

                    cursor.execute(f'''
                    
                        CREATE TABLE {table}(
                       `id` INTEGER NOT NULL PRIMARY KEY,
                        `pen` tinyint NOT NULL,
                        `reader` varchar(5) NOT NULL,
                        `antenna` tinyint NOT NULL,
                        `rfid` varchar(20) NOT NULL,
                        `readtime` timestamp NOT NULL,
                        `identifier` varchar(30) DEFAULT '0'
                    
                    )''')
        
                    print("New table created successfully!!!")
                    status = 0
        
                except sqlite3.Error() as e:
                    print(e, " occured")
                    status=-1  #table creation failed

    except Exception as e:
        print("[SQLITE3]Error while calling connect")
        print(f"\nError Message={e}")
        logger.error(f"[SMS_DB_HANDLER_SQLITE3] Database Connection related. While calling .connect(), exception raised ={e}")
        status = -2
       

#connect to db
connectToDB()



#04/25/2023 - Faster way - scan the last 2 million records for todays date - we have gotten about 1million 350 thousand records in 1 day
def getTodaysRecordCount():

    today_date = dt.strftime(dt.now(),"%Y-%m-%d")
 
    sql = "SELECT COUNT(*) FROM (SELECT readtime from {} order by id DESC LIMIT 2000000) as derived "
    sql += "WHERE date(readtime)='{}'"
    sql = sql.format(table,today_date)
  
    cursor.execute(sql)
    resultsCount = cursor.fetchall()

    #select count(*) from (select readtime from feeder_data_new order by id DESC LIMIT 2000000) as derived WHERE date(readtime)="2023-04-25";   
    return resultsCount[0][0]


def insertDataReal(data):
    sql = "INSERT INTO {} (pen,reader,antenna,rfid,readtime,identifier) VALUES (?,?,?,?,?,?)".format(table)
    #print(data)   
    vals = data 
    cursor.executemany(sql,vals)
    db.commit()
    #conn.commit()
    return cursor.rowcount


