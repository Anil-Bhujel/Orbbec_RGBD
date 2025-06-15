import sqlite3
import shutil
import os
import datetime
import requests
import hmac
import hashlib
import time
import sys
import zipfile 

'''

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
This script makes an incremental backup of yesterday's data
and sends it over the UNL server.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

***** PLEASE CHANGE THE TWO VARIABLES *********

1. db_path  	(path to where the main database is located rfid_reads.db)
2. backup_dir   (folder where the daily backups will be made and uploaded to the server)

Note: the backup file will have a name "msu-backup_{today_date}_{current_time}.db.zip"
but the data it contains will be from yesterday. This script should be run sometime after midnight to send the data from the previous day.

Suggest: running a cronjob at say 00:05:00  (5 minutes after midnight)

'''


# Database and backup paths
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~PLEASE CHANGE THESE TWO VARIABLES TO APPROPRIATE PATH ~~~~~~~~~~~~~~~~~~~~~
db_path = "/home/pigs/ros2_ws/rfid_scripts/rfid_reads.db"
backup_dir = "/home/pigs/ros2_ws/rfid_scripts/backups"
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Server endpoint URL
upload_url = "https://unlamilab.com/msu-upload"
secret_key = 'PP0g9qYCwPZOKsQrTdqwwEIr2WoED9WC'  # Secret key that both Python script and server share

#Tables to be copied - no need to change this. The table where the data is saved is feeder_rfid_data
table_main = "feeder_rfid_data"  #which table to copy
table_copy = "feeder_rfid_data"      #which table to create and copy into

#make the backup dir
if(not os.path.exists(backup_dir)):
	os.makedirs(backup_dir)
	
#Setup todays and yesterdays date and backup filename
now = datetime.datetime.now()
today_date = now.strftime("%Y-%m-%d")

DAY_OFFSET = 1  #1 means yesterday, 2 means 2 days ago, 3 means 3 days ago
yesterday = now - datetime.timedelta(days=DAY_OFFSET)
yesterday_date = yesterday.strftime("%Y-%m-%d")
print(f"\nGoing to backup date={yesterday_date}\n")

current_time = now.strftime("%H-%M-%S")
backup_filename = f"msu-backup_{today_date}_{current_time}.db"
backup_file = os.path.join(backup_dir,backup_filename)

# Zip filename
zip_filename = backup_filename+".zip"
zip_path = os.path.join(backup_dir, zip_filename)

#Log the backup status to a table
def log_backup_status(status, message, timestamp, response=None):
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Insert log entry into the backup_log table
    cursor.execute("INSERT INTO backup_log (status, message, timestamp, response) VALUES (?, ?, ?, ?)", 
                   (status, message, timestamp, response))
    conn.commit()
    conn.close()

# Create the backup_log table if it does not exist
def create_log_table():
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS backup_log (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        status TEXT,
        message TEXT,
        timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
        response TEXT
    )""")
    conn.commit()
    conn.close()


# Create a backup of today's data
def backup_yesterday():
	record_count = None
	try:
		conn = sqlite3.connect(db_path)
		cursor = conn.cursor()

		# Count the records for yesterday (for logging)
		#sql=f"SELECT COUNT(*) FROM {table_main} WHERE date(readtime) = '{yesterday_date}'"
		#print(f"\nSQL Command Check Record Counts:\n....{sql}\n")
		cursor.execute(f"SELECT COUNT(*) FROM {table_main} WHERE date(readtime) = '{yesterday_date}'")
		record_count = cursor.fetchone()[0]
		print(f"Record Count = {record_count}")
		

		# Create a new temporary database to hold only today's data
		temp_db = os.path.join(backup_dir, f"temp_today_{today_date}.db")
		temp_conn = sqlite3.connect(temp_db)
		temp_cursor = temp_conn.cursor()

        
		# Copy data for today into the new temporary database
		cursor.execute(f"ATTACH DATABASE '{temp_db}' AS 'tempDB'") #tempDB needs to be in quote (only exists in memory)
		cursor.execute(f"CREATE TABLE tempDB.{table_copy} AS SELECT * FROM {table_main} WHERE date(readtime) = '{yesterday_date}'")
		temp_conn.commit()

		# Detach the temporary database
		cursor.execute("DETACH DATABASE 'tempDB'")
		
		# Backup the temp database
		shutil.copy(temp_db, backup_file)

		# Clean up
		temp_conn.close()
		os.remove(temp_db)
		conn.close()

	
		# Log success with timestamp
		backup_name_only = os.path.split(backup_file)[1] #just the name
		timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
		log_backup_status("Success", f"[{yesterday_date}] - [Backup-Created] - {backup_name_only},RecordCount={record_count}", timestamp, None)
		print(f"Backup created at {backup_file}")
        
	except Exception as e:
		# Log failure with timestamp and error message
		timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
		log_backup_status("Failure", f"[{yesterday_date}] [Backup-FAILED] Error: {str(e)}", timestamp, None)
		print(f"Backup failed: {str(e)}")

	#Return filename and record count
	return backup_file, record_count
	
# Function to generate HMAC signature
def generate_signature(secret_key, file_content, timestamp):
    # You can include the file content or any other data you want to protect
    message = file_content + str(timestamp).encode('utf-8')
    signature = hmac.new(secret_key.encode('utf-8'), message, hashlib.sha256).hexdigest()
    return signature

# Send file with signature to the server
def upload_file_with_signature(file_path, upload_url, secret_key,record_count):
	
	try:
		# Read the file content
		with open(file_path, 'rb') as f:
			file_content = f.read()

		# Get current timestamp for uniqueness and validity
		timestamp = int(time.time())

		# Generate HMAC signature
		signature = generate_signature(secret_key, file_content, timestamp)

		# Prepare data for POST request
		files = {'file': open(file_path, 'rb')}
		data = {
			'timestamp': timestamp,
			'signature': signature,
			'record_count':str(record_count)
		}

		# Send POST request to Laravel server
		response = requests.post(upload_url, files=files, data=data)
		
		# Close the file after sending the request
		files['file'].close()
		
        #Log success/failure
		timestamp_log = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
		if response.status_code == 200:
			print("File uploaded successfully!")
			print("Server Response:", response.text)
			log_backup_status("Success", "[SEND-To-Server-SUCCESS]", timestamp_log, response.text)
            
		else:
			print(f"Failed to upload file. Status code: {response.status_code}")
			print("Error:", response.text)
			log_backup_status("Failure", f"[SEND-To-Server-FAILURE] ServerResponse: {response.status_code}", timestamp_log, response.text)
	except Exception as e:
		# Log failure if there is an error in sending
		timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
		log_backup_status("Failure", f"Error in sending backup: {str(e)}", timestamp, None)
		print(f"Failed to send backup: {str(e)}")

# Function to zip the backup file
def zip_backup(backup_path, zip_path):
    try:
        with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
            zipf.write(backup_path, os.path.basename(backup_path))

        print(f"Backup zipped to {zip_path}")

		#Delete the original backup file if the zip operation was successful
        # Check if the zip file is valid
        with zipfile.ZipFile(zip_path, 'r') as zipf_check:
            # Try to extract the contents to ensure it's a valid zip file
            zipf_check.testzip()  # This checks the integrity of the zip file
            print(f"Zip file {zip_path} is valid.")

        # If the zip file is valid, delete the original backup
        os.remove(backup_path)
        print(f"Original backup file {backup_path} deleted successfully.")


    except Exception as e:
        print(f"Error zipping backup: {e}")
        log_backup_status("Failure", f"[ZIP-FAILURE]. Exception={str(e)}")



#
#

####      MAIN CALL      ####################
#
#
#

# Create log table if does not exist
create_log_table()

# Run the backup function
backup_file, record_count = backup_yesterday()

#Send it to server with the signature
# Only send the backup if it was successfully created
if backup_file and record_count is not None:

	#Make a zip
	zip_backup(backup_file,zip_path)
	#Upload the zip
	#upload_file_with_signature(backup_file, upload_url, secret_key,record_count)
	print(f"\n** SENDING TO SERVER ***")
	upload_file_with_signature(zip_path, upload_url, secret_key,record_count)
else:
	print(f"\n** SENDING TO SERVER ***")
	timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
	log_backup_status("Failure", f"[SEND][NOT INITIATED due to backup failure, either file missing or sql command did not run]",timestamp,None)
