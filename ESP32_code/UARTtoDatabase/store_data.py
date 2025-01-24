import sqlite3
import serial
import re

failed_transmission = 0 #counting how many data strings come in incorrectly
buffer = "" # Buffer to hold incomplete data

# Configure serial port
SERIAL_PORT = '/dev/tty.usbserial-0001'  # Replace with your port
BAUD_RATE = 115200 # baud rate for UART

# Connect to SQLite database
conn = sqlite3.connect('database.db')
cursor = conn.cursor()

# Create table named "sensor_data", first column is timestamp, and the rest are REAL data types
cursor.execute('''
    CREATE TABLE IF NOT EXISTS sensor_data ( 
        timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
        temperature REAL,
        humidity REAL,
        nh3 REAL,
        h2s REAL,
        co2 REAL,
        ch4 REAL
    )
''')
conn.commit()

# Open serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

# define a string that mimics the sensor output format.
data_pattern = re.compile(r"Received data: Temp:(\d+\.\d+),Humidity:(\d+\.\d+),NH3:(\d+\.\d+),H2S:(\d+\.\d+),CO2:(\d+\.\d+),CH4:(\d+\.\d+)")

while True:
    try:
        # Read data from the serial port
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"Received: {line}")

        # Skip lines that do not have received data:
        if "Received data: " not in line:
            continue

        # Append new line data to the buffer
        buffer += line

        # Check if the buffer contains a full line of sensor data
        match = data_pattern.search(buffer)
        if match:
            temperature = float(match.group(1))
            humidity = float(match.group(2))
            nh3 = float(match.group(3))
            h2s = float(match.group(4))
            co2 = float(match.group(5))
            ch4 = float(match.group(6))

            # Insert data into the database
            cursor.execute('''
                INSERT INTO sensor_data (temperature, humidity, nh3, h2s, co2, ch4)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', (temperature, humidity, nh3, h2s, co2, ch4))
            conn.commit()

            print(f"Stored data - Temp: {temperature}, Humidity: {humidity}, NH3: {nh3}, H2S: {h2s}, CO2: {co2}, CH4: {ch4}")

            # Clear the buffer since data has been processed
            buffer = ""
        else:
            # If the buffer does not contain a full match, check for malformed data
            # If a transmission is failed, increment the counter
            if len(buffer) > 100: # threshold to detect invalid data length
                failed_transmission += 1
                print(f"Failed Transmissions: {failed_transmission}")
                buffer = "" # Clear the buffer and wait for the next valid data

    except Exception as e:
        print(f"An error occurred: {e}")