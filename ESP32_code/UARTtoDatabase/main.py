import sqlite3
import serial  # For serial communication
import time

# Path to your database
DB_PATH = "./database.db"

# Initialize the database (if not already done)
def initialize_database():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS SensorData (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
        temperature REAL,
        humidity REAL,
        ammonia REAL,
        h2s REAL,
        co2 REAL,
        methane REAL
    )
    """)
    conn.commit()
    conn.close()
    print("Database initialized and table created!")

# Insert data into the database
def insert_sensor_data(temperature, humidity, ammonia, h2s, co2, methane):
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("""
    INSERT INTO SensorData (temperature, humidity, ammonia, h2s, co2, methane)
    VALUES (?, ?, ?, ?, ?, ?)
    """, (temperature, humidity, ammonia, h2s, co2, methane))
    conn.commit()
    conn.close()
    print("Data inserted into the database.")

# Read data from ESP32 via USB
def read_from_serial(port="/dev/tty.usbserial-0001", baud_rate=115200):
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to {port} at {baud_rate} baud rate.")

        while True:
            # Read a line of data from the ESP32
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(f"Received data: {line}")

                # Split the data by commas and parse values
                try:
                    values = line.split(",")
                    temperature = float(values[0])
                    humidity = float(values[1])
                    ammonia = float(values[2])
                    h2s = float(values[3])
                    co2 = float(values[4])
                    methane = float(values[5])

                    # Insert data into the database
                    insert_sensor_data(temperature, humidity, ammonia, h2s, co2, methane)

                except (ValueError, IndexError) as e:
                    print(f"Error parsing data: {e}")

    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    initialize_database()
    # Update port name based on your operating system and device
    read_from_serial(port="/dev/tty.usbserial-0001", baud_rate=115200)