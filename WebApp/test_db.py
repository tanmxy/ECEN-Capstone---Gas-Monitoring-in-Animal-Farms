import sqlite3  # Importing sqlite database to use the library
from datetime import datetime  # Helps with timestamps
import random  # Creating random test data
import time  # To add delays for periodic insertion

# Function to generate realistic values for different gases
def generate_data():
    # Connect to the SQLite database
    connection = sqlite3.connect('test_database.db')
    cursor = connection.cursor()
    
    # Create a table with columns for each gas
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS gas_data (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            ammonia REAL NOT NULL,
            carbon_dioxide REAL NOT NULL,
            methane REAL NOT NULL,
            nitrous_oxide REAL NOT NULL,
            hydrogen_sulfide REAL NOT NULL,
            timestamp TEXT DEFAULT (datetime('now', 'localtime'))
        )
    ''')

    # Clear previous data from the table before inserting new data
    cursor.execute('DELETE FROM gas_data')
    
    # Generate 30 entries
    for _ in range(30):
        # Generate random realistic values for each gas and round to 4 decimals
        ammonia = round(random.uniform(0, 25), 4)  # Ammonia in ppm, safe range typically 0-25 ppm
        carbon_dioxide = round(random.uniform(350, 1000), 4)  # Carbon Dioxide in ppm, safe range 350-1000 ppm
        methane = round(random.uniform(0, 10), 4)  # Methane in ppm, safe range typically 0-10 ppm
        nitrous_oxide = round(random.uniform(0, 1), 4)  # Nitrous Oxide in ppm, safe range typically 0-1 ppm
        hydrogen_sulfide = round(random.uniform(0, 5), 4)  # Hydrogen Sulfide in ppm, safe range typically 0-5 ppm

        # Insert the values into the database
        cursor.execute('''
            INSERT INTO gas_data (ammonia, carbon_dioxide, methane, nitrous_oxide, hydrogen_sulfide)
            VALUES (?, ?, ?, ?, ?)
        ''', (ammonia, carbon_dioxide, methane, nitrous_oxide, hydrogen_sulfide))
    
    # Commit changes and close the connection
    connection.commit()
    connection.close()
    print(f"Generated 30 new data points at {datetime.now()}")

# Run the data generation process every 5 minutes
while True:
    generate_data()
    time.sleep(300)  # 300 seconds = 5 minutes
