import mysql.connector  # MySQL connector for database interactions
import random  # To generate random gas concentration values
import time  # To pause execution between insertions
from mysql.connector import Error  # For handling MySQL errors
from datetime import datetime  # To get the current date and time
import pytz  # To handle timezone conversions

# MySQL database configuration (modify these details as per your database setup)
db_config = {
    'host': 'database-1.cjeamsa0et2u.us-east-2.rds.amazonaws.com',  # AWS RDS endpoint
    'user': 'admin',  # Username for the database
    'password': 'GasMonitoringTeam2024',  # Password for the database
    'database': 'test_database'  # Target database name
}

# Define the timezone for consistent timestamping (Central Time in this case)
tz = pytz.timezone('US/Central')

def insert_gas_data():
    """
    Connects to the MySQL database, generates a new gas data point with a timestamp, 
    and inserts it into the gas_data table.
    """
    try:
        # Connect to the MySQL database
        connection = mysql.connector.connect(**db_config)
        cursor = connection.cursor()

        # Generate random values for various gases (in ppm within their safe ranges)
        ammonia = round(random.uniform(0, 25), 4)  # Ammonia in parts per million
        carbon_dioxide = round(random.uniform(350, 1000), 4)  # Carbon Dioxide in ppm
        methane = round(random.uniform(0, 10), 4)  # Methane in ppm
        nitrous_oxide = round(random.uniform(0, 1), 4)  # Nitrous Oxide in ppm
        hydrogen_sulfide = round(random.uniform(0, 5), 4)  # Hydrogen Sulfide in ppm

        # Get the current timestamp in the specified timezone
        timestamp = datetime.now(tz).strftime('%Y-%m-%d %H:%M:%S')

        # SQL query to insert the generated data into the gas_data table
        insert_query = '''
        INSERT INTO gas_data (ammonia, carbon_dioxide, methane, nitrous_oxide, hydrogen_sulfide, timestamp)
        VALUES (%s, %s, %s, %s, %s, %s)
        '''
        # Execute the SQL query with the generated data
        cursor.execute(insert_query, (ammonia, carbon_dioxide, methane, nitrous_oxide, hydrogen_sulfide, timestamp))

        # Commit the transaction to save changes to the database
        connection.commit()
        print("Inserted 1 new gas data point.")

    except Error as e:
        # Handle and display database connection or execution errors
        print(f"Error: {e}")
        
    finally:
        # Ensure the database connection is properly closed to avoid resource leaks
        if 'connection' in locals() and connection.is_connected():
            cursor.close()  # Close the cursor
            connection.close()  # Close the connection

# Continuously generate and insert data every 5 seconds
if __name__ == "__main__":
    while True:
        insert_gas_data()  # Call the function to insert a new data point
        time.sleep(5)  # Wait for 5 seconds before the next iteration
