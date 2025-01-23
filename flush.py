import mysql.connector  # MySQL connector for database operations
from mysql.connector import Error  # For handling MySQL-related errors

# MySQL database configuration (modify these details based on your database setup)
db_config = {
    'host': 'database-1.cjeamsa0et2u.us-east-2.rds.amazonaws.com',  # Database host (AWS RDS endpoint in this case)
    'user': 'admin',  # Database username
    'password': 'GasMonitoringTeam2024',  # Database password
    'database': 'test_database'  # Target database name
}

try:
    # Attempt to connect to the MySQL database using the provided configuration
    connection = mysql.connector.connect(**db_config)
    cursor = connection.cursor()  # Create a cursor object to execute SQL queries

    # SQL query to delete all rows from the gas_data table
    # Truncate table resets the table and clears all rows efficiently. 
    truncate_query = 'TRUNCATE TABLE gas_data'

    # Execute the truncate query
    cursor.execute(truncate_query)

    # Commit the transaction to confirm the changes
    connection.commit()
    print("All data in gas_data table has been flushed.")  # Notify the user of successful operation

except Error as e:
    # Print any error that occurs during the database operation
    print(f"Error: {e}")

finally:
    # Ensure that the database connection and cursor are properly closed
    if 'connection' in locals() and connection.is_connected():
        cursor.close()  # Close the cursor
        connection.close()  # Close the database connection
