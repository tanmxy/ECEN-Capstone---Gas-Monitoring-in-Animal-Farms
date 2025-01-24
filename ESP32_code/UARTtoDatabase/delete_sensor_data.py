import sqlite3

# Connect to the database
conn = sqlite3.connect('database.db')
cursor = conn.cursor()

# Delete all rows from the sensor_data table
cursor.execute("DELETE FROM sensor_data")
conn.commit()

print("Everything has been deleted from sensor_data table.")

# Optionally, you can also reset the autoincrement ID counter if needed
cursor.execute("DELETE FROM sqlite_sequence WHERE name='sensor_data'")
conn.commit()

# Close the connection
conn.close()