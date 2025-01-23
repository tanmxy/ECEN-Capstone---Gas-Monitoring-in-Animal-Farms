from flask import Flask, jsonify, send_file, request
from flask_cors import CORS
import mysql.connector
from mysql.connector import Error
from datetime import datetime, timedelta
import pytz
import csv
import os
import getpass
from werkzeug.security import generate_password_hash, check_password_hash

# Prompt the user for the database password until the correct password is provided
correct_password = 'GasMonitoringTeam2024'
db_password = ''
while db_password != correct_password:
    db_password = getpass.getpass(prompt='Enter the MySQL database password: ')
    if db_password != correct_password:
        print("Incorrect password. Please try again.")

# Initialize the Flask application
app = Flask(__name__)
CORS(app)  # Enable CORS for all routes to allow cross-origin requests

# MySQL database configuration settings
db_config = {
    'host': 'database-1.cjeamsa0et2u.us-east-2.rds.amazonaws.com',
    'user': 'admin',
    'password': db_password,
    'database': 'test_database'
}

# Define the timezone for date and time processing
tz = pytz.timezone('US/Central')

# Route: Register a new user
@app.route('/register', methods=['POST'])
def register():
    data = request.json  # Get JSON data from the frontend request
    username = data.get('username')
    password = data.get('password')

    # Validate input fields
    if not username or not password:
        return jsonify({"error": "Username and password are required."}), 400

    # Hash the password for secure storage in the database
    hashed_password = generate_password_hash(password)

    try:
        # Connect to the database
        connection = mysql.connector.connect(**db_config)
        cursor = connection.cursor()

        # Insert the new user into the database
        query = "INSERT INTO users (username, password) VALUES (%s, %s)"
        cursor.execute(query, (username, hashed_password))
        connection.commit()  # Save changes to the database

        return jsonify({"message": "User registered successfully!"}), 201

    except mysql.connector.IntegrityError:
        # Handle case where username already exists in the database
        return jsonify({"error": "Username already exists."}), 400
    except Error as e:
        # Handle other database errors and return them to the frontend
        return jsonify({"error": str(e)}), 500
    finally:
        # Ensure the database connection is properly closed
        if 'connection' in locals() and connection.is_connected():
            cursor.close()
            connection.close()

# Route: User login
@app.route('/login', methods=['POST'])
def login():
    data = request.json  # Get JSON data from the frontend request
    username = data.get('username')
    password = data.get('password')

    # Validate input fields
    if not username or not password:
        return jsonify({"error": "Username and password are required."}), 400

    try:
        # Connect to the database
        connection = mysql.connector.connect(**db_config)
        cursor = connection.cursor()

        # Query the database for the user's hashed password
        query = "SELECT password FROM users WHERE username = %s"
        cursor.execute(query, (username,))
        result = cursor.fetchone()  # Fetch one result; we only expect one user entry

        if result:
            # Extract the hashed password from the query result
            stored_password = result[0]
            # Verify if the provided password matches the hashed password
            if check_password_hash(stored_password, password):
                return jsonify({"message": "Login successful!"}), 200
            else:
                return jsonify({"error": "Invalid password."}), 401
        else:
            return jsonify({"error": "Username does not exist."}), 404

    except Error as e:
        # Handle any database error
        return jsonify({"error": str(e)}), 500
    finally:
        # Ensure the database connection is properly closed
        if 'connection' in locals() and connection.is_connected():
            cursor.close()
            connection.close()

# Route: Get gas data for monitoring
@app.route('/get_gas_data', methods=['GET'])
def get_gas_data():
    try:
        # Connect to the database
        connection = mysql.connector.connect(**db_config)
        cursor = connection.cursor()

        # Delete records older than 30 minutes to maintain recent data
        thirty_minutes_ago = datetime.now(tz) - timedelta(minutes=30)
        formatted_time = thirty_minutes_ago.strftime('%Y-%m-%d %H:%M:%S')
        delete_query = 'DELETE FROM gas_data WHERE timestamp < %s'
        cursor.execute(delete_query, (formatted_time,))
        connection.commit()  # Commit changes to delete old records

        # Fetch all rows from the gas_data table, ordered by timestamp (latest first)
        cursor.execute('SELECT * FROM gas_data ORDER BY timestamp DESC')
        rows = cursor.fetchall()  # Fetch all records from the query

        # Calculate statistics (average, min, max) for each gas type in the last 30 minutes
        cursor.execute('''
            SELECT AVG(ammonia), MIN(ammonia), MAX(ammonia),
                   AVG(carbon_dioxide), MIN(carbon_dioxide), MAX(carbon_dioxide),
                   AVG(methane), MIN(methane), MAX(methane),
                   AVG(nitrous_oxide), MIN(nitrous_oxide), MAX(nitrous_oxide),
                   AVG(hydrogen_sulfide), MIN(hydrogen_sulfide), MAX(hydrogen_sulfide)
            FROM gas_data
            WHERE timestamp >= %s
        ''', (formatted_time,))
        stats = cursor.fetchone()  # Fetch the calculated statistics for each gas type

        # Convert database rows to a list of dictionaries for JSON response
        data = [{
            'id': row[0],  # Record ID
            'ammonia': round(row[1], 4),  # Ammonia concentration rounded to 4 decimal places
            'carbon_dioxide': round(row[2], 4),  # Carbon Dioxide concentration
            'methane': round(row[3], 4),  # Methane concentration
            'nitrous_oxide': round(row[4], 4),  # Nitrous Oxide concentration
            'hydrogen_sulfide': round(row[5], 4),  # Hydrogen Sulfide concentration
            'timestamp': row[6].astimezone(tz).strftime('%Y-%m-%d %H:%M:%S')  # Convert timestamp to specified timezone
        } for row in rows]

        # Prepare average, min, and max statistics for the response
        stats_data = {
            'ammonia': {'average': round(stats[0], 4) if stats[0] else None, 'min': round(stats[1], 4) if stats[1] else None, 'max': round(stats[2], 4) if stats[2] else None},
            'carbon_dioxide': {'average': round(stats[3], 4) if stats[3] else None, 'min': round(stats[4], 4) if stats[4] else None, 'max': round(stats[5], 4) if stats[5] else None},
            'methane': {'average': round(stats[6], 4) if stats[6] else None, 'min': round(stats[7], 4) if stats[7] else None, 'max': round(stats[8], 4) if stats[8] else None},
            'nitrous_oxide': {'average': round(stats[9], 4) if stats[9] else None, 'min': round(stats[10], 4) if stats[10] else None, 'max': round(stats[11], 4) if stats[11] else None},
            'hydrogen_sulfide': {'average': round(stats[12], 4) if stats[12] else None, 'min': round(stats[13], 4) if stats[13] else None, 'max': round(stats[14], 4) if stats[14] else None}
        }

        return jsonify({'data': data, 'stats': stats_data})

    except Error as e:
        # Handle any database error and return it to the frontend
        return jsonify({'error': str(e)}), 500
    finally:
        # Ensure the database connection is properly closed
        if 'connection' in locals() and connection.is_connected():
            cursor.close()
            connection.close()

# Route: Export gas data to CSV
@app.route('/export_csv', methods=['GET'])
def export_csv():
    try:
        # Connect to the database
        connection = mysql.connector.connect(**db_config)
        cursor = connection.cursor()

        # Fetch all rows from the gas_data table
        cursor.execute('SELECT * FROM gas_data ORDER BY timestamp DESC')
        rows = cursor.fetchall()

        # Define the file path and name for the CSV file
        csv_file_path = 'gas_data_export.csv'

        # Write data to the CSV file
        with open(csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Write header row
            writer.writerow(['ID', 'Ammonia', 'Carbon Dioxide', 'Methane', 'Nitrous Oxide', 'Hydrogen Sulfide', 'Timestamp'])
            # Write data rows
            for row in rows:
                writer.writerow(row)

        # Send the CSV file to the user
        return send_file(csv_file_path, as_attachment=True)

    except Error as e:
        # Handle any database error
        return jsonify({'error': str(e)}), 500
    finally:
        # Ensure the database connection is properly closed
        if 'connection' in locals() and connection.is_connected():
            cursor.close()
            connection.close()

# Run the Flask server on port 5000 with debug mode enabled
if __name__ == '__main__':
    app.run(port=5000, debug=True)
