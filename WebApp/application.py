from flask import Flask, render_template, jsonify, send_file, request
from flask_cors import CORS
import mysql.connector
from mysql.connector import Error
from datetime import datetime, timedelta
import pytz
import csv
import os

# Initialize Flask app — must be named 'application' for EB
application = Flask(__name__, template_folder='templates')
CORS(application)

db_config = {
    'host': 'database-1.cjweoy8siati.us-east-2.rds.amazonaws.com',
    'user': 'admin',
    'password': os.environ.get('DB_PASSWORD') or 'GasMonitoringTeam2024',
    'database': 'tempsensor_data'
}

# MySQL database configuration (Using environment variable for password)
#db_config = {
    #'host': 'database-1.cjweoy8siati.us-east-2.rds.amazonaws.com',
   # 'user': 'admin',
    #'password': os.environ.get('DB_PASSWORD'),  # AWS EB will provide this
    #'database': 'tempsensor_data'
#}

# Timezone configuration (Central Time)
tz = pytz.timezone('US/Central')

from flask import render_template

@application.route('/')
def home():
    return render_template('index.html')
# --------------------------- SENSOR DATA ROUTE ---------------------------

@application.route('/get_sensor_data', methods=['GET'])
def get_sensor_data():
    """ Fetch the latest sensor readings from MySQL """
    try:
        connection = mysql.connector.connect(**db_config)
        cursor = connection.cursor()

        # Fetch recent sensor readings
        cursor.execute('''
            SELECT id, temperature, humidity, CO2, NH3, CH4, H2S, timestamp 
            FROM sensor_readings ORDER BY timestamp DESC
        ''')
        rows = cursor.fetchall()

        # Calculate stats for last 360 entries (or fewer)
        cursor.execute('''
            SELECT temperature, humidity, CO2, NH3, CH4, H2S
            FROM sensor_readings ORDER BY timestamp DESC LIMIT 360
        ''')
        recent_data = cursor.fetchall()

        def stats_for(col_index):
            values = [r[col_index] for r in recent_data if r[col_index] is not None]
            if not values:
                return {'average': None, 'min': None, 'max': None}
            return {
                'average': round(sum(values)/len(values), 2),
                'min': round(min(values), 2),
                'max': round(max(values), 2)
            }

        stats_data = {
            'temperature': stats_for(0),
            'humidity': stats_for(1),
            'CO2': stats_for(2),
            'NH3': stats_for(3),
            'CH4': stats_for(4),
            'H2S': stats_for(5)
        }

        # Convert to JSON format with proper CST formatting
        data = [{
            'id': row[0],
            'temperature': round(row[1], 2) if row[1] else None,
            'humidity': round(row[2], 2) if row[2] else None,
            'CO2': row[3],
            'NH3': row[4],
            'CH4': row[5],
            'H2S': row[6],
            'timestamp': pytz.utc.localize(row[7]).astimezone(tz).strftime('%Y-%m-%d %H:%M:%S')
        } for row in rows]

        return jsonify({'data': data, 'stats': stats_data})

    except Error as e:
        return jsonify({'error': str(e)}), 500
    finally:
        if 'connection' in locals() and connection.is_connected():
            cursor.close()
            connection.close()

# --------------------------- CSV EXPORT ROUTE ---------------------------

@application.route('/export_csv', methods=['GET'])
def export_csv():
    """ Export sensor data (temperature, humidity, CO2, NH3, CH4, H2S) to a CSV file """
    try:
        connection = mysql.connector.connect(**db_config)
        cursor = connection.cursor()

        cursor.execute('SELECT * FROM sensor_readings ORDER BY timestamp DESC')
        rows = cursor.fetchall()

        csv_file_path = 'sensor_readings_export.csv'
        with open(csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['ID', 'Timestamp', 'Temperature (°C)', 'Humidity (%)', 'CO2 (ppm)', 'NH3', 'H2S', 'CH4'])
            for row in rows:
                timestamp = pytz.utc.localize(row[1]).astimezone(tz).strftime('%Y-%m-%d %H:%M:%S') if row[1] else ''
                writer.writerow([row[0], timestamp, row[2], row[3], row[4], row[5], row[7], row[6]])

        return send_file(csv_file_path, as_attachment=True)

    except Error as e:
        return jsonify({'error': str(e)}), 500
    finally:
        if 'connection' in locals() and connection.is_connected():
            cursor.close()
            connection.close()


@application.route('/get_visualization_data', methods=['GET'])
def get_visualization_data():
    """ Return the last 2160 rows of data for visualization """
    try:
        connection = mysql.connector.connect(**db_config)
        cursor = connection.cursor()

        cursor.execute('''
            SELECT temperature, humidity, CO2, NH3, CH4, H2S, timestamp
            FROM sensor_readings
            ORDER BY timestamp DESC
            LIMIT 2160
        ''')
        rows = cursor.fetchall()

        # Reverse to chronological order and format timestamps to CST
        data = [{
            'temperature': round(row[0], 2) if row[0] else None,
            'humidity': round(row[1], 2) if row[1] else None,
            'CO2': row[2],
            'NH3': row[3],
            'CH4': row[4],
            'H2S': row[5],
            'timestamp': pytz.utc.localize(row[6]).astimezone(tz).strftime('%H:%M')
        } for row in reversed(rows)]

        return jsonify(data)

    except Error as e:
        return jsonify({'error': str(e)}), 500
    finally:
        if 'connection' in locals() and connection.is_connected():
            cursor.close()
            connection.close()


# --------------------------- RUN LOCAL DEV SERVER ---------------------------

if __name__ == '__main__':
    application.run(host='0.0.0.0', port=8080)

