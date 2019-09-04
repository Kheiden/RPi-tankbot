#!/usr/bin/env python3
from subprocess import check_output

import mysql.connector

def get_readings():
    connection = mysql.connector.connect(user='service_account', password='baseball0',
                                                          host='192.168.1.10',
                                                          port='3306')
    database_name = 'robot'
    table_name = 'ambient_air_data'
    cursor = connection.cursor()
    number_readings_to_take = 2

    while True:
      out = check_output(["/home/pi/Adafruit_BME680/bme680",
                          str(number_readings_to_take),
                          str(number_readings_to_take)])
      for i in range(number_readings_to_take):
        data = str(out).split("\\n")[i + 1]
        datetime = data.split(' T:')[0]
        temperature = data.split(' degC')[0].split(' T: ')[1]
        pressure = data.split(' P: ')[1].split(' hPa')[0]
        humidity = data.split(' H: ')[1].split(' %rH')[0]
        gas = data.split(' G: ')[1].split(' Ohms')[0]

        try:
            table_columns = "(`datetime`, `temperature_celsius`, `pressure_hPa`, `percent_relative_humidity`, `gas_resistance`)"
            table_values = (datetime, temperature, pressure, humidity, gas)
            query = "INSERT INTO {database_name}.{table_name} {table_columns} VALUES {table_values}".format(
                database_name=database_name,
                table_name=table_name,
                table_columns=table_columns,
                table_values=table_values)
            cursor.execute(query)
            output = connection.commit()
        except:
            pass

    cursor.close()
get_readings()
