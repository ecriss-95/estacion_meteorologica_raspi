#!/usr/bin/env python

import os
import sys
import json
import time
import atexit
import psycopg2
import pytz
from psycopg2 import Error
from timeloop import Timeloop
from datetime import timedelta, datetime, tzinfo
from ESP_Controller import ArduinoSerial

interval = 60
timer_serial_read = Timeloop()

class ESPInterface(object):
    """
    Establish communication between.

    """

    def __init__(self):
        """
        Initialize node.

        """

        self.port = "/dev/ttyUSB0"    # Puerto Serial ESP8266
        self.baud = 19200
        self.timeout = 1
        self.debug = 1

        self.arduino = None

        print("[ESP CONTROLLER] Trying to connect to ESP base (port=%s, baud=%d, timeout=%f)"
                      % (self.port, self.baud, self.timeout))

        try:
            # Create an instance of ArduinoSerial to communicate with arduino via serial port
            self.arduino = ArduinoSerial(self.port, self.baud, self.timeout)

            if self.arduino.open_serial():
                atexit.register(self.kill_arduino)
                print("[ESP CONTROLLER] Connected! Ready to operate")
            else:
                print("[ESP CONTROLLER] Unable to open connection to ESP base. Please check controller.")
        except Exception as e:
            print("[ROBOT CONTROLLER] Unable to connect to ESP base: " + str(e))
    
    def read_from_arduino(self):
        """Read message receive from microcontroller and save dataBase."""
        message, dmsg, raw_msg = self.arduino.read_message(debug=True)

        if message:
            print(raw_msg)
            try:
                
                # Connect to an existing database (IP Raspberry)
                connection = psycopg2.connect(user="pi",
                                            password="123456",
                                            host="192.168.100.60",
                                            port="5432",
                                            database="pi")        

                print("PostgreSQL Connected... ")
                # Create a cursor to perform database operations
                cursor = connection.cursor()

                # Executing a SQL query to insert data into  table
                insert_query = """ INSERT INTO estacion_data (time, temperatura, humedad, lluvia, uv, presion, velocidad_viento,
                                   direccion_viento, pluviometro) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)"""
                item_purchase_time = datetime.now()
                item_tuple = (item_purchase_time, dmsg['temperatura'], dmsg['humedad'], 
                              dmsg['lluvia'], dmsg['uv'], dmsg['presion'], dmsg['velocidad_viento'],
                              dmsg['direccion_viento'], dmsg['pluviometro'])

                cursor.execute(insert_query,item_tuple)
                connection.commit()
                print("1 Record inserted successfully")
                # # Fetch result
                # cursor.execute("SELECT * from estacion_data")
                # record = cursor.fetchall()
                # print("Result ", record)

            except (Exception, Error) as error:
                print("Error while connecting to PostgreSQL", error)
            finally:
                if (connection):
                    cursor.close()
                    connection.close()
                    print("PostgreSQL connection is closed")
                    print(" ")
        
        

    def kill_arduino(self):
        """Close serial communication with microcontroller."""
        self.arduino.close_serial()
        print("[ESP CONTROLLER] Exit :: ESP base disconnected successfully")


@timer_serial_read.job(interval=timedelta(seconds=interval))
def ESP_serial_read():
    esp8266Interface.read_from_arduino()
    

if __name__ == "__main__":

    esp8266Interface = ESPInterface()
    # init timer to get readings from arduino
    timer_serial_read.start(block=True)
