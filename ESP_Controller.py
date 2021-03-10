#!/usr/bin/env python

import time
import serial

class ESP:
    """Save Station status and parse new status messages."""

    def __init__(self):
        """
        Initialize ESP variables.

        Variables:
            time_stmap: Fecha y Hora sin la Zona []
            temperatura: Temperatura ambiente []
            humedad: Humedad relativa del entorno []
            lluvia: Lluvia con el Pluviometro []
            uv: Indice de Luz UltraVioleta []
            presion: Presion Atmosférica []
            velociadad_viento: Velocidad del viento []
            direccion_viento: Direccion del viento []
            pluviometro []
        """
        self.temperatura = 0.0
        self.humedad = 0.0
        self.lluvia = 0.0
        self.uv = 0.0
        self.presion = 0.0
        self.vlocidad_viento = 0.0
        self.direccion_viento = 0.0
        self.pluviometro = 0.0
        self.is_first_measure = True

    def parse_measurements(self, list_of_msgs=[]):
        """
        Parse a line sent by microcontroller and asign the values to the right variables.

        Parameters:
            list_of_msgs: The string sent by microcontroller
        Returns:
            result: True if parsing was successful, False if it wasn't
        """
        # Skips first measurement bootloader ESP
        if self.is_first_measure:
            self.is_first_measure = False
            return True

        if len(list_of_msgs) and len(list_of_msgs) == 8:
            try:
                self.temperatura = float(list_of_msgs[1])
                self.humedad = float(list_of_msgs[2])
                self.lluvia = float(list_of_msgs[3])
                self.uv = float(list_of_msgs[4])
                self.presion = float(list_of_msgs[5])
                self.vlocidad_viento = float(list_of_msgs[6])
                self.direccion_viento = float(list_of_msgs[7])
                self.pluviometro = float(list_of_msgs[8])

            except Exception as e:
                print("[ARDUINO] Parsing error in Arduino message: " + str(e))
                return False
            return True
        else:
            print("[ARDUINO] Error while parsing message. Message is either empty or does not contain required fields")
            return False

    def get_measurements(self, debug=False, raw_msg=""):
        """
        Packaging of parameters received from microcontroller.

        Parameters:
            debug: Prints a pretty debug message
            raw_msg: Mesage received from microcontroller
        Returns:
            dmsg: Dictionary with station info
            message: Debug message
            raw_msg: Mesage received from microcontroller
        """
        message = 'OK'
        if debug:
            message = "temperatura:" + str(self.temperatura) + "\n"
            message += "humedad:" + str(self.humedad) + "\n"
            message += "lluvia:" + str(self.lluvia) + "\n"
            message += "uv:" + str(self.uv) + "\n"
            message += "presion:" + str(self.presion) + "\n"
            message += "velocidad_viento:" + str(self.vlocidad_viento) + "\n"
            message += "direccion_viento:" + str(self.direccion_viento) + "\n"
            message += "pluviometro:" + str(self.pluviometro) + "\n"

        dmsg = {}
        dmsg['temperatura'] = self.temperatura
        dmsg['humedad'] = self.humedad
        dmsg['uv'] = self.uv
        dmsg['presion'] = self.presion
        dmsg['velocidad_viento'] = self.vlocidad_viento
        dmsg['direccion'] = self.direccion_viento
        dmsg['pluviometro'] = self.pluviometro
        return message, dmsg, raw_msg

class ArduinoSerial:
    """Establish serial communication with microcontroller."""

    ESP8266 = ESP()

    def __init__(self, port='/dev/ttyUSB0', baudrate=19200, timeout=1.0):
        """
        Initialize serial communication variables and sets station limits.

        Parameters:
            port: Microcontroller's serial port.
            baud: Baud rate used in serial communication.
            timeout: Read timeout in seconds.

        Variables:
            time_stmap: Fecha y Hora sin la Zona []
            temperatura: Temperatura ambiente []
            humedad: Humedad relativa del entorno []
            lluvia: Lluvia con el Pluviometro []
            uv: Indice de Luz UltraVioleta []
            presion: Presion Atmosférica []
            velociadad_viento: Velocidad del viento []
            direccion_viento: Direccion del viento []
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ard_serial = serial.Serial()

        self.last_command_time = 0
        self.command_time_delta = 0.05
        
        self.vel_linear_max = 0.6
        self.vel_linear_min = -0.1
        self.vel_angular_max = 1.5
        self.vel_angular_min = -1.5

    def open_serial(self):
        """Establish serial communication with microcontroller. Controller is reset every time a serial com is opened."""
        try:
            self.ard_serial = serial.Serial(self.port,
                                            baudrate=self.baudrate,
                                            bytesize=serial.EIGHTBITS,
                                            parity=serial.PARITY_NONE,
                                            stopbits=serial.STOPBITS_ONE,
                                            timeout=self.timeout,
                                            xonxoff=0,
                                            rtscts=0
                                            )
        except Exception as e:
            print("[ARDUINO] Error while opening serial communication: " + str(e))
            return False
        self.ard_serial.setDTR(False)
        time.sleep(1)
        self.ard_serial.flushInput()
        self.ard_serial.setDTR(True)
        time.sleep(3)
        return True

    def read_message(self, debug=False):
        """
        Read message sent by microcontroller.

        Parameters:
            debug: prints debug info
        Returns:
            message: Debug message
            dmsg: Dictionary with station status info
            raw_msg: Mesage received from microcontroller
        """
        if self.ard_serial.inWaiting() > 0:
            # self.ard_serial.flushInput()
            raw_msg = self.ard_serial.readline()
            message = raw_msg.rstrip('\n')
            self.ESP8266.parse_measurements(message.split())
            return self.ESP8266.get_measurements(debug, raw_msg)
        else:
            return [], {}, ""

    def close_serial(self):
        """Stop station, disable motors and close serial communication with microcontroller."""
        time.sleep(2)
        self.ard_serial.close()