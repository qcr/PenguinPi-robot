#!usr/bin/python3

import sys
import serial
import struct
import time
import os
import threading
import queue
import datetime
import string
import re

import traceback

# create a comms mutex
comms_mutex = threading.Lock()


#Communications Defines
STARTBYTE = 0x11 #Device Control 1

BIG_ENDIAN = True #NETWORK BYTE ORDER

DGRAM_MAX_LENGTH = 10 #bytes

CRC_8_POLY = 0xAE

#addresses
AD_MOTORS 	= 0x01
AD_MOTOR_A 		= 0x04
AD_MOTOR_B 		= 0x05

AD_SERVOS 	= 0x02
AD_SERVO_A 		= 0x08
AD_SERVO_B 		= 0x09

AD_LEDS 	= 0x03
AD_LED_R 		= 0x0C
AD_LED_G 		= 0x0D
AD_LED_B 		= 0x0E

AD_DISPLAYS	= 0x04
AD_DISPLAY_A 	= 0x10

AD_BTNS 	= 0x05
AD_BTN_A 		= 0x14
AD_BTNS_B 		= 0x15
AD_BTNS_C 		= 0x16

AD_ADCS 	= 0x06
AD_ADC_V 		= 0x18
AD_ADC_C 		= 0x19

AD_OLED		= 0x07




AD_SYSTEM	= 0xF0				#System Control of the AVR i.e taking control of the I2C

AD_ALL 		= 0xFF

#opcodes
#MOTOR
MOTOR_SET_SPEED_DPS = 0x01
MOTOR_SET_DEGREES = 0x02
MOTOR_SET_DIRECTION = 0x03
MOTOR_SET_GAIN_P = 0x04
MOTOR_SET_GAIN_I = 0x05
MOTOR_SET_GAIN_D = 0x06
MOTOR_SET_ENC_MODE = 0x07
MOTOR_SET_ENC = 0x08
MOTOR_SET_CONTROL_MODE = 0x09

MOTOR_GET_SPEED_DPS = 0x81
MOTOR_GET_DEGREES = 0x82
MOTOR_GET_DIRECTION = 0x83
MOTOR_GET_GAIN_P = 0x84
MOTOR_GET_GAIN_I = 0x85
MOTOR_GET_GAIN_D = 0x86
MOTOR_GET_ENC_MODE = 0x87
MOTOR_GET_ENC = 0x88
MOTOR_GET_CONTROL_MODE = 0x89

#SERVO
SERVO_SET_POSITION = 0x01
SERVO_SET_STATE = 0x02
SERVO_SET_MIN_RANGE = 0x03
SERVO_SET_MAX_RANGE = 0x04
SERVO_SET_MIN_PWM = 0x05
SERVO_SET_MAX_PWM = 0x06

SERVO_GET_POSITION = 0x81
SERVO_GET_STATE = 0x82
SERVO_GET_MIN_RANGE = 0x83
SERVO_GET_MAX_RANGE = 0x84
SERVO_GET_MIN_PWM = 0x85
SERVO_GET_MAX_PWM = 0x86

#LED
LED_SET_STATE = 0x01
LED_SET_BRIGHTNESS = 0x02
LED_SET_COUNT = 0x03

LED_GET_STATE = 0x81
LED_GET_BRIGHTNESS = 0x82
LED_GET_COUNT = 0x83

#DISPLAY
DISPLAY_SET_VALUE = 0x01
DISPLAY_SET_DIGIT_1 = 0x02
DISPLAY_SET_DIGIT_0 = 0x03
DISPLAY_SET_MODE = 0x04

DISPLAY_GET_VALUE = 0x81
DISPLAY_GET_DIGIT_1 = 0x82
DISPLAY_GET_DIGIT_0 = 0x83
DISPLAY_GET_MODE = 0x84

#OLED
OLED_SET_IP_ETH_1  =0x01
OLED_SET_IP_ETH_2  =0x02
OLED_SET_IP_ETH_3  =0x03
OLED_SET_IP_ETH_4  =0x04
OLED_SET_IP_WLAN_1 =0x05
OLED_SET_IP_WLAN_2 =0x06
OLED_SET_IP_WLAN_3 =0x07
OLED_SET_IP_WLAN_4 =0x08


#BUTTON
BUTTON_SET_PROGRAM_MODE = 0x01
BUTTON_SET_PIN_MODE = 0x02

BUTTON_GET_PROGRAM_MODE = 0x81
BUTTON_GET_PIN_MODE = 0x82

#ADC
ADC_SET_SCALE = 0x01
ADC_SET_RAW = 0x02
ADC_SET_READING = 0x03

ADC_GET_SCALE = 0x81
ADC_GET_RAW = 0x82
ADC_GET_READING = 0x83

#GLOBAL
ALL_STOP = 0xFF
CLEAR_DATA = 0xEE
SET_RESET = 0xDD
GET_RESET = 0x11
ALL_SET_DIP = 0x20
ALL_GET_DIP = 0xA0
ALL_SET_MOTORS = 0x23
ALL_GET_MOTORS = 0xA3

class UART(object):
    """Setup UART comunication between the raspberry pi and the microcontroler
    """
    def __init__(self, port='/dev/serial0', baud=115200):

        try:
            self.ser = serial.Serial(    port = port,
                                        baudrate = baud,
                                        parity = serial.PARITY_NONE,
                                        stopbits = serial.STOPBITS_ONE,
                                        bytesize = serial.EIGHTBITS,
                                        exclusive = True,
                                        timeout = 1
            )
        except serial.serialutil.SerialException:
            # somebody else has the port open, give up now
            print("can't acquire exclusive access to communications port", file=sys.stderr)
            sys.exit(1)

        self.queue = queue.Queue()
        self.receive_thread = threading.Thread(target=self.uart_recv, daemon=True)
        self.close_event = threading.Event()

    def start(self):
        """Start UART comunication between the raspberry pi and the
        microcontroler
        """
        #opens serial interface and starts recieve handler
        if self.ser.isOpen() is False:
            self.ser.open()

        #reset the buffers
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        #display that connection is started
        print("Transmitting on " + self.ser.name)
        print("Serial settings: " + str(self.ser.baudrate) + " " + str(self.ser.bytesize) + " " + self.ser.parity + " " + str(self.ser.stopbits))

        #start the recive thread
        self.receive_thread.start()

    def stop(self):
        '''Close the comunication between the raspberry pi and the
        microcontroler
        '''
        self.close_event.set()
        self.receive_thread.join()
        self.ser.close()
        self.ser.__del__()

    def flush(self):
        '''flushes input and output buffer, clears queue
        '''
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def putcs(self, dgram):
        '''function: prepends startbyte and puts datagram into write buffer
        '''
        dgram = (struct.pack('!B', STARTBYTE)) + dgram
        self.ser.write(dgram)
        #print(str(datetime.datetime.now())+": DGRAM: ad=0x%02x op=0x%02x" % (dgram[2], dgram[3]) )    ;


    def uart_recv(self):
        '''thread: receives command packets, and prints other messages
        '''
        startLine = True;
        while not self.close_event.is_set():
            try:
                # blocking read on first byte
                byte = self.ser.read(size=1)
                if len(byte) == 0:
                    continue;
                if ord(byte) == 0:
                    continue
                elif ord(byte) == STARTBYTE:
                    # we have a packet start, read the rest

                    paylen = self.ser.read(size=1)  # get the length
                    paylenint = ord(paylen)

                    dgram = self.ser.read(paylenint-1)  # read the rest
                    if len(dgram) != paylenint-1:
                        print('short read', len(dgram), paylenint-1)

                    dgram = paylen + dgram   # datagram is length + the rest

                    # remove the crc byte
                    crcDgram = dgram[-1]
                    dgram = dgram[:-1]

                    #run crcCalc to ensure correct data
                    crcCalc = crc8(dgram, paylenint-1)
                    if crcCalc == crcDgram:
                        # valid CRC, put the datagram into the queue
                        self.queue.put(dgram)
                    else:
                        # bad CRC, print some diagnostics
                        print("ERROR: CRC Failed ", hex(crcCalc), " received ", hex(crcDgram))
                        print_hex(dgram, ' DG')
                        self.queue.put(None)
                else: 
                    # text from the Atmel, perhaps an error message
                    if ord(byte) == 0x0a:
                        # it's a linefeed
                        print("")   # print the LF
                        startLine = True;  # indicate a new line is coming
                    else:
                        if startLine:
                            # print a timestamped message
                            print(str(datetime.datetime.now())+": ", end="");
                            startLine = False;
                        print(chr(ord(byte)), end="")
                            
            except serial.SerialException:
                print("--- caught serial port error")

#create UART object
uart = UART("/dev/serial0", 115200)


def init():
    '''Initlise the UART object
    '''
    time.sleep(0.5)
    uart.start()

def close():
    '''Close the UART object
    '''
    stop_all()
    time.sleep(1)
    uart.stop()
    print("UART stopped and closed")


def crc8(word, length):
    '''cyclic redundancy check
    '''
    crc = 0
    for i in range(0, length):
        crc = crc ^ word[i]
        for j in range(0, 8):
            if crc & 1:
                crc = (crc >> 1) ^ CRC_8_POLY
            else:
                crc =  (crc >> 1)
    return crc


def print_hex(bin, label=''):
    '''print hex value
    '''
    print(label + ': ', " ".join(hex(n) for n in bin))

re_type = re.compile(r"""(?P<t>[a-z0-9]+)  # type
                 (\[
                    (?P<d>[0-9])   # optional dimension
                 \])?
                 """, re.VERBOSE)

def get_struct_format(type):
    if type:
        m = re_type.match(type)

        # get the dimension
        if m.groupdict()['d']:
            n = int(m.groupdict()['d'])
        else:
            n = 1
        paytype = m.groupdict()['t']

        if paytype == 'char' or paytype == 'int8':
            format = 'b'
        elif paytype == 'uchar' or paytype== 'uint8':
            format = 'B'
        elif paytype == 'int16':
            format = 'h'
        elif paytype == 'uint16':
            format = 'H'
        elif paytype == 'int32':
            format = 'i'
        elif paytype == 'uint32':
            format = 'I'
        elif paytype == 'float':
            format = 'f'
        else:
            raise NameError('bad type', type)

        return n*format
    else:
        return ''

def send_datagram(address, opCode, payload=None, paytype='', rxtype=''):
    '''Create and send the datagram to sent to the microcontroler
    '''

    # turn the payload type into a format string for struct.pack
    format = get_struct_format(paytype)

    # if the args are a list, append that to the address and opcode
    args = [address, opCode]
    if payload:
        if type(payload) is list:
            args.extend(payload)
        else:
            args.append(payload)

    # convert to binary string
    dgram = struct.pack('!BB'+format, *args)

    # build up the rest of the datagram
    length = (len(dgram) + 2)
    dgram = (struct.pack("!B", length)) + dgram
    crc = crc8(dgram, length-1)
    dgram = dgram + (struct.pack("!B", crc))

    with comms_mutex:
        # the following code can be executed by only one thread at a time
        uart.putcs(dgram) # send the message to PPI

        # is a response expected?
        if opCode & 0x80:
            # yes, get the response

            # check that rxtype is given
            if not rxtype:
                raise NameError("no receive data type specified");

            # attempt to get message from the queue
            try:
                dgram = uart.queue.get(timeout=0.2)
            except queue.Empty:
                print('-- queue read times out')
                # some error occurred, flag it
                return None

            if dgram:
                result = extract_payload(dgram, address, opCode&0x7f, rxtype)
                if result is None:
                    print('-- error in extract_payload')
            else:
                # datagram is None, indicates failure at the packet RX end
                print('-- null entry in queue')

            uart.queue.task_done()

            return result
        else:
            return

def extract_payload(bin, address, opcode, paytype):
    '''Extracts payload from the microcontroler
       return is int or float
    '''

    format = get_struct_format(paytype)

    if bin[1] == address and bin[2] == opcode:
        # check that address and opcode match
        ret = struct.unpack(format, bin[3:])
        return ret
    else:
        print("address/opcode mismatch:", bin, " expected ", [address,opcode])
        print_hex(bin, 'packet')
        return None

def get_dip():
    '''Read the DIP switches.  
       Switch 1 is the high-order bit.`
       ON means 1.
    '''
    dip = send_datagram(AD_ALL, ALL_GET_DIP, rxtype='uint8') & 0xff
    return dip

def motor_setget(speedL, speedR):
    # send the motor speeds to Atmel
    encoders = send_datagram(AD_ALL, ALL_GET_MOTORS, [speedR,speedL], 'int16[2]', rxtype='uint16[2]')

    return encoders

def stop_all():
    send_datagram(AD_ALL, ALL_STOP)

def clear_data():
    send_datagram(AD_ALL, CLEAR_DATA)

### --- Device Classes --- ###
'''
Motor Object
    Control motors by setting the degrees to the desired distance: make conversion from meters
    to degrees in your own code
'''
class Motor(object):
    '''Motor class uesed with the penguin pi
    '''

    #creates and instance of Motor
    def __init__(self, address):
        self.address = address
        #this might be superflous...
        self.speedDPS = 0
        self.degrees = 0
        self.dir = 0
        self.encoderMode = 1
        self.gainP = 0
        self.gainI = 0
        self.gainD = 0

#SETTERS
    def set_power(self, speed):
        self.speedDPS = speed
        send_datagram(self.address, MOTOR_SET_SPEED_DPS, speed, 'int16')

    #not implmented on the micro controler
    def set_degrees(self, degrees):
        self.degrees = degrees
        send_datagram(self.address, MOTOR_SET_DEGREES, degrees, 'int16')

    def set_direction(self, direction):
        self.dir = direction
        send_datagram(self.address, MOTOR_SET_DIRECTION, direction, 'uint8')

    def set_encoder_mode(self, mode):
        self.encoderMode = mode
        send_datagram(self.address, MOTOR_SET_ENC_MODE, mode, 'uint8')

    #works with set_degrees not implmented on the microcontroler
    def set_PID(self, kP=0, kI=0, kD=0):
        self.gainP = kP
        self.gainI = kI
        self.gainD = kD
        send_datagram(self.address, MOTOR_SET_GAIN_P, kP, 'float')
        send_datagram(self.address, MOTOR_SET_GAIN_I, kI, 'float')
        send_datagram(self.address, MOTOR_SET_GAIN_D, kD, 'float')

#GETTERS
    def get_speed(self):
        return send_datagram(self.address, MOTOR_GET_SPEED_DPS, rxtype='int16')

    def get_ticks(self):
        self.degrees = send_datagram(self.address, MOTOR_GET_DEGREES, rxtype='int16')
        return self.degrees

    def get_encoder(self):
        return send_datagram(self.address, MOTOR_GET_ENC, rxtype='int16')

    def get_direction(self):
        self.dir = send_datagram(self.address, MOTOR_GET_DIRECTION, rxtype='uint8')
        return self.dir

    def get_encoder_mode(self):
        self.encoderMode = send_datagram(self.address, MOTOR_GET_ENC_MODE, rxtype='uint8')
        return self.encoderMode

    def get_PID(self):
        kP=kI=kD = -1

        kP = send_datagram(self.address, MOTOR_GET_GAIN_P, rxtype='float')
        self.gainP = kP

        kI = send_datagram(self.address, MOTOR_GET_GAIN_I, rxtype='float')
        self.gainI = kI

        kD = send_datagram(self.address, MOTOR_GET_GAIN_D, rxtype='float')
        self.gainD = kD

        return kP, kI, kD

    def get_all(self):
        self.get_speed()
        self.get_ticks()
        self.get_direction()
        self.get_PID()
        self.get_encoder_mode()


'''
Servo Object
    position alters the servos output shaft, between the minimum and maximum ranges
    position is clipped to max/min on the ATMEGA side
    neutral position is typically 90 (degrees), in a range of 0-180 (degrees)
    #PWM range can be adjusted to allow for servos with a wider range of control
'''
class Servo(object):

    def __init__(self, address):
        self.address = address
        self.state = 0
        self.position = 90
        self.minRange = 0
        self.maxRange = 180
        self.minPWMRange = 1500
        self.maxPWMRange = 3000

#SETTERS
    def set_position(self, position):
        self.position = position
        send_datagram(self.address, SERVO_SET_POSITION, position, 'int16')

    def set_state(self, state):
        self.state = state
        send_datagram(self.address, SERVO_SET_STATE, state, 'uint8')

    def set_range(self, minimum, maximum):
        self.minRange = minimum
        self.maxRange = maximum
        send_datagram(self.address, SERVO_SET_MIN_RANGE, minimum, 'int16')
        send_datagram(self.address, SERVO_SET_MAX_RANGE, maximum, 'int16')

#GETTERS
    def get_position(self):
        self.position = send_datagram(self.address, SERVO_GET_POSITION, rxtype='int16')
        return self.position

    def get_state(self):
        self.state = send_datagram(self.address, SERVO_GET_STATE, rxtype='uint8')
        return self.state

    def get_range(self):
        self.minRange = send_datagram(self.address, SERVO_GET_MIN_RANGE, rxtype='int16')
        self.maxRange = send_datagram(self.address, SERVO_GET_MAX_RANGE, rxtype='int16')

        return self.minRange, self.maxRange

    def get_PWM_range(self):
        self.minPWMRange = send_datagram(self.address, SERVO_GET_MIN_PWM, rxtype='int16')
        self.maxPWMRange = send_datagram(self.address, SERVO_GET_MAX_PWM, rxtype='int16')

        return self.minPWMRange, self.maxPWMRange

    def get_all(self):
        self.get_position()
        self.get_state()
        self.get_range()
        self.get_PWM_range()

'''
LED object
    state overrides brightness
    brightness is a percentage
    count is a multiplier of 21us, and determines how long the led is lit for
'''
class LED(object):
    def __init__(self, address):
        self.address = address
        self.state = 0
        self.brightness = 0
        self.count = 0
#SETTERS
    def set_state(self, state):
        self.state = state
        send_datagram(self.address, LED_SET_STATE, state, 'uint8')

    def set_brightness(self, brightness):
        self.brightness = brightness
        send_datagram(self.address, LED_SET_BRIGHTNESS, brightness, 'uint8')

    def set_count(self, count):
        self.count = count
        send_datagram(self.address, LED_SET_COUNT, count, 'uint16')

#GETTERS
    def get_state(self):
        self.state = send_datagram(self.address, LED_GET_STATE, rxtype='uint8')
        return self.state

    def get_brightness(self):
        self.brightness = send_datagram(self.address, LED_GET_BRIGHTNESS, rxtype='uint8')
        return self.brightness

    def get_count(self):
        self.count = send_datagram(self.address, LED_GET_COUNT, rxtype='uint16')
        return self.count

    def get_all(self):
        self.get_state()
        self.get_brightness()
        self.get_count()


'''
Display object
    dual digit 7 segment display
    can set each digit individualy: will override the value
'''
class Display(object):
    def __init__(self, address):
        self.address = address
        self.value = 0
        self.digit0 = 0
        self.digit1 = 0
        self.mode = 0    # hex
#SETTERS
    def set_value(self, value):
        self.value = value
        if self.mode == 2 and value < 0:
            # signed mode for a negative number, form the 2's complement
            value = value + 0xff + 1;
        send_datagram(self.address, DISPLAY_SET_VALUE, value, 'uchar')

    def set_digit0(self, digit0):
        self.digit0 = digit0
        send_datagram(self.address, DISPLAY_SET_DIGIT_0, digit0, 'uint8')

    def set_digit1(self, digit1):
        self.digit1 = digit1
        send_datagram(self.address, DISPLAY_SET_DIGIT_1, digit1, 'uint8')

    def set_mode(self, mode):
        if mode == 'x':     # hex %02x
            self.mode = 0
        elif mode == 'u':   # decimal %2d
            self.mode = 1
        elif mode == 'd':   # signed decimal %1d
            self.mode = 2
        else:
            print("ERROR: Incompatible Payload Type Defined. (Python)")
            raise NameError('Debug')
            return 0
        send_datagram(self.address, DISPLAY_SET_MODE, self.mode, 'uint8')

#GETTERS
    def get_value(self):
        self.value = send_datagram(self.address, DISPLAY_GET_VALUE, rxtype='uint8')
        return self.value

    def get_digit0(self):
        self.digit0 = send_datagram(self.address, DISPLAY_GET_DIGIT_0, rxtype='uint8')
        return self.digit0

    def get_digit1(self):
        self.digit1 = send_datagram(self.address, DISPLAY_GET_DIGIT_1, rxtype='uint8')
        return self.digit1

    def get_mode(self):
        self.mode = send_datagram(self.address, DISPLAY_GET_MODE, rxtype='uint8')
        return self.mode


    def get_all(self):
        self.get_value()
        self.get_digit0()
        self.get_digit1()

'''
Button Object
    Controls the behaviour of the onboard buttons
'''
class Button(object):
    def __init__(self, address):
        self.address = address
        self.program_mode = 0
        self.pin_mode = 0

#SETTERS
    def set_program_mode(self, program_mode):
        self.program_mode = program_mode
        send_datagram(self.address, BUTTON_SET_PROGRAM_MODE, program_mode, 'uint8')

    def set_pin_mode(self, pin_mode):
        self.pin_mode = pin_mode
        send_datagram(self.address, BUTTON_SET_PIN_MODE, pin_mode, 'uint8')

#GETTERS
    def get_program_mode(self):
        self.program_mode = send_datagram(self.address, BUTTON_GET_PROGRAM_MODE, rxtype='uint8')
        return self.program_mode

    def get_pin_mode(self):
        self.pin_mode = send_datagram(self.address, BUTTON_GET_PIN_MODE, rxtype='uint8')
        return self.pin_mode

    def get_all(self):
        self.get_program_mode()
        self.get_pin_mode()


'''
ADC Object
    used to retrieve ADC readings from the atmega.
    currently the battery voltage and current are being monitored
    these are AD_ADC_V and AD_ADC_C respectively.
'''
class AnalogIn(object):

    def __init__(self, address):
        self.address = address
        self.raw = 0
        self.value = 0
        self.scale = 0

#SETTERS
    def set_scale(self, scale):
        self.scale = scale
        send_datagram(self.address, ADC_SET_SCALE, scale, 'float')

#GETTERS
    def get_scale(self):
        self.scale = send_datagram(self.address, ADC_GET_SCALE, rxtype='float')
        return self.scale

    def get_raw(self):
        self.raw = send_datagram(self.address, ADC_GET_RAW, rxtype='uint16')
        return self.raw

    def get_value(self):
        self.value = send_datagram(self.address, ADC_GET_READING, rxtype='float')
        return self.value

    def get_all(self):
        self.get_scale()
        self.get_raw()
        self.get_value()
		
		
'''
OLED Object
    used to set registers in the AVR to show information on the OLED
'''
class OLED(object):

    def __init__(self, address):
        self.address = address
        self.ip_eth_1   = 0
        self.ip_eth_2   = 0
        self.ip_eth_3   = 0
        self.ip_eth_4   = 0
        self.ip_wlan_1  = 0
        self.ip_wlan_2  = 0
        self.ip_wlan_3  = 0
        self.ip_wlan_4  = 0
		
#SETTERS
    def set_ip_eth( self, ip_addr ):
        octets = [int(x) for x in ipaddr.split('.')]
        self.ip_eth_1 = octets[0]
        self.ip_eth_2 = octets[1]
        self.ip_eth_3 = octets[2]
        self.ip_eth_4 = octets[3]

        send_datagram(self.address, OLED_SET_IP_ETH_1, self.ip_eth_1, 'uint8')
        send_datagram(self.address, OLED_SET_IP_ETH_2, self.ip_eth_2, 'uint8')
        send_datagram(self.address, OLED_SET_IP_ETH_3, self.ip_eth_3, 'uint8')
        send_datagram(self.address, OLED_SET_IP_ETH_4, self.ip_eth_4, 'uint8')

    def set_ip_wlan( self, ipaddr ):
        octets = [int(x) for x in ipaddr.split('.')]
        self.ip_wlan_1 = octets[0]
        self.ip_wlan_2 = octets[1]
        self.ip_wlan_3 = octets[2]
        self.ip_wlan_4 = octets[3]

        send_datagram(self.address, OLED_SET_IP_WLAN_1, self.ip_wlan_1, 'uint8')
        send_datagram(self.address, OLED_SET_IP_WLAN_2, self.ip_wlan_2, 'uint8')
        send_datagram(self.address, OLED_SET_IP_WLAN_3, self.ip_wlan_3, 'uint8')
        send_datagram(self.address, OLED_SET_IP_WLAN_4, self.ip_wlan_4, 'uint8')
