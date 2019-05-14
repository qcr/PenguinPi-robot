#!usr/bin/python3

import serial
import struct
import time
import os
import threading
import queue

#Communications Defines
STARTBYTE = 0x11 #Device Control 1

BIG_ENDIAN = True #NETWORK BYTE ORDER

DGRAM_MAX_LENGTH = 10 #bytes

CRC_8_POLY = 0xAE

#addresses
AD_MOTORS = 0x01
AD_MOTOR_A = 0x04
AD_MOTOR_B = 0x05

AD_SERVOS = 0x02
AD_SERVO_A = 0x08
AD_SERVO_B = 0x09

AD_LEDS = 0x03
AD_LED_R = 0x0C
AD_LED_G = 0x0D
AD_LED_B = 0x0E

AD_DISPLAYS = 0x04
AD_DISPLAY_A = 0x10

AD_BTNS = 0x05
AD_BTN_A = 0x14
AD_BTNS_B = 0x15
AD_BTNS_C = 0x16

AD_ADCS = 0x06
AD_ADC_V = 0x18
AD_ADC_C = 0x19

AD_ALL = 0xFF

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

class UART(object):
    """Setup UART comunication between the raspberry pi and the microcontroler
    """
    def __init__(self, port='/dev/attyAMA0', baud=115200):
        self.ser = serial.Serial(    port = port,
                                    baudrate = baud,
                                    parity = serial.PARITY_NONE,
                                    stopbits = serial.STOPBITS_ONE,
                                    bytesize = serial.EIGHTBITS,
                                    timeout = 1
        )
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

    def uart_recv(self):
        '''thread: receives command packets, and prints other messages
        '''
        while not self.close_event.is_set():
            #read first byte
            com = self.ser.read(size=1)
            if len(com) == 0:
                continue;
            if ord(com) == 0:
                continue
            elif ord(com) == STARTBYTE:
                #extract the packet from the UART
                paylen = self.ser.read()
                paylenint, = struct.unpack("!B", paylen)
                dgram = self.ser.read(paylenint-2)
                crcDgram = self.ser.read()
                dgram = paylen + dgram
                crcDgram, = struct.unpack("!B", crcDgram)\
                #run crcCalc to ensure correct data
                crcCalc = crc8(dgram, paylenint-1)
                if crcCalc == crcDgram:
                    self.queue.put(dgram)
                else:
                    #todo: throw an exception?
                    print("ERROR: CRC Failed (Python)")
                    print("Calculated CRC: " + hex(crcCalc) + " Recieved CRC: " + hex(crcDgram))

            else: #displayable
                if ord(com) == 10:
                    print(" ")
                else:
                    print(com.decode("utf-8", "ignore"))

#create UART object
uart = UART("/dev/ttyAMA0", 115200)



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


def print_hex(bin):
    '''print hex value
    '''
    print(" ".join(hex(n) for n in bin))


def form_datagram(address, opCode, payload=0x00, paytype=''):
    '''Create the datagram to sent to the microcontroler
    '''
    if paytype == 'char':
        #form a bytestring of the payload
        bin = ((struct.pack("!BBb", address, opCode, payload)))
    elif paytype == 'uchar':
        #form a bytestring of the payload
        bin = ((struct.pack("!BBB", address, opCode, payload)))
    elif paytype == 'int':
        #h represents a 2 byte signed int
        bin = ((struct.pack("!BBh", address, opCode, payload)))
    elif paytype == 'float':
        bin = ((struct.pack("!BBf", address, opCode, payload)))
    elif paytype == '':
        #empty payload: probably a getter
        bin = ((struct.pack("!BB", address, opCode)))
    else :
        print("ERROR: Incompatible Payload Type Defined. (Python)")
        return 0
    length = (len(bin) + 2)
    bin = (struct.pack("!B", length)) + bin
    crc = crc8(bin, length-1)
    bin = bin + (struct.pack("!B", crc))
    return bin


def extract_payload(bin, address, opcode, paytype):
    '''Extracts payload from the microcontroler
    '''
    if paytype == 'char':
        upackstr = "!b"
    elif paytype == 'int':
        upackstr = "!h" #h represents a 2 byte signed int
    elif paytype == 'float':
        upackstr = "!f"
    else :
        print("ERROR: Incompatible Payload Type Defined. (Python)")
        return 0

    if bin[1] == address:
        if bin[2] == opcode:
            com, = struct.unpack(upackstr, bin[3:])
            return com
    else:
        print(bin.decode("utf-8", "ignore"))
        return 0


def get_variable(address, opcode, paytype, timeout=2):
    '''requests vearables from the microcontroler
    '''
    dgram = form_datagram(address, opcode)
    uart.putcs(dgram)
    bin = uart.queue.get()
    #clear the MSB of the opcode
    opcode = opcode & 0b01111111
    payload = extract_payload(bin, address, opcode, paytype)
    uart.queue.task_done()
    return payload


def stop_all():
    dgram = form_datagram(AD_ALL, ALL_STOP)
    uart.putcs(dgram)

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
        dgram = form_datagram(self.address, MOTOR_SET_SPEED_DPS, speed, 'int')
        uart.putcs(dgram)

    #not implmented on the micro controler
    def set_degrees(self, degrees):
        self.degrees = degrees
        dgram = form_datagram(self.address, MOTOR_SET_DEGREES, degrees, 'int')
        uart.putcs(dgram)

    def set_direction(self, direction):
        self.dir = direction
        dgram = form_datagram(self.address, MOTOR_SET_DIRECTION, direction, 'char')
        uart.putcs(dgram)

    def set_encoder_mode(self, mode):
        self.encoderMode = mode
        dgram = form_datagram(self.address, MOTOR_SET_ENC_MODE, mode, 'char')
        uart.putcs(dgram)

    #works with set_degrees not implmented on the microcontroler
    def set_PID(self, kP=0, kI=0, kD=0):
        self.gainP = kP
        self.gainI = kI
        self.gainD = kD
        dgram = form_datagram(self.address, MOTOR_SET_GAIN_P, kP, 'float')
        uart.putcs(dgram)
        dgram = form_datagram(self.address, MOTOR_SET_GAIN_I, kI, 'float')
        uart.putcs(dgram)
        dgram = form_datagram(self.address, MOTOR_SET_GAIN_D, kD, 'float')
        uart.putcs(dgram)

#GETTERS
    def get_speed(self):
        self.speedDPS = get_variable(self.address, MOTOR_GET_SPEED_DPS, 'int')
        return self.speedDPS

    def get_ticks(self):
        self.degrees = get_variable(self.address, MOTOR_GET_DEGREES, 'int')
        return self.degrees

    def get_encoder(self):
        enc = get_variable(self.address, MOTOR_GET_ENC, 'int')
        return enc

    def get_direction(self):
        self.dir = get_variable(self.address, MOTOR_GET_DIRECTION, 'char')
        return self.dir

    def get_encoder_mode(self):
        self.encoderMode = get_variable(self.address, MOTOR_GET_ENC_MODE, 'char')
        return self.encoderMode

    def get_PID(self):
        kP=kI=kD = -1

        kP = get_variable(self.address, MOTOR_GET_GAIN_P, 'float')
        self.gainP = kP

        kI = get_variable(self.address, MOTOR_GET_GAIN_I, 'float')
        self.gainI = kI

        kD = get_variable(self.address, MOTOR_GET_GAIN_D, 'float')
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
        dgram = form_datagram(self.address, SERVO_SET_POSITION, position, 'int')
        uart.putcs(dgram)

    def set_state(self, state):
        self.state = state
        dgram = form_datagram(self.address, SERVO_SET_STATE, state, 'char')
        uart.putcs(dgram)

    def set_range(self, minimum, maximum):
        self.minRange = minimum
        self.maxRange = maximum
        dgram = form_datagram(self.address, SERVO_SET_MIN_RANGE, minimum, 'int')
        uart.putcs(dgram)
        dgram = form_datagram(self.address, SERVO_SET_MAX_RANGE, maximum, 'int')
        uart.putcs(dgram)

#GETTERS
    def get_position(self):
        self.position = get_variable(self.address, SERVO_GET_POSITION, 'int')
        return self.position

    def get_state(self):
        self.state = get_variable(self.address, SERVO_GET_STATE, 'char')
        return self.state

    def get_range(self):
        self.minRange = get_variable(self.address, SERVO_GET_MIN_RANGE, 'int')
        self.maxRange = get_variable(self.address, SERVO_GET_MAX_RANGE, 'int')

        return self.minRange, self.maxRange

    def get_PWM_range(self):
        self.minPWMRange = get_variable(self.address, SERVO_GET_MIN_PWM, 'int')
        self.maxPWMRange = get_variable(self.address, SERVO_GET_MAX_PWM, 'int')

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
        dgram = form_datagram(self.address, LED_SET_STATE, state, 'char')
        uart.putcs(dgram)

    def set_brightness(self, brightness):
        self.brightness = brightness
        dgram = form_datagram(self.address, LED_SET_BRIGHTNESS, brightness, 'char')
        uart.putcs(dgram)

    def set_count(self, count):
        self.count = count
        dgram = form_datagram(self.address, LED_SET_COUNT, count, 'int')
        uart.putcs(dgram)

#GETTERS
    def get_state(self):
        self.state = get_variable(self.address, LED_GET_STATE, 'char')
        return self.state

    def get_brightness(self):
        self.brightness = get_variable(self.address, LED_GET_BRIGHTNESS, 'char')
        return self.brightness

    def get_count(self):
        self.count = get_variable(self.address, LED_GET_COUNT, 'int')
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
        dgram = form_datagram(self.address, DISPLAY_SET_VALUE, value, 'uchar')
        uart.putcs(dgram)

    def set_digit0(self, digit0):
        self.digit0 = digit0
        dgram = form_datagram(self.address, DISPLAY_SET_DIGIT_0, digit0, 'char')
        uart.putcs(dgram)

    def set_digit1(self, digit1):
        self.digit1 = digit1
        dgram = form_datagram(self.address, DISPLAY_SET_DIGIT_1, digit1, 'char')
        uart.putcs(dgram)

    def set_mode(self, mode):
        if mode == 'x':     # hex %02x
            self.mode = 0
        elif mode == 'u':   # decimal %2d
            self.mode = 1
        elif mode == 'd':   # signed decimal %1d
            self.mode = 2
        else:
            print("ERROR: Incompatible Payload Type Defined. (Python)")
            return 0
        dgram = form_datagram(self.address, DISPLAY_SET_MODE, self.mode, 'char')
        uart.putcs(dgram)

#GETTERS
    def get_value(self):
        self.value = get_variable(self.address, DISPLAY_GET_VALUE, 'char')
        return self.value

    def get_digit0(self):
        self.digit0 = get_variable(self.address, DISPLAY_GET_DIGIT_0, 'char')
        return self.digit0

    def get_digit1(self):
        self.digit1 = get_variable(self.address, DISPLAY_GET_DIGIT_1, 'char')
        return self.digit1

    def get_mode(self):
        self.mode = get_variable(self.address, DISPLAY_GET_MODE, 'char')
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
        dgram = form_datagram(self.address, BUTTON_SET_PROGRAM_MODE, program_mode, 'char')
        uart.putcs(dgram)

    def set_pin_mode(self, pin_mode):
        self.pin_mode = pin_mode
        dgram = form_datagram(self.address, BUTTON_SET_PIN_MODE, pin_mode, 'char')
        uart.putcs(dgram)

#GETTERS
    def get_program_mode(self):
        self.program_mode = get_variable(self.address, BUTTON_GET_PROGRAM_MODE, 'char')
        return self.program_mode

    def get_pin_mode(self):
        self.pin_mode = get_variable(self.address, BUTTON_GET_PIN_MODE, 'char')
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
        dgram = form_datagram(self.address, ADC_SET_SCALE, scale, 'float')
        uart.putcs(dgram)

#GETTERS
    def get_scale(self):
        self.scale = get_variable(self.address, ADC_GET_SCALE, 'float')
        return self.scale

    def get_raw(self):
        self.raw = get_variable(self.address, ADC_GET_RAW, 'int')
        return self.raw

    def get_value(self):
        self.value = get_variable(self.address, ADC_GET_READING, 'float')
        return self.value

    def get_all(self):
        self.get_scale()
        self.get_raw()
        self.get_value()

