##############################################################################
# Xeon rocket gps datas getting script, gets NEO-M8Q datas and provides them #
##############################################################################

# import general librairies
import time
import board

# import component librairies
import serial
from serial.serialutil import SerialException

# import custom functions
from util_calculations import *

# define functions for the GPS
def testNMEA(sentence,strFiltre):
    """Returns True or False depending on validity of sentence:str depending on some factors"""
    if sentence.startswith(strFiltre) and len(listNMEA(sentence))==13 and listNMEA(sentence)[3]!=None and listNMEA(sentence)[5]!=None and listNMEA(sentence)[3]!='' and listNMEA(sentence)[5]!='':
        return True
    return False

def waitUntilNMEA(uart,strFiltre):
    """Returns either an NMEA sentence:str (when provided) or an error message:str if not convinient"""
    while True:
        try :
            sentence = uart.readline().decode("utf-8", errors="ignore").strip()
            # now return it only if correct
            if testNMEA(sentence,strFiltre):
                return sentence
            elif (sentence.startswith('$GNRMC')) and len(listNMEA(sentence))!=13:
                return "! TRAM ERROR"
            elif (sentence.startswith('$GNRMC')) and listNMEA(sentence)[3]!='':
                return "! NO GPS FIX"
            
        except KeyboardInterrupt:
            break
        except:
            uart.close()
            uart.open()

def datasGPS(sentence):
    """Returns the 3-uple (NMEA sentence:str, latitude:str, longitude:str)"""
    if sentence.startswith("!"):
        return "!", "!", "!"
    else:
        return sentence, str(latOf(sentence)), str(longOf(sentence)),

def send_ubx_message(serial_port, msg):
    serial_port.write(bytearray(msg))
    time.sleep(0.1)

def set_update_rate(serial_port, rate_ms):
    msg = [
        0xB5, 0x62,         # UBX header
        0x06, 0x08,         # CFG-RATE class and ID
        0x06, 0x00,         # Length of payload
        rate_ms & 0xFF, (rate_ms >> 8) & 0xFF,  # measRate in ms
        0x01, 0x00,         # navRate (fixed to 1)
        0x00, 0x00          # timeRef (0 = UTC, 1 = GPS time)
    ]
    ck_a = 0
    ck_b = 0
    for i in range(2, len(msg)):
        ck_a = (ck_a + msg[i]) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    msg.append(ck_a)
    msg.append(ck_b)
    
    send_ubx_message(serial_port, msg)


# the code
def gps_reader(shared_data):
    """The whole code"""

    # Init GPS module
    uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1)
    set_update_rate(ser, update_rate_ms) # Increase the acquisition rate
    typeNMEAfilter = '$GNRMC' # type of the NMEA used : '$' for all, '$G' for GPS and GLONASS, '$GNRMC' for the most relayablility

    with open("./datas/datasGPS.txt", "a") as file:
        file.write('\n\n\n\n ===================================================================')
        file.write('')

    startTime = time.monotonic()

    while True:
        
        try:
            currentTime = round(time.monotonic() - startTime, 2) # Time lap between lauching the script and now
            #currentNMEA = waitUntilNMEA(uart,typeNMEAfilter)

            if ser.in_waiting:
                    data = ser.readline().decode('ascii', errors='ignore').strip()
                    if testNMEA(data):
                        currentNMEA = data
        
                        datas = {
                            'time':currentTime,
                            'NMEA':datasGPS(currentNMEA)[0],
                            'lattD':datasGPS(currentNMEA)[1],
                            'longD':datasGPS(currentNMEA)[2],
                        }
                        
                        string = "{},{},{},{}".format(*datas)

        except:
            with open("./logs/gps.txt", "a") as file: 
                file.write(str(startTime)," : Error reading datas from GPS")
            continue
        
        try:
            shared_data['gpsDatas'] = datas
        except:
            with open("./logs/gps.txt", "a") as file: 
                file.write(str(startTime)," : Error writing datas on shared variables")
            continue
        
        with open("./datas/datasGPS.txt", "a") as file: 
            file.write(string)
