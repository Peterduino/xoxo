###################################################################################
# Xeon rocket data getting script, gets BNO055 and BMP280 datas and provides them #
###################################################################################

# import general librairies
import time
import board
import RPi.GPIO as GPIO

# import components librairies
import adafruit_bmp280
import adafruit_bno055

# import custom functions
from util_calculations import *

def data_reader(shared_data):
    
    # init BMP and BNO
    i2c = board.I2C()
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)
    bno055 = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
    last_val = 0xFFFF

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    #led_pin = 23
    #GPIO.setup(led_pin, GPIO.OUT)
    
    #sea_level_pressure = input("Sea level pressure : ")
    bmp280.sea_level_pressure = 1020.5 #float(sea_level_pressure) if sea_level_pressure else 1020.5

    with open("./datas/datasOthers.txt", "a") as file:
                file.write("\n\n\n\n=======================================================")
                file.write("time,temp,press,alti,accel,magne,gyro,euler,linAc,gravi")


    startTime = time.monotonic()

    datas = {}

    while True:
        
        currentTime = round(time.monotonic() - startTime, 2) # Time lap between lauching the script and now
        
        datas['time'] = currentTime

        try: 
            datas['temp']  = round(bmp280.temperature, 2) if bmp280.temperature is not None else ""
            datas['press'] = round(bmp280.pressure, 2) if bmp280.pressure is not None else ""
            datas['alti']  = round(bmp280.altitude, 2) if bmp280.altitude is not None else ""
        except:
            with open("./logs/others.txt", "a") as file:
                file.write(str(startTime)," : Error reading BMP")
            pass

        try:
            datas['accel'] = (bno055.acceleration if bno055.acceleration is not None else ('!', '!', '!'))
            datas['magne'] = (bno055.magnetic if bno055.magnetic is not None else ('!', '!', '!'))
            datas['gyro']  = (bno055.gyro if bno055.gyro is not None else ('!', '!', '!'))
            datas['euler'] = (bno055.euler if bno055.euler is not None else ('!', '!', '!'))
            datas['linAc'] = (bno055.linear_acceleration if bno055.linear_acceleration is not None else ('!', '!', '!'))
            datas['gravi'] = (bno055.gravity if bno055.gravity is not None else ('!', '!', '!'))
        except:
            with open("./logs/others.txt", "a") as file:
                file.write(str(startTime)," : Error reading BNO")
            pass

        try:
            shared_data['data'] = datas
        except:
            with open("./logs/others.txt", "a") as file:
                file.write(str(startTime)," : Error writing datas on shared variables")
        
        with open("./datas/datasOthers.txt", "a") as file:
            file.write("\n{},{},{},{},{},{},{},{},{},{}".format(*datas))
