################################################################################
# Xeon rocket parafoil control script, gets all datas and control the parafoil #
################################################################################

# import general librairies
import time
import RPi.GPIO as GPIO
from util_calculations import pointIsGood

def switchOffLeds(leds):
    for led in leds:
        GPIO.output(led,GPIO.LOW)

def pilot(shared_data):

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    leds = [20,21,16,12]

    for led in leds:
        GPIO.setup(led,GPIO.OUT)

    while True:
        try:
            gyroAltiDatas = shared_data.get('data', 0)
            gpsDatas = shared_data.get('gpsDatas', 0)
            nvSecu = shared_data.get('nvSecu', 0)
        except:
            with open("./logs/pilot.txt", "a") as file: 
                file.write(str(startTime)," : Error reading datas on shared values")
        try:
            point_inside = pointIsGood(gpsDatas[1],gpsDatas[2])
            if point_inside==True:
                switchOffLeds()
                GPIO.output(21, GPIO.HIGH)
                print("GOOD")
            else:
                switchOffLeds()
                GPIO.output(21)
                print("OUT")
        except:
            print("NO FIX")
        

        print(f"PILOT: \n data : {gyroAltiDatas}, gps_data : {gpsDatas}, nvSecu : {nvSecu}")
        time.sleep(1)

