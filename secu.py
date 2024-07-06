#############################################################################
# Xeon rocket security checker, gets sensors datas and provides Secu. Level #
#############################################################################

# import general librairies
import time
from random import randint

def secu(shared_data):

    while True:

        gyroAltiDatas = shared_data.get('data', 0)  # as datas = {'time','temp','press','alti','accel','magne','gyro','euler','linAc','gravi'}
        gpsDatas = shared_data.get('gps_data', 0)   # as datas = {'time','NMEA','lattD','longD'}

        shared_data['nvSecu'] = randint(0,10)

        time.sleep(2)

