from machine import Pin, UART, I2C
#ref.: https://www.youtube.com/watch?v=y2EDhzDiPTE
#ref.: https://github.com/ahmadlogs/rpi-pico-upy/tree/main/gps2-rpi-pico

#Import utime library to implement delay
import utime, time
import math
import network

#________________________________________________________
from ssd1306 import SSD1306_I2C
#https://github.com/stlehmann/micropython-ssd1306
#________________________________________________________
from micropyGPS import MicropyGPS
#https://github.com/inmcm/micropyGPS
#________________________________________________________

from secrets import secrets

#________________________________________________________
from umqtt.simple import MQTTClient
#https://github.com/danjperron/PicoWMqttDs18b20/blob/main/main.py
#https://www.hivemq.com/blog/iot-reading-sensor-data-raspberry-pi-pico-w-micropython-mqtt-node-red/
#https://github.com/thonny/thonny/issues/2524

##########################################################
#Oled I2C connection
i2c=I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)
##########################################################

##########################################################
#GPS Module UART Connection
gps_module = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
##########################################################


##########################################################
TIMEZONE = -5
my_gps = MicropyGPS(TIMEZONE)
##########################################################

previousLatitude = 0
previousLongitude = 0

def degreesToRadians(degrees):
    return degrees * math.pi / 180

# Calculate the Distance between two coordinates
# https://stackoverflow.com/questions/365826/calculate-distance-between-2-gps-coordinates
def distanceInMeterBetweenEarthCoordinates(lat1, lon1, lat2, lon2):
    earthRadiusMeter = 6371000;
    dLat = degreesToRadians(lat2-lat1);
    dLon = degreesToRadians(lon2-lon1);

    lat1 = degreesToRadians(lat1)
    lat2 = degreesToRadians(lat2)
    
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.sin(dLon/2) * math.sin(dLon/2) * math.cos(lat1) * math.cos(lat2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return earthRadiusMeter * c


##########################################################
def convert(parts):
    if (parts[0] == 0):
        return None
        
    data = parts[0]+(parts[1]/60.0)
    # parts[2] contain 'E' or 'W' or 'N' or 'S'
    if (parts[2] == 'S'):
        data = -data
    if (parts[2] == 'W'):
        data = -data

    data = '{0:.6f}'.format(data) # to 6 decimal places
    return str(data)
##########################################################

##########################################################
#MQTT  Topic Setup
def connectMQTT():
    #https://forum.micropython.org/viewtopic.php?t=11076&p=60836
    client = MQTTClient(secrets['client_id'],secrets['mqtt_server'], keepalive=30) # Just added Keepalive=30 to fix the error code 2
    client.connect()
    return client

def publish(topic, value):
    # print(topic)
    # print(value)
    # pub_msg = "%5.2f" % value
    print("Publish: ",topic,"  ",value)
    # client.publish(topic, pub_msg)
    client.publish(topic, value)
    #print("publish Done")


#network declaration
# Set country to avoid possible errors / https://randomnerdtutorials.com/micropython-mqtt-esp32-esp8266/
rp2.country('CA')

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

#connect using ssid
wlan.connect(secrets['ssid'],secrets['password'])
while not wlan.isconnected():
    #machine.idle() # save power while waiting
    print('Waiting for connection...')
    utime.sleep(1.0)
ip = wlan.ifconfig()[0]
print(f'IP Address: {ip}')

oled.fill(0)
oled.text("IP Address", 0, 0)
oled.text(ip, 0, 16)
oled.rotate(True)
oled.show()

# utime.sleep(1)
#
# oled.poweroff()
# 
# oled.fill(0)
# oled.text("IP Address", 0, 0)
# oled.text(ip, 0, 16)
# oled.rotate(True)
# oled.show()
# 
# utime.sleep(2)
# 
# oled.poweron()


# # Wait for connection with 10 second timeout
# timeout = 10
# while timeout > 0:
#     if wlan.status() < 0 or wlan.status() >= 3:
#         break
#     timeout -= 1
#     print('Waiting for connection...')
#     time.sleep(1)
#     
# # Handle connection error
# # Error meanings
# # 0  Link Down
# # 1  Link Join
# # 2  Link NoIp
# # 3  Link Up
# # -1 Link Fail
# # -2 Link NoNet
# # -3 Link BadAuth
# 
# if wlan.status() != 3:
#     raise RuntimeError('Wi-Fi connection failed')
# else:
#     for i in range(wlan.status()):
#         led.on()
#         time.sleep(.1)
#         led.off()
#     print('Connected')
#     status = wlan.ifconfig()
#     print('ip = ' + status[0])
    




# client = connectMQTT()

try:
    client = connectMQTT()
except OSError as e:
    machine.reset()
  
# Test the distance function
print('Distance between same points: '+str(distanceInMeterBetweenEarthCoordinates(0,0,0,0)))
print('Distance London to Arlington: '+str(distanceInMeterBetweenEarthCoordinates(51.5, 0, 38.8, -77.1)))

print(distanceInMeterBetweenEarthCoordinates(45.3989, -73.4823, 45.3999, -73.4833))


##########################################################
try:
    while True:
        #_________________________________________________
        #print(i2c.scan())
        length = gps_module.any()
        if length>0:
            b = gps_module.read(length)
            for x in b:
                msg = my_gps.update(chr(x))
        #_________________________________________________
        latitude = convert(my_gps.latitude)
        longitude = convert(my_gps.longitude)
        #_________________________________________________
        if (latitude == None and latitude == None):
            oled.fill(0)
            oled.text("No Data", 0, 0)
            oled.show()
            continue
        #_________________________________________________
        t = my_gps.timestamp
        #t[0] => hours : t[1] => minutes : t[2] => seconds
        gpsTime = '{:02d}:{:02d}:{:02}'.format(t[0], t[1], t[2])
        
        gpsdate = my_gps.date_string('s_ymd')
        speed = my_gps.speed_string('kph') #'kph' or 'mph' or 'knot'
        distance = distanceInMeterBetweenEarthCoordinates(float(latitude), float(longitude), float(previousLatitude), float(previousLongitude))
        #_________________________________________________
#         print('Lat:', latitude)
#         print('Lng:', longitude)
#         print('time:', gpsTime)
#         print('Date:', gpsdate)
#         print('speed:', speed)
#         print('Distance:'+ str(distance))
        print(gpsdate+'@'+gpsTime+': '+latitude+', '+longitude+', '+str(distance))
        #_________________________________________________
        oled.fill(0)
        oled.text('Lat:'+ latitude, 0, 0)
        oled.text('Lng:'+ longitude, 0, 12)
#         oled.text('Speed:'+ speed, 0, 24)
        oled.text('Time:'+ gpsTime, 0, 24)
        oled.text('Date:'+ gpsdate, 0, 36)
        oled.text('Dist:'+ str(distance), 0, 48)
        oled.show()
        #_________________________________________________

        if distance > 2:
            previousLatitude = latitude
            previousLongitude = longitude
            if distance > 1000: distance = 0
            publish('gps/data','{"lat":'+latitude+',"lon":'+longitude+',"d":'+str(distance)+'}')

        utime.sleep(1.0)
        
except OSError as e:
    machine.reset()

finally:
    oled.fill(0)
    oled.show()
 
