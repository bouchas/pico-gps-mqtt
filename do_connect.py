from machine import Pin
import network
import time
import rp2
from secrets import secrets

def link_status(i):
    switcher={
        0:'Link Down',
        1:'Link Join',
        2:'Link NoIp',
        3:'Link Up',
        -1:'Link Fail',
        -2:'Link NoNet',
        -3:'Link BadAuth'
    }
    return switcher.get(i,"Invalid error")

def do_connect(ssid=secrets['ssid'],psk=secrets['password']):
    led = Pin("LED", Pin.OUT)

    # Network declaration
    # Set country to avoid possible errors / https://randomnerdtutorials.com/micropython-mqtt-esp32-esp8266/
    rp2.country('CA')

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, psk)

    # Wait for connection with 10 second timeout
    timeout = 10
    while timeout > 0:
        if wlan.status() < 0 or wlan.status() >= 3:
            break
        timeout -= 1
        print('Waiting for connection... ' + link_status(wlan.status()))
        time.sleep(1)

    if wlan.status() != 3:
        print('Wi-Fi connection failed: ' + link_status(wlan.status()))
    else:
        for i in range(wlan.status()):
            led.on()
            time.sleep(.1)
            led.off()
            time.sleep(.2)
        print('Wi-Fi connection succeed: ' + link_status(wlan.status()) + ' ('+wlan.ifconfig()[0]+')')

    return wlan.ifconfig()[0]
