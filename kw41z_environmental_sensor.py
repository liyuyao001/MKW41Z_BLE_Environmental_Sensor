#coding=utf-8 
#!/usr/bin/env python 
#!/usr/bin/python

from __future__ import print_function 
import sys 
import binascii 
import bluepy.btle 
import os 

import pycurl, json, time 
import RPi.GPIO as GPIO 
from StringIO import StringIO 

import struct 
from bluepy.btle import UUID, Peripheral

ble_mac = "00:60:37:00:00:07"
humidity_uuid = UUID(0x2a19)
temp_chara_handle = 0x0018
temp_chara_uuid = 0x2a1e

##如果探测到的温湿度之一超过了设定的阈值，则通过Instapush向手机推送通知
TEMP_THRESHOLD = 30 
HUMIDITY_THRESHOLD = 70
#temperature_val = 0 
#humidity_val = 0
##下面的代码负责设置BLE连接好之后接收Notifications
ble_conn = None 

class MyDelegate(bluepy.btle.DefaultDelegate):
    def __init__(self, conn):
        bluepy.btle.DefaultDelegate.__init__(self)
        self.conn = conn

    def handleNotification(self, cHandle, data):
        global temperature_val
        data = binascii.b2a_hex(data)
        #print("Notification:", str(cHandle), " data ", data)
        temperature_str = data[2:4]
	#update the temperature value
        temperature_val = int(temperature_str,16)
        print("Temperature is ", temperature_val," deg C")
        #print "Temperature is %d" %temperature_val + " deg C"

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            pass
        elif isNewData:
            print("\\nDiscovery:", "MAC:", dev.addr, " Rssi ", str(dev.rssi)) 

def ble_connect(devAddr):
    global ble_conn
    if not devAddr is None and ble_conn is None:
        ble_conn = bluepy.btle.Peripheral(devAddr, bluepy.btle.ADDR_TYPE_PUBLIC)
        ble_conn.setDelegate(MyDelegate(ble_conn))
        print("connected") 

def ble_disconnect():
    global ble_conn
    ble_conn = None
    print("disconnected")

##下面的代码负责读取湿度值
 
def read_humidity():
    global humidity_val
    p = Peripheral(ble_mac, "public")
    ch = p.getCharacteristics(uuid=humidity_uuid)[0]
    if (ch.supportsRead()):
        humidity_val = binascii.b2a_hex(ch.read())
        humidity_val = int(humidity_val, base=16)
        humidity_str = str(humidity_val)
        print("Humidity is ", humidity_val," %")
        #print "humidity = " + humidity_str + " %"
        time.sleep(1) 

##下面的代码负责向InstaPush发送
##填写Application ID 和 Application Secret fields 这两个字段的内容
appID = "597360dca4c48a0c9bde80b3" 
appSecret = "ee81bae04c9fab314a39e7229237bc9d"
#填写Event Title中的内容
pushEvent = "neawreading"

#向手机推送的消息内容
def push_notification():
    pushMessage = "Temperature: " + str(temperature_val) + " deg C\nHumidity is: " + str(humidity_val) +" %"
    c = pycurl.Curl() 
    c.setopt(c.URL,'https://api.instapush.im/v1/post') 
    c.setopt(c.HTTPHEADER, ['x-instapush-appid:' + appID, 'x-instapush-appsecret:' + appSecret, 'Content-Type: application/json']) 

    json_fields = {} 
    json_fields['event'] = pushEvent 
    json_fields['trackers'] = {} 
    json_fields['trackers']['message'] = pushMessage 
    postfields = json.dumps(json_fields) 
    c.setopt(c.POSTFIELDS, postfields)
    c.perform()

def daemon_loop():
    flag_notify = False
    read_humidity()
 # scan
#    scanner = bluepy.btle.Scanner().withDelegate(MyDelegate(None)) timeout = 
#    10.0 devices = scanner.scan(timeout) for dev in devices:
#        if dev.addr == ble_mac:
#            print("\\nDiscovery:", "MAC:", dev.addr, " Rssi ", str(dev.rssi)) 
#            for (adtype, desc, value) in dev.getScanData():
#                print (" %s(0x%x) = %s" % (desc, int(adtype), value)) break
    # connect
    ble_connect(ble_mac)
    # write , set listen
#    snd_content_str = """\\x01\\x00""" ble_conn.writeCharacteristic(handle, 
#    snd_content_str)
    setup_data = b"\x01\x00"
    notify =ble_conn.getCharacteristics(uuid=temp_chara_uuid)[0]
    notify_handle = notify.getHandle() + 1
    ble_conn.writeCharacteristic(notify_handle, setup_data, withResponse=True)
    # wait notification
    ble_conn.waitForNotifications(10.0)

    #if there is no pending notification
    if flag_notify == False:
        if temperature_val > TEMP_THRESHOLD or humidity_val > HUMIDITY_THRESHOLD:
            flag_notify = True
            c.perform()
            time.sleep(3)
            flag_notify = False
            
    # disconnect
    ble_disconnect()
	
if __name__ == '__main__':

    ble_connect(ble_mac)
    while True: 
        flag_notify = False
                
        #read_humidity()
        ch = ble_conn.getCharacteristics(uuid=humidity_uuid)[0]
        if (ch.supportsRead()):
            humidity_val = binascii.b2a_hex(ch.read())
            humidity_val = int(humidity_val, base=16)
            humidity_str = str(humidity_val)
            print("Humidity is ", humidity_val," %")
            #print "humidity = " + humidity_str + " %"
            #time.sleep(1) 
        
        setup_data = b"\x01\x00"
        notify =ble_conn.getCharacteristics(uuid=temp_chara_uuid)[0]
        notify_handle = notify.getHandle() + 1
        ble_conn.writeCharacteristic(notify_handle, setup_data, withResponse=True)
        # wait notification
        ble_conn.waitForNotifications(1.0)
        
        #if there is no pending notification
        #if flag_notify == False:
        if temperature_val > TEMP_THRESHOLD or humidity_val > HUMIDITY_THRESHOLD:
            #flag_notify = True
            push_notification()
            time.sleep(5) 
            #flag_notify = False
                
        # disconnect
        #ble_disconnect()
        #time.sleep(3) 
        	

