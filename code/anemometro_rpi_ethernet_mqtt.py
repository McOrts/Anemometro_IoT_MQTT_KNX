#!/usr/bin/python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import smtplib
import datetime
import time
import MySQLdb as mdb
import logging
import os
import fnmatch
import math
import json
from pprint import pprint
import sys
import paho.mqtt.client as mqtt

# To avoid the error on rc.local execution envirotment: 
#WARNING urllib3.connectionpool Retrying (Retry(total=2, connect=None, read=None, redirect=None, status=None)) after connection broken by 'NewConnectionError('<urllib3.connection
logging.getLogger("urllib3").setLevel(logging.ERROR)

# Then the code sets up the logging module. We are going to use the basicConfig() function to set up the default handler 
# so that any debug messages are written to the file /home/pi/event_error.log.
logging.basicConfig(filename='/home/pi/wind_event_error.log',
  level=logging.DEBUG,
  format='%(asctime)s %(levelname)s %(name)s %(message)s')
logger=logging.getLogger(__name__)

# Read the configuration file
DEFAULT_CONFIG_PATH = '/home/pi/config.json'
with open(DEFAULT_CONFIG_PATH, 'r') as config_file:
  config = json.load(config_file)

# anemometer parameters
count = 0
radius = 9.0
comp_factor = 2.3
interval = 60
speed = 0

# Function for storing readings into MySQL
def insertDB(speed):
  try:
    con = mdb.connect(config['db_server_ip'],
                      config['db_update_user'],
                      config['db_update_password'],
                      'measurements');
    cursor = con.cursor()
    sql = "INSERT INTO wind(speed) VALUES ('%s')" % (speed)
    cursor.execute(sql)
    sql = []
    con.commit()
    con.close()
  except mdb.Error as e:
    logger.error(e)

# Function for send an email notifing the speed reach the maximum 
def send_email_wind_speed(speed):
    # Specifying the from and to addresses
    toaddr = config['mail']['toaddr']
    cc = config['mail']['cc']
    fromaddr = config['mail']['fromaddr']
    message_subject = "[WIND] Home wind speed alert"
    message_text = "The wind speed is (Km/h): %s" % speed
    message = "From: %s\r\n" % fromaddr + "To: %s\r\n" % toaddr + "Subject: %s\r\n" % message_subject + "\r\n"  + message_text
    toaddrs = [toaddr] 
    username = '¿?@gmail.com'
    password = '¿?'
    # Sending the mail  
    try:
      server = smtplib.SMTP_SSL("smtp.gmail.com", 465)
      server.ehlo()
      server.login(config['mail']['username'],config['mail']['password'])
      # ssl server doesn't support or need tls, so don't call server.starttls() 
      server.sendmail(fromaddr, toaddrs, message)
      server.close()
    except:
      print ("failed to send mail")

# Function for calculate wind speed
def calculate_speed(r_cm, factor, time_sec):
    global count
    circ_cm = (2 * math.pi) * r_cm
    rot = count / 2.0
    dist_km = (circ_cm * rot) / 100000.0 # convert to kilometres
    km_per_sec = dist_km / time_sec
    km_per_hour = km_per_sec * 3600 # convert to distance per hour
    km_per_hour = km_per_hour * factor
    km_per_hour = round(km_per_hour,2)
    return km_per_hour

def spin(channel):
    global count
    count += 1

# Function for publish MQTT messages
def publish_MQTT_messages (p_event):
  try:
    client = mqtt.Client()
    # compose the message 
    topic = config['domohome_wind']['topic']
    payload = '{:3.2f}'.format(p_event / 1.)
    client.connect(config['MQTT']['broker_server_ip'],1883,60)
    client.publish (topic, payload)

    client.disconnect();
  except Exception as e:
    print ("exception")
    logger.error("Exception MQTT sending message: %s\n"+" "+e.__str__())

GPIO.setmode(GPIO.BCM)
GPIO.setup(config['domohome_wind']['gpio_pin'], GPIO.IN, GPIO.PUD_UP)
GPIO.add_event_detect(config['domohome_wind']['gpio_pin'], GPIO.FALLING, callback=spin)

while True:
    count = 0
    time.sleep(interval)
    speed = calculate_speed(radius, comp_factor, interval)
#    insertDB (speed)
    publish_MQTT_messages(speed)
    if speed > config['domohome_wind']['max_warning']:
        send_email_wind_speed(speed) 
