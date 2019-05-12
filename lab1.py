#!/usr/bin/python3
import time
import requests
from Adafruit_IO import Client

aio = Client('MonkeyCanCode', '3c49549385104947bae4e60d5db5a060')

while True:
	print("Ingest data into io.adafruit.com...")
	r = requests.get('https://thingspeak.com/channels/698947/feeds.json?key=PIGU600LAG50NJ3T&results=10')
	for row in r.json()['feeds']:
		feed = aio.feeds('pressure')
		aio.send_data(feed.key, float(row['field1']))
		time.sleep(2)
		feed = aio.feeds('altitude')
		aio.send_data(feed.key, float(row['field2']))
		time.sleep(2)
		feed = aio.feeds('concentration')
		aio.send_data(feed.key, float(row['field3']))
		time.sleep(2)
		feed = aio.feeds('light')
		aio.send_data(feed.key, float(row['field4']))
		time.sleep(2)
		feed = aio.feeds('humidity')
		aio.send_data(feed.key, float(row['field5']))
		time.sleep(2)
		feed = aio.feeds('temp')
		aio.send_data(feed.key, float(row['field6']))
		time.sleep(2)
		feed = aio.feeds('vsig')
		aio.send_data(feed.key, float(row['field7']))
		time.sleep(2)
		feed = aio.feeds('bpm')
		aio.send_data(feed.key, float(row['field8']))
		time.sleep(2)
	print("Ingest data into io.adafruit.com completed")
	time.sleep(60)
