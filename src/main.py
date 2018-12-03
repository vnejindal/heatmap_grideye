"""
Entry File for connection Manager
"""
import sys
import plotly 
#import plotly.plotly as py
import plotly.graph_objs as go

import paho.mqtt.client as mqtt
import json 

# Code snippet from: https://www.hackster.io/64742/people-counting-with-helium-grid-eye-and-raspberry-pi-49f601

import numpy as np
import math
import time
from matplotlib import pyplot as plt
#from Adafruit_AMG88xx import Adafruit_AMG88xx
from scipy.interpolate import griddata
import cv2


mqtt_rc_codes = {
                    0: 'Connection successful',
                    1: 'Connection refused - incorrect protocol version',
                    2: 'Connection refused - invalid client identifier',
                    3: 'Connection refused - server unavailable',
                    4: 'Connection refused - bad username or password',
                    5: 'Connection refused - not authorised'
                }

g_mqtt_info = {}
g_mqtt_info['broker_ip'] = '18.224.198.60'
g_mqtt_info['broker_port'] = 1883
g_mqtt_info['mqtt_topic'] =  'demosfp/iot83/sfp/325749705066801400640038'
#g_mqtt_info['mqtt_topic'] =  'demosfp/iot83/sfp/324948705371801900400070'


######## MQTT Handling ###########
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    global mqtt_rc_codes
    global g_mqtt_info
    if rc == 0:
        print 'Client connected successfully'
        client.subscribe(g_mqtt_info['mqtt_topic'])
        print 'Subscription successful. Topic: ', g_mqtt_info['mqtt_topic']
    else:
        print 'Client connection failed: ', str(rc), mqtt_rc_codes[rc]
    

def on_disconnect(client, userdata, rc):
    if rc == 0:
        print 'Client disconnected successfully'
    else:
        print 'Client disconnection issue: ', str(rc)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    #print("vne:: "+msg.topic+" "+str(msg.payload))
    try:
        process_mqtt_msg(msg.payload)
    except Exception as e:
        print(e)
        print 'Invalid Json: ', msg.payload
        
    #exit()
        
def on_publish(client, userdata, mid):
    print("mid: "+str(mid))
    pass

def start_mqtt():
    """
    Connects with MQTT broker
    """
    global g_mqtt_info
    srv_ip = g_mqtt_info['broker_ip']
    srv_port = g_mqtt_info['broker_port']
    srv_keepalive = 1
    
    print 'connecting to broker:', srv_ip,':', srv_port, ' ', srv_keepalive
    
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.on_publish = on_publish
    mqtt_client.on_disconnect = on_disconnect
    
    mqtt_client.connect(srv_ip, srv_port, srv_keepalive)

    print 'connection to broker successful'  
    
    mqtt_client.loop_forever()
    #client.loop_start()
    #dpublish.read_device_data('temperature', '1', client)
    
def process_mqtt_msg(msg_payload):
    """
    Decodes MQTT Json 
    """
    jdata = msg_payload.decode('utf-8')
    json_payload =  json.loads(jdata)
    
    amg_data = json_payload["sensors"]["AMG8833"] 
    jdata = amg_data.decode('utf-8')
    
    amg_json = json.loads(jdata)
    #print amg_json["0_16"]
    get_heatmap(amg_json)

def get_heatmap(amg_json):
    """
    Uses plotly to get AMG 8x8 heatmap
    """
    #print type(amg_json["0_16"])
    
    amg_indexes = ["0_16", "16_32", "32_48", "48_64"]
    
    amg_list = []
    for aindex in amg_indexes:
        #print amg_json[aindex]
        amg_list += list(split_amg_data(amg_json[aindex], 8))
    
    #print amg_list
    #plot_grideye_data(amg_list)
    get_people_count(amg_list)

def get_people_count(amg_list):
    """
    Get people count using grid-eye data
    """
    print 'Calculating People Count...'
    
    pixels = []
    
    for count in range(0,8):
        pixels += amg_list[count]
        
    #print pixels 
   
    pixmax = max(pixels)
    pixels = [x / pixmax for x in pixels]
    points = [(math.floor(ix / 8), (ix % 8)) for ix in range(0, 64)]
    grid_x, grid_y = np.mgrid[0:7:32j, 0:7:32j]
    
    #print grid_x 
    #print grid_y
    
    # bicubic interpolation of 8x8 grid to make a 32x32 grid
    bicubic = griddata(points, pixels, (grid_x, grid_y), method='cubic')
    image = np.array(bicubic)
    image = np.reshape(image, (32, 32))
    #print(image)
    plt.imsave('color_img.jpg', image)
   
      
    # Read image
    img = cv2.imread("color_img.jpg", cv2.IMREAD_GRAYSCALE)
    img = cv2.bitwise_not(img)

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 255

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 5

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1

    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.87

    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.01

    # Set up the detector with default parameters.
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    keypoints = detector.detect(img)
    for i in range (0, len(keypoints)):
        x = keypoints[i].pt[0]
        y = keypoints[i].pt[1]
        print(x, y)
            
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
    # Show keypoints
    cv2.imshow("Keypoints", im_with_keypoints)
    cv2.waitKey(0)
    
def plot_grideye_data(amg_list):
    """
    Function to plot the AMG grid eye data using plotly APIs
    """
    trace = go.Heatmap(z=amg_list)
    data=[trace]
    plotly.offline.plot(data, filename='amg-heatmap')

def split_amg_data(amg_data, chunk_size):
    for i in range(0, len(amg_data),chunk_size):
            yield amg_data[i:i+chunk_size]

def main():    
    print 'program started...'
    print 'plotly version: ' + plotly.__version__
    
    start_mqtt()
    
    '''
    trace = go.Heatmap(z=[[1, 20, 30],
                      [20, 1, 60],
                      [30, 60, 1]])
    data=[trace]
    plotly.offline.plot(data, filename='basic-heatmap')
    '''
    
    print 'program end...'

if __name__ == '__main__':
    main()
