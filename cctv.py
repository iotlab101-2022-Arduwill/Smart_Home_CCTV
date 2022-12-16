import cv2
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import datetime


MQTT_SERVER = "54.147.58.115"
MQTT_PATH = "image"

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(MQTT_PATH)
    # The callback for when a PUBLISH message is received from the server.


def on_message(client, userdata, msg):
    # more callbacks, etc
    # Create a file with write byte permission
    
    
   
        a = datetime.datetime.now()
        date = str(datetime.datetime.now())
        date = date[:19]
        
        camera = cv2.VideoCapture(-1)
        camera.set(3, 640)
        camera.set(4, 480)


        _, image = camera.read()

        cv2.imwrite(date+".jpg" ,image)

        f=open(date+".jpg", "rb") #3.7kiB in same folder
        fileContent = f.read()
        byteArr = bytearray(fileContent)


        publish.single(MQTT_PATH, byteArr, hostname=MQTT_SERVER)
        print("image send!" + date+".jpg")
        
    
    
    

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_SERVER, 1883, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()
