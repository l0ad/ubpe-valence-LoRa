# MQTT client for TTN applications device uplink messages retrieval
# Dump all device uplink messages in Json files
# see https://www.thethingsindustries.com/docs/integrations/mqtt/
# Authentication information must be filled in ttn-mqtt-auth.cfg file, whose format must be:
# [Auth]
# ttn-client-id = <ttn-client id, must be anything>
# ttn-login = <application>@ttn
# ttn-password = <generated API key>

import paho.mqtt.client as mqtt
import configparser as config
from datetime import datetime
import json


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    if rc != 0:
        print("connection not established, exiting...")
        exit(1)
    print("connection established, waiting for devices uplink data...")
    client.subscribe('v3/+/devices/+/up')


def save_payload(payload):
    now = datetime.now()
    current_time = now.strftime("%d-%m-%y_%Hh%M")
    path = current_time + ".json"
    json_object = json.loads(payload)
    json_formatted_str = json.dumps(json_object, indent=2)
    print(json_formatted_str)
    print("saving in " + path)
    with open(path, 'w') as dump_file:
        dump_file.write(json_formatted_str+"\n")
        dump_file.close()


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):

    print(datetime.now().strftime("%d-%m-%y_%Hh%Mm%Ss"), end=" ")
    save_payload(msg.payload.decode('utf-8'))
    print("[saved]")


config = config.ConfigParser()
config.read_file(open(r'ttn-mqtt-auth.cfg'))
ttn_client_id = config.get('Auth', 'ttn-client-id')
ttn_login = config.get('Auth', 'ttn-login')
ttn_password = config.get('Auth', 'ttn-password')

client = mqtt.Client(ttn_client_id)
client.username_pw_set(ttn_login, ttn_password)
client.on_connect = on_connect
client.on_message = on_message
print("TTN device uplink dumper started!")
client.connect("eu1.cloud.thethings.network", 1883, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()
