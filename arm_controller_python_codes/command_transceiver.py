import threading
import serial
import time
import paho.mqtt.client as mqtt
connected = False
port = '/dev/ttyACM0'
baud = 115200


host="localhost"
port_mqtt=1883
subscription_topic="command"
feedback_publisher="sensors"


serial_port = serial.Serial(port, baud, timeout=10)
time.sleep(3)
connected=True


value_at_max_angle=[288,492,576,189]
value_at_min_angle=[247,85 ,204,266]
max_angle=[180,90,90,180]

angle_array=[]





def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(subscription_topic)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global serial_port
    lst_cmd=msg.payload.decode()
    lst_cmd=[int(i) for i in lst_cmd.split(",")]
    print(lst_cmd)
    print(bytearray(lst_cmd))
    serial_port.write(bytearray(lst_cmd))
    


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(host, port_mqtt, 60)


def map_var(current_var,fromlow,fromhigh,tolow,tohigh):
    return int((((tohigh-tolow)/(fromhigh-fromlow))*(current_var-fromlow))+tolow)


def handle_data(data):
    global angle_array
    global client
    if len(data)>5:
        angle_array=data.split(",")
        angle_string=",".join(angle_array)[:-2]
        #print(angle_string.split(",")[:-2])
        angle_output=[]
        #print(angle_string)
        for i,raw_value in enumerate(angle_string.split(",")[:-1]):
            outval=str(map_var(int(raw_value),value_at_min_angle[i],value_at_max_angle[i],0,max_angle[i]))
            #print(outval)
            angle_output.append(outval)
        #print(angle_output)
        client.publish(feedback_publisher,",".join(angle_output))

def read_from_port(ser):
    global connected
    while True:
        if connected==True:
        #print("test")
            try:
                reading = ser.readline().decode()
                handle_data(reading)
            except:
                pass

thread = threading.Thread(target=read_from_port, args=(serial_port,))
thread.start()

client.loop_forever()

#while True:
#    lst_cmd=bytearray([100,70,30,100,1,200])
#    print("current command",lst_cmd)
#    serial_port.write(lst_cmd)
#    time.sleep(10)