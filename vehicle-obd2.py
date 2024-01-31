import can
import paho.mqtt.client as paho
import json
import time
import struct
import serial
import pynmea2
import os
import logging
from   logging.handlers import RotatingFileHandler

def on_connect(client, userdata, flags, rc):
    if (rc == 0):
        logger.debug('MQTT connected successfully')
    else:
        logger.debug("MQTT did not successfully connect.  Return code=%d", rc)

def on_disconnect(client, userdata, rc):
    logger.debug('MQTT disconnected. Reconnecting...')

def ReqPID (channel, PID):
    reqTimeout = 3

    logger.debug("Making CAN request for PID: %d (decimal)", PID)
    msg = can.Message(arbitration_id=0x7DF, data=[2,1,PID,170,170,170,170,170], is_extended_id=False)
    channel.send(msg)

    sendTime = time.time()

    while ( (time.time() - sendTime) < (reqTimeout) ):
        msg = can.Message()
        msg = channel.recv(timeout=reqTimeout)
        if (msg):
            if (msg.arbitration_id == 2024 and msg.data[1]==65 and msg.data[2] == PID):
                logger.debug('Received matching CAN PID')
                return msg
        else:
            logger.debug('CAN bus seems inactive (not receiving anything)')
    logger.debug('CAN bus giving up on request')
    return None

def getSupported (bus,PID):
    supportedPID = '0' * 32

    message = ReqPID(bus,PID)
    if (message is not None):
        tempArray = bytearray([message.data[3],message.data[4],message.data[5],message.data[6]])
        tempBinary = ''

        for x in tempArray:
            tempBinary += bin(x)[2:].zfill(8)

        supportedPID = tempBinary

    return supportedPID

def binToHex (message):
    tempHex = "0x" + format(int(message, 2), '0>8x')
    return tempHex

def ReqLocation (ser):
    reqTimeout = 3
    tempPayload = {}

    ser.flushInput()

    logger.debug('Looking for location information')

    startTime = time.time()
    while ((time.time() - startTime) < (reqTimeout)):
        line = ser.readline().decode('utf-8')

        if line.startswith('$'):
            try:
                msg = pynmea2.parse(line)
                if msg.sentence_type == 'GGA':
                    lat = msg.latitude
                    lon = msg.longitude
                    num_sats = msg.num_sats
                    altitude = msg.altitude
                    altitude_units = msg.altitude_units
                    horizontal_dil = msg.horizontal_dil
                    gps_qual = msg.gps_qual
                    logger.debug(msg)
                    if (gps_qual != 0):
                        tempPayload = {'lat': lat, 'lon': lon, 'num_sats': num_sats, 'altitude': altitude, 'altitude_units': altitude_units, 'horizontal_dil': horizontal_dil, 'gps_qual': gps_qual}
                    else:
                        tempPayload = {'gps_qual': gps_qual, 'num_sats': num_sats}
                    logger.debug('Received NMEA stream location data')
                    return tempPayload
            except:
                    logger.debug('NMEA stream parse error')
    logger.debug('Failed to receive location data; giving up')
    return None

#Start of main code block
IR_GPS = os.getenv('IR_GPS')

MQTT_BROKER = os.getenv('MQTT_BROKER')
MQTT_PORT = int(os.getenv('MQTT_PORT'))
MQTT_USERNAME = os.getenv('MQTT_USERNAME')
MQTT_PASSWORD = os.getenv('MQTT_PASSWORD')
MQTT_USE_TLS = int(os.getenv('MQTT_USE_TLS'))
MQTT_BASE_TOPIC = os.getenv('MQTT_BASE_TOPIC')
MQTT_QOS = int(os.getenv('MQTT_QOS'))

LOCATION_DIV = int(os.getenv('LOCATION_DIV'))
LOOP_INTERVAL = int(os.getenv('LOOP_INTERVAL'))
VEHICLE_SPEED_DIV = int(os.getenv('VEHICLE_SPEED_DIV'))
ENGINE_RPM_DIV = int(os.getenv('ENGINE_RPM_DIV'))
FUEL_LEVEL_DIV = int(os.getenv('FUEL_LEVEL_DIV'))
TRIP_TIME_DIV = int(os.getenv('TRIP_TIME_DIV'))
ODOMETER_DIV = int(os.getenv('ODOMETER_DIV'))
COOLANT_TEMP_DIV = int(os.getenv('COOLANT_TEMP_DIV'))
INTAKE_AIR_TEMP_DIV = int(os.getenv('INTAKE_AIR_TEMP_DIV'))
DEBUG_VERBOSE = int(os.getenv('DEBUG_VERBOSE'))

CAF_APP_LOG_DIR = os.getenv("CAF_APP_LOG_DIR", "/tmp")

log_file_path = os.path.join(CAF_APP_LOG_DIR, "iox-vehicle-obd2.log")

logger = logging.getLogger(__name__)
if (DEBUG_VERBOSE == 1):
    logger.setLevel(logging.DEBUG)
else:
    logger.setLevel(logging.INFO)

handler = RotatingFileHandler(log_file_path, maxBytes=5000000, backupCount=1)
log_format = logging.Formatter('[%(asctime)s]{%(pathname)s:%(lineno)d}%(levelname)s- %(message)s')
handler.setFormatter(log_format)
logger.addHandler(handler)

sn = os.getenv('CAF_SYSTEM_SERIAL_ID')
topic = MQTT_BASE_TOPIC + "/" + sn

logger.info('-------------------------------------------')
logger.info('CONFIGURATION:')
logger.info("ROUTER SERIAL NUM: %s", sn)
logger.info("IR_GPS: %s", IR_GPS)
logger.info("LOOP_INTERVAL: %d", LOOP_INTERVAL)
logger.info("LOCATION_DIV: %d", LOCATION_DIV)
logger.info("VEHICLE_SPEED_DIV: %d", VEHICLE_SPEED_DIV)
logger.info("ENGINE_RPM_DIV: %d", ENGINE_RPM_DIV)
logger.info("FUEL_LEVEL_DIV: %d", FUEL_LEVEL_DIV)
logger.info("TRIP_TIME_DIV: %d", TRIP_TIME_DIV)
logger.info("ODOMETER_DIV: %d", ODOMETER_DIV)
logger.info("COOLANT_TEMP_DIV: %d", COOLANT_TEMP_DIV)
logger.info("INTAKE_AIR_TEMP_DIV: %d", INTAKE_AIR_TEMP_DIV)
logger.info("MQTT_BROKER: %s", MQTT_BROKER)
logger.info("MQTT_PORT: %d", MQTT_PORT)
logger.info("MQTT_USERNAME: %s", MQTT_USERNAME)
logger.info("MQTT_PASSWORD: %s", MQTT_PASSWORD)
logger.info("MQTT_USE_TLS: %d", MQTT_USE_TLS)
logger.info("MQTT_QOS: %d", MQTT_QOS)
logger.info("MQTT_TOPIC: %s", topic)
logger.info("DEBUG_VERBOSE: %d", DEBUG_VERBOSE)

mqttClient = paho.Client()
mqttClient.reconnect_delay_set(min_delay=5, max_delay=60)
mqttClient.on_connect = on_connect
mqttClient.on_disconnect = on_disconnect
mqttClient.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

if (MQTT_USE_TLS == 1):
    mqttClient.tls_set()

mqttSuccess = False
while (mqttSuccess == False):
    try:
        logger.debug('Attempting to connect to MQTT data broker.')
        mqttClient.connect(MQTT_BROKER, MQTT_PORT)
        mqttSuccess = True
    except:
        logger.debug('Having trouble connecting to MQTT data broker.  Will try again.')
        time.sleep(5)

mqttClient.loop_start()

canbus = can.Bus(channel='vxcan0', bustype='socketcan', bitrate=250000)
gpsser = serial.Serial(IR_GPS, 115200, timeout=2)

cycleCount = 0

while True:
    logger.debug('Starting loop')

    timestamp = int(time.time() * 1000)
    tempPayload = {}

    #Location

    if (LOCATION_DIV != 0):
        if ((cycleCount % LOCATION_DIV) == 0):
            locationData = ReqLocation(gpsser)
            if (locationData is not None):
                tempPayload['location'] = locationData

    #Check for supported PIDs
    tempPayload['canbusActive'] = 0
    if ((cycleCount % 1) == 0):
        supported_PID_01_20 = getSupported(canbus,0)
        if (supported_PID_01_20 != ('0' * 32)):
            tempPayload['canbusActive'] = 1
        supported_PID_21_40 = '0' * 32
        supported_PID_41_60 = '0' * 32
        supported_PID_61_80 = '0' * 32
        supported_PID_81_A0 = '0' * 32
        supported_PID_A1_C0 = '0' * 32

        if supported_PID_01_20[32-1] == '1':
            supported_PID_21_40=getSupported(canbus,32)
        if supported_PID_21_40[32-1] == '1':
            supported_PID_41_60 = getSupported(canbus,64)
        if supported_PID_41_60[32-1] == '1':
            supported_PID_61_80 = getSupported(canbus,96)
        if supported_PID_61_80[32-1] == '1':
            supported_PID_81_A0 = getSupported(canbus,128)
        if supported_PID_81_A0[32-1] == '1':
            supported_PID_A1_C0 = getSupported(canbus,160)

    if (cycleCount == 0):
        logger.info("Supported PID_01_20: %s", binToHex(supported_PID_01_20))
        logger.info("Supported PID_21_40: %s", binToHex(supported_PID_21_40))
        logger.info("Supported PID_41_60: %s", binToHex(supported_PID_41_60))
        logger.info("Supported PID_61:80: %s", binToHex(supported_PID_61_80))
        logger.info("Supported PID_81_A0: %s", binToHex(supported_PID_81_A0))
        logger.info("Supported PID_A1_C0: %s", binToHex(supported_PID_A1_C0))

    #Vehicle Speed
    #PID=0x0D=d13; supported_PID_01_20, bit 12
    if (VEHICLE_SPEED_DIV != 0):
        if ((cycleCount % VEHICLE_SPEED_DIV) == 0 and supported_PID_01_20[12] == '1'):
            message = ReqPID(canbus,13)
            if (message is not None):
                tempData = message.data[3]
                tempPayload['vehicleSpeed'] = {'value': tempData, 'unit': 'kmph'}

    #Engine RPM
    #PID=0x0C=d12; supported_PID_01_20, bit 11
    if (ENGINE_RPM_DIV != 0):
        if ((cycleCount % ENGINE_RPM_DIV) == 0 and supported_PID_01_20[11] == '1'):
            message = ReqPID(canbus,12)
            if (message is not None):
                tempArray = bytearray([message.data[3],message.data[4]])
                tempData = (struct.unpack('>H', tempArray)[0])/4
                tempPayload['engineRPM'] = {'value': tempData, 'unit': 'rpm'}

    #Fuel tank level
    #PID=0x2F=d47; supported_PID_21_40, bit 14
    if (FUEL_LEVEL_DIV != 0):
        if ((cycleCount % FUEL_LEVEL_DIV) == 0 and supported_PID_21_40[14] == '1'):
            message = ReqPID(canbus,47)
            if (message is not None):
                tempData = round(((100/255) * message.data[3]),2)
                tempPayload['fuelLevel'] = {'value': tempData, 'unit': 'percent'}

    #Time since engine start
    #PID=0x1F=d31; supported_PID_01_20, bit 30
    if (TRIP_TIME_DIV != 0):
        if ((cycleCount % TRIP_TIME_DIV) == 0 and supported_PID_01_20[30] == '1'):
            message = ReqPID(canbus,31)
            if (message is not None):
                tempArray = bytearray([message.data[3],message.data[4]])
                tempData = struct.unpack('>H', tempArray)[0]
                tempPayload['tripTime'] = {'value': tempData, 'unit': 'seconds'}

    #Odometer
    #PID=0xA6=d166; supported_PID_A1_C0, bit 5
    if (ODOMETER_DIV != 0):
        if ((cycleCount % ODOMETER_DIV) == 0 and supported_PID_A1_C0[5] == '1'):
            message = ReqPID(canbus,166)
            if (message is not None):
                tempArray = bytearray([message.data[3],message.data[4],message.data[5],message.data[6]])
                tempData = round(((struct.unpack('>I', tempArray)[0])/10),2)
                tempPayload['odometer'] = {'value': tempData, 'unit': 'km'}

    #Engine coolant temperature
    #PID=0x05=d05; supported_PID_01_20, bit 4
    if (COOLANT_TEMP_DIV != 0):
        if ((cycleCount % COOLANT_TEMP_DIV) == 0 and supported_PID_01_20[4] == '1'):
            message = ReqPID(canbus,5)
            if (message is not None):
                tempData = message.data[3] - 40
                tempPayload['engineCoolantTemp'] = {'value': tempData, 'unit': 'C'}

    #Intake air temperature
    #PID=0x0F=d15; supported_PID_01_20, bit 14
    if (INTAKE_AIR_TEMP_DIV != 0):
        if ((cycleCount % INTAKE_AIR_TEMP_DIV) == 0 and supported_PID_01_20[14] == '1'):
            message = ReqPID(canbus,15)
            if (message is not None):
                tempData = message.data[3] - 40
                tempPayload['intakeAirTemp'] = {'value': tempData, 'unit': 'C'}

    if (tempPayload):
        tempPayload['timestamp'] = timestamp
        tempPayload['identifier'] = sn
        tempJSON = json.dumps(tempPayload, indent=4)
        logger.debug('Publishing message over MQTT')
        mqttClient.publish(topic,tempJSON,qos=MQTT_QOS)

    logger.debug('Finishing loop')

    startTime = time.time();
    while ( (time.time() - startTime) < LOOP_INTERVAL ):
        canbus.recv(timeout=0.1)

    cycleCount += 1

