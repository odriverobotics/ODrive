import sys
import flask
import os
from flask import make_response, request, jsonify, session
from flask_socketio import SocketIO, send, emit
from flask_cors import CORS
from engineio.payload import Payload
import json
import time
import argparse
import logging
import math
import traceback

# interface for odrive GUI to get data from odrivetool

# Flush stdout by default
# Source:
# https://stackoverflow.com/questions/230751/how-to-flush-output-of-python-print
old_print = print
def print(*args, **kwargs):
    kwargs.pop('flush', False)
    old_print(*args, **kwargs)
    file = kwargs.get('file', sys.stdout)
    file.flush() if file is not None else sys.stdout.flush()

app = flask.Flask(__name__)
# disable logging, very noisy!
log = logging.getLogger('werkzeug')
log.disabled = True
app.config['SECRET_KEY'] = 'secret'
app.config.update(
    SESSION_COOKIE_SECURE=True,
    SESSION_COOKIE_HTTPONLY=True,
    SESSION_COOKIE_SAMESITE='None'
)
CORS(app, support_credentials=True)
Payload.max_decode_packets = 500
socketio = SocketIO(app, cors_allowed_origins="*", async_mode = "threading")

#def get_odrive():
#    globals()['odrives'] = []
#    globals()['odrives'].append(odrive.find_any())
#    globals()['odrives'][0].__channel__._channel_broken.subscribe(lambda: handle_disconnect())
#    print("odrives found")
#    socketio.emit('odrive-found')

async def discovered_device(device):
    # when device is discovered, add it to list of serial numbers and global odrive list
    # shamelessly lifted from odrive python package
    serial_number = await odrive.get_serial_number_str(device)
    if serial_number in globals()['discovered_devices']:
        index = globals()['discovered_devices'].index(serial_number)
    else:
        globals()['discovered_devices'].append(serial_number)
        index = len(globals()['discovered_devices']) - 1
    odrive_name = "odrive" + str(index)

    # add to list of odrives
    while globals()['inUse']:
        time.sleep(0.1)
    globals()['odrives'][odrive_name] = device
    globals()['odrives_status'][odrive_name] = True
    print("Found " + str(serial_number))
    print("odrive list: " + str([key for key in globals()['odrives'].keys()]))
    # tell GUI the status of known ODrives (previously connected and then disconnected ODrives will be "False")
    socketio.emit('odrives-status', json.dumps(globals()['odrives_status']))
    # triggers a getODrives socketio message
    socketio.emit('odrive-found')

def start_discovery():
    print("starting disco loop...")
    log = fibre.Logger(verbose = False)

    domain = fibre.Domain("usb:idVendor=0x1209,idProduct=0x0D32,bInterfaceClass=0,bInterfaceSubClass=1,bInterfaceProtocol=0")
    domain = domain.__enter__()
    discovery = domain.run_discovery(discovered_device)

def handle_disconnect(odrive_name):
    print("lost odrive")
    globals()['odrives_status'][odrive_name] = False
    # emit the whole list of odrive statuses
    # in the GUI, mark and use status as ODrive state.
    socketio.emit('odrives-status', json.dumps(globals()['odrives_status']))

@socketio.on('findODrives')
def getODrives(message):
    print("looking for odrive")
    start_discovery()

@socketio.on('enableSampling')
def enableSampling(message):
    print("sampling enabled")
    session['samplingEnabled'] = True
    emit('samplingEnabled')

@socketio.on('stopSampling')
def stopSampling(message):
    session['samplingEnabled'] = False
    emit('samplingDisabled')

@socketio.on('sampledVarNames')
def sampledVarNames(message):
    session['sampledVars'] = message
    print(session['sampledVars'])
    
@socketio.on('startSampling')
def sendSamples(message):
    print(session['samplingEnabled'])
    while session['samplingEnabled']:
        emit('sampledData', json.dumps(getSampledData(session['sampledVars'])))
        time.sleep(0.02)

@socketio.on('message')
def handle_message(message):
    print(message)
    emit('response', 'hello from server!')

@socketio.on('getODrives')
def get_odrives(data):
    # spinlock
    while globals()['inUse']:
        time.sleep(0.1)

    globals()['inUse'] = True
    odriveDict = {}
    #for (index, odrv) in enumerate(globals()['odrives']):
    #    odriveDict["odrive" + str(index)] = dictFromRO(odrv)
    for key in globals()['odrives_status'].keys():
        if globals()['odrives_status'][key] == True:
            odriveDict[key] = dictFromRO(globals()['odrives'][key])
    globals()['inUse'] = False
    emit('odrives', json.dumps(odriveDict))

@socketio.on('getProperty')
def get_property(message):
    # message is dict natively
    # will be {"path": "odriveX.axisY.blah.blah"}
    while globals()['inUse']:
        time.sleep(0.1)
    if globals()['odrives_status'][message["path"].split('.')[0]]:
        globals()['inUse'] = True
        val = getVal(globals()['odrives'], message["path"].split('.'))
        globals()['inUse'] = False
        emit('ODriveProperty', json.dumps({"path": message["path"], "val": val}))

@socketio.on('setProperty')
def set_property(message):
    # message is {"path":, "val":, "type":}
    while globals()['inUse']:
        time.sleep(0.1)
    globals()['inUse'] = True
    print("From setProperty event handler: " + str(message))
    postVal(globals()['odrives'], message["path"].split('.'), message["val"], message["type"])
    val = getVal(globals()['odrives'], message["path"].split('.'))
    globals()['inUse'] = False
    emit('ODriveProperty', json.dumps({"path": message["path"], "val": val}))

@socketio.on('callFunction')
def call_function(message):
    # message is {"path"}, no args yet (do we know which functions accept arguments from the odrive tree directly?)
    while globals()['inUse']:
        time.sleep(0.1)
    print("From callFunction event handler: " + str(message))
    globals()['inUse'] = True
    callFunc(globals()['odrives'], message["path"].split('.'))
    globals()['inUse'] = False

@app.route('/', methods=['GET'])
def home():
    return "<h1>ODrive GUI Server</h1>"

def dictFromRO(RO):
    # create dict from an odrive RemoteObject that's suitable for sending as JSON
    returnDict = {}

    for key in dir(RO):
        v = getattr(RO, key)
        if not key.startswith('_') and isinstance(v, fibre.libfibre.RemoteObject):
            # recurse
            returnDict[key] = dictFromRO(v)
        
        elif key.startswith('_') and key.endswith('_property'):
            # grab value of that property
            # indicate if this property can be written or not
            val = str(v.read())
            _type = str(v.read._outputs[0][1])
            if val == "inf":
                val = "Infinity"
                _type = "str"
            elif val == "-inf":
                val = "-Infinity"
                _type = "str"
            returnDict[key[1:-9]] = {"val": val,
                               "readonly": not hasattr(v, 'exchange'),
                               "type": _type}
        elif not key.startswith('_') and hasattr(v, '__call__'):
            # this is a function - do nothing for now.
            print('found a function!',key)
            returnDict[key] = "function"

    return returnDict

def postVal(odrives, keyList, value, argType):
    # expect a list of keys in the form of ["key1", "key2", "keyN"]
    # "key1" will be "odriveN"
    # like this: postVal(odrives, ["odrive0","axis0","config","calibration_lockin","accel"], 17.0)
    try:
        odrv = keyList.pop(0)
        RO = odrives[odrv]
        keyList[-1] = '_' + keyList[-1] + '_property'
        for key in keyList:
            RO = getattr(RO, key)
        if argType == "number":
            RO.exchange(float(value))
        elif argType == "boolean":
            RO.exchange(value)
        elif argType == "string":
            if value == "Infinity":
                RO.exchange(math.inf)
            elif value == "-Infinity":
                RO.exchange(-math.inf)
        else:
            pass # dont support that type yet
    except fibre.ObjectLostError:
        handle_disconnect(odrv)
    except Exception as ex:
        print("exception in postVal: ", traceback.format_exc())

def getVal(odrives, keyList):
    try:
        odrv = keyList.pop(0)
        RO = odrives[odrv]
        keyList[-1] = '_' + keyList[-1] + '_property'
        for key in keyList:
            RO = getattr(RO, key)
        retVal = RO.read()
        if retVal == math.inf:
            retVal = "Infinity"
        elif retVal == -math.inf:
            retVal = "-Infinity"
        return retVal
    except fibre.ObjectLostError:
        handle_disconnect(odrv)
    except Exception as ex:
        print("exception in getVal: ", traceback.format_exc())
        return 0

def getSampledData(vars):
    #use getVal to populate a dict
    #return a dict {path:value}
    samples = {}
    for path in vars["paths"]:
        keys = path.split('.')
        samples[path] = getVal(globals()['odrives'], keys)

    return samples

def callFunc(odrives, keyList):
    try:
        #index = int(''.join([char for char in keyList.pop(0) if char.isnumeric()]))
        odrv = keyList.pop(0)
        RO = odrives[odrv]
        for key in keyList:
            RO = getattr(RO, key)
        if hasattr(RO, '__call__'):
            RO.__call__()
    except fibre.ObjectLostError:
        handle_disconnect(odrv)
    except:
        print("fcn call failed")

if __name__ == "__main__":
    print("args from python: " + str(sys.argv[1:0]))
    #print(sys.argv[1:])
    # try to import based on command line arguments or config file

    for optPath in sys.argv[1:]:
        print("adding " + str(optPath.rstrip()) + " to import path for odrive_server.py")
        sys.path.insert(0,optPath.rstrip())

    import odrive
    import odrive.utils # for dump_errors()
    import fibre

    # global for holding references to all connected odrives
    globals()['odrives'] = {}
    # global dict {'odriveX': True/False} where True/False reflects status of connection
    # on handle_disconnect, set it to False. On connection, set it to True
    globals()['odrives_status'] = {}
    globals()['discovered_devices'] = []
    # spinlock
    globals()['inUse'] = False

    log = fibre.Logger(verbose=False)
    shutdown = fibre.Event()

    socketio.run(app, host='0.0.0.0', port=5000)
