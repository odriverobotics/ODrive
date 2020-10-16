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
Payload.max_decode_packets = 100
socketio = SocketIO(app, cors_allowed_origins="*", async_mode = "threading")

#def get_odrive():
#    globals()['odrives'] = []
#    globals()['odrives'].append(odrive.find_any())
#    globals()['odrives'][0].__channel__._channel_broken.subscribe(lambda: handle_disconnect())
#    print("odrives found")
#    socketio.emit('odrive-found')

def discovered_device(device):
    # when device is discovered, add it to list of serial numbers and global odrive list
    # shamelessly lifted from odrive python package
    serial_number = '{:012X}'.format(device.serial_number) if hasattr(device, 'serial_number') else "[unknown serial number]"
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
    print("Found " + str(serial_number))
    print("odrive list: " + str([key for key in globals()['odrives'].keys()]))
    socketio.emit('odrive-found')

def start_discovery():
    print("starting disco loop...")
    log = fibre.Logger(verbose = False)
    shutdown = fibre.Event()
    fibre.find_all("usb", None, discovered_device, shutdown, shutdown, log)

def handle_disconnect():
    print("lost odrive")
    #socketio.emit('odrive-disconnected')

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
    for key in globals()['odrives'].keys():
        odriveDict[key] = dictFromRO(globals()['odrives'][key])
    globals()['inUse'] = False
    emit('odrives', json.dumps(odriveDict))

@socketio.on('getProperty')
def get_property(message):
    # message is dict natively
    # will be {"path": "odriveX.axisY.blah.blah"}
    while globals()['inUse']:
        time.sleep(0.1)
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
    for key in RO._remote_attributes.keys():
        if isinstance(RO._remote_attributes[key], fibre.remote_object.RemoteObject):
            # recurse
            returnDict[key] = dictFromRO(RO._remote_attributes[key])
        elif isinstance(RO._remote_attributes[key], fibre.remote_object.RemoteProperty):
            # grab value of that property
            # indicate if this property can be written or not
            returnDict[key] = {"val": str(RO._remote_attributes[key].get_value()),
                               "readonly": not RO._remote_attributes[key]._can_write,
                               "type": str(RO._remote_attributes[key]._property_type.__name__)}
        elif isinstance(RO._remote_attributes[key], fibre.remote_object.RemoteFunction):
            # this is a function - do nothing for now.
            returnDict[key] = "function"
        else:
            returnDict[key] = RO._remote_attributes[key]
    return returnDict

def postVal(odrives, keyList, value, argType):
    # expect a list of keys in the form of ["key1", "key2", "keyN"]
    # "key1" will be "odriveN"
    # like this: postVal(odrives, ["odrive0","axis0","config","calibration_lockin","accel"], 17.0)
    try:
        #index = int(''.join([char for char in keyList.pop(0) if char.isnumeric()]))
        odrv = keyList.pop(0)
        RO = odrives[odrv]
        for key in keyList:
            RO = RO._remote_attributes[key]
        if argType == "number":
            RO.set_value(float(value))
        elif argType == "boolean":
            RO.set_value(value)
        else:
            pass # dont support that type yet
    except fibre.protocol.ChannelBrokenException:
        handle_disconnect()
    except:
        print("exception in postVal")

def getVal(odrives, keyList):
    try:
        #index = int(''.join([char for char in keyList.pop(0) if char.isnumeric()]))
        odrv = keyList.pop(0)
        RO = odrives[odrv]
        for key in keyList:
            RO = RO._remote_attributes[key]
        if isinstance(RO, fibre.remote_object.RemoteObject):
            return dictFromRO(RO)
        else:
            return RO.get_value()
    except fibre.protocol.ChannelBrokenException:
        handle_disconnect()
    except:
        print("exception in getVal")
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
            RO = RO._remote_attributes[key]
        if isinstance(RO, fibre.remote_object.RemoteFunction):
            RO.__call__()
    except fibre.protocol.ChannelBrokenException:
        handle_disconnect()
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

    globals()['odrives'] = {}
    globals()['discovered_devices'] = []
    # spinlock
    globals()['inUse'] = False

    log = fibre.Logger(verbose=False)
    shutdown = fibre.Event()

    socketio.run(app, host='0.0.0.0', port=5000)
