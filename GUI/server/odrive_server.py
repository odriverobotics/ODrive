import sys
import os
import json
import time
import argparse
import socketio as sio
import eventlet

# interface for odrive GUI to get data from odrivetool

socketio = sio.Server(cors_allowed_origins='*')
app = sio.WSGIApp(socketio, static_files={
    '/': {'content_type': 'text/html', 'filename': 'index.html'}
})

def get_all_odrives():
    globals()['odrives'] = []
    globals()['odrives'].append(odrive.find_any())
    globals()['odrives'][0].__channel__._channel_broken.subscribe(lambda: handle_disconnect())

def handle_disconnect():
    print("lost odrive, attempting to reconnect")
    # synchronizing flag for connect/disconnect
    globals()['inUse'] = True
    globals()['odrives'] = []
    while len(globals()['odrives']) == 0:
        get_all_odrives()
        time.sleep(0.1)
    globals()['inUse'] = False
    print("reconnected!")

@socketio.on('enableSampling')
def enableSampling(sid, message):
    print("sampling enabled")
    globals()['samplingEnabled'] = True
    socketio.emit('samplingEnabled')

@socketio.on('stopSampling')
def stopSampling(sid, message):
    globals()['samplingEnabled'] = False
    socketio.emit('samplingDisabled')

@socketio.on('sampledVarNames')
def sampledVarNames(sid, message):
    globals()['sampledVars'] = message
    print(globals()['sampledVars'])

@socketio.on('startSampling')
def sendSamples(sid, message):
    print(globals()['samplingEnabled'])
    while globals()['samplingEnabled']:
        socketio.emit('sampledData', json.dumps(getSampledData(globals()['sampledVars'])))
        time.sleep(0.02)

@socketio.on('message')
def handle_message(sid, message):
    print(message)
    socketio.emit('response', 'hello from server!')

@socketio.on('getODrives')
def get_odrives(sid, data):
    # spinlock
    while globals()['inUse']:
        time.sleep(0.1)

    globals()['inUse'] = True
    odriveDict = {}
    for (index, odrv) in enumerate(globals()['odrives']):
        odriveDict["odrive" + str(index)] = dictFromRO(odrv)
    globals()['inUse'] = False
    socketio.emit('odrives', json.dumps(odriveDict))

@socketio.on('getProperty')
def get_property(sid, message):
    # message is dict natively
    # will be {"path": "odriveX.axisY.blah.blah"}
    while globals()['inUse']:
        time.sleep(0.1)
    globals()['inUse'] = True
    val = getVal(globals()['odrives'], message["path"].split('.'))
    socketio.emit('ODriveProperty', json.dumps({"path": message["path"], "val": val}))
    globals()['inUse'] = False

@socketio.on('setProperty')
def set_property(sid, message):
    # message is {"path":, "val":, "type":}
    while globals()['inUse']:
        time.sleep(0.1)
    globals()['inUse'] = True
    print("From setProperty event handler: " + str(message))
    postVal(globals()['odrives'], message["path"].split('.'), message["val"], message["type"])
    globals()['inUse'] = False

@socketio.on('callFunction')
def call_function(sid, message):
    # message is {"path"}, no args yet (do we know which functions accept arguments from the odrive tree directly?)
    while globals()['inUse']:
        time.sleep(0.1)
    globals()['inUse'] = True
    print("From callFunction event handler: " + str(message))
    callFunc(globals()['odrives'], message["path"].split('.'))
    globals()['inUse'] = False

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
    index = int(''.join([char for char in keyList.pop(0) if char.isnumeric()]))

    RO = odrives[index]
    for key in keyList:
        RO = RO._remote_attributes[key]
    if argType == "number":
        RO.set_value(float(value))
    elif argType == "boolean":
        RO.set_value(value == "true")
    else:
        pass # dont support that type yet

def getVal(odrives, keyList):
    index = int(''.join([char for char in keyList.pop(0) if char.isnumeric()]))
    RO = odrives[index]
    try:
        for key in keyList:
            RO = RO._remote_attributes[key]
        if isinstance(RO, fibre.remote_object.RemoteObject):
            return dictFromRO(RO)
        else:
            return RO.get_value()
    except:
        return 0

def getSampledData(vars):
    #use getVal to populate a dict
    #return a dict {path:value}
    samples = {}
    for path in vars["paths"]:
        keys = path.split('.')
        samples[path] = getVal(globals()['odrives'], keys)

    return samples

# call a function from a GET request


def callFunc(odrives, keyList):
    index = int(''.join([char for char in keyList.pop(0) if char.isnumeric()]))
    RO = odrives[index]
    for key in keyList:
        RO = RO._remote_attributes[key]
    if isinstance(RO, fibre.remote_object.RemoteFunction):
        try:
            RO.__call__()
        except:
            print("fcn call failed")

if __name__ == "__main__":
    # sleep to allow time for background.js to register event callbacks for stdout,stderr
    time.sleep(3)
    print("args from python: " + str(sys.argv[1:0]))
    #print(sys.argv[1:])
    # try to import based on command line arguments or config file

    for optPath in sys.argv[1:]:
        print("adding " + str(optPath.rstrip()) + " to import path for odrive_server.py")
        sys.path.insert(0,optPath.rstrip())

    import odrive
    import odrive.utils # for dump_errors()
    import fibre

    globals()['odrives'] = []
    # spinlock
    globals()['inUse'] = False

    # busy wait for connection
    while len(globals()['odrives']) == 0:
        print("looking for odrives...")
        get_all_odrives()

    print("found odrives!")
    globals()['connected'] = True

    #socketio.run(app, host='0.0.0.0', port=5000)
    eventlet.wsgi.server(eventlet.listen(('', 5000)), app)
