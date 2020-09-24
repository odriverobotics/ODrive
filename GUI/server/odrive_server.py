import sys
import flask
import os
from flask import make_response, request, jsonify, session
from flask_socketio import SocketIO, send, emit
from flask_cors import CORS
import json
import time
import argparse


# interface for odrive GUI to get data from odrivetool

#better handling of websockets
# eventlet.monkey_patch()

app = flask.Flask(__name__)
app.config['SECRET_KEY'] = 'secret'
app.config.update(
    SESSION_COOKIE_SECURE=True,
    SESSION_COOKIE_HTTPONLY=True,
    SESSION_COOKIE_SAMESITE='None'
)
CORS(app, support_credentials=True)
socketio = SocketIO(app, cors_allowed_origins="*")
configDict = {}

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

@app.route('/', methods=['GET'])
def home():
    return "<h1>ODrive GUI Server</h1>"


@app.route('/api/odrives', methods=["GET"])
def api_odrives():
    print(len(globals()['odrives']))
    print("inUse = " + str(globals()['inUse']))
    # spinlock
    while globals()['inUse']:
        time.sleep(0.1)

    globals()['inUse'] = True
    odriveDict = {}
    for (index, odrv) in enumerate(globals()['odrives']):
        odriveDict["odrive" + str(index)] = dictFromRO(odrv)
    response = jsonify(odriveDict)
    response.headers.add('Access-Control-Allow-Origin', '*')
    globals()['inUse'] = False
    return response


@app.route('/api/property', methods=["GET", "PUT"])
def api_property():
    # here, reqDict["key"] is a list of keys from the query
    # ?key=odrive0&key=axis0&key=config...
    print("inUse = " + str(globals()['inUse']))
    while globals()['inUse']:
        time.sleep(0.1)

    globals()['inUse'] = True
    if request.method == 'PUT':
        reqDict = request.args.to_dict(flat=False)
        postVal(globals()['odrives'], reqDict["key"], reqDict["val"][0], reqDict["type"][0])
        response = make_response(jsonify({"message": "success"}), 200)
        globals()['inUse'] = False
        return response
    else:
        print("request: " + str(request))
        reqDict = request.args.to_dict(flat=False)
        response = jsonify(getVal(globals()['odrives'], reqDict["key"]))
        response.headers.add('Access-Control-Allow-Origin', '*')
        globals()['inUse'] = False
        return response


@app.route('/api/function', methods=["PUT"])
def api_function():
    # execute a function from the odrive config dict?
    print("inUse = " + str(globals()['inUse']))
    while globals()['inUse']:
        time.sleep(0.1)

    globals()['inUse'] = True
    reqDict = request.args.to_dict(flat=False)
    callFunc(globals()['odrives'], reqDict["key"])
    response = make_response(jsonify({"message": "success"}), 200)
    globals()['inUse'] = False
    return response


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

# set a value from a POST http request


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
        samples[path] = getVal(odrives, keys)

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
    print("args from python:")
    print(sys.argv[1:])
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

    socketio.run(app, host='0.0.0.0', port=5000)
