import Vue from 'vue';
import Vuex from 'vuex';
import TuningDashOld from "./assets/dashboards/Tuning_0_4_12.json";
import TuningDashNew from "./assets/dashboards/Tuning_0_5_1.json";
import * as socketio from "./comms/socketio";
import {filterBy, deleteBy } from "./lib/utils.js";
//import { v4 as uuidv4 } from "uuid";




Vue.use(Vuex);

export default new Vuex.Store({
    // state is the data for this app
    state: {
        odrives: Object,
        odriveConfigs: Object,
        odriveParameters: Object,
        odriveFunctions: Object,
        axes: Array,
        odriveServerAddress: String,
        serverConnected: Boolean,
        ODrivesConnected: Object,
        serverOutput: [],
        dashboards: [
            {
                name: "Start",
                component: "Start",
            },
        ],
        timeSampleStart: 0,
        sampledProperties: [], // make this an object where the full path is a key and the value is the sampled var
        propSamples: { time: [] }, // {time: [time values], ...path: [path var values]}
        newData: false,
        sampling: false,
        currentDash: "Start",
        firstConn: false,
        wizardMotor: "odrive0",
    },
    // mutations are functions that change the data
    mutations: {
        setWizardMotor(state, odrive) {
            // odrive is string, "odrive0" or "odriveN"
            state.wizardMotor = odrive;
        },
        setDash(state, dashName) {
            state.currentDash = dashName;
        },
        setOdrives(state, odrives) {
            state.odrives = odrives;
            if (state.firstConn == false) {
                // first time we're getting odrive data, add correct config page
                let TuningDash;
                if (state.odrives.odrive0.fw_version_minor.val == "5") {
                    TuningDash = TuningDashNew;
                }
                else if (state.odrives.odrive0.fw_version_minor.val == "4") {
                    TuningDash = TuningDashOld;
                }
                else {
                    TuningDash = TuningDashNew;
                }
                state.dashboards.push({
                    name: "Wizard",
                    component: "Wizard",
                });
                state.dashboards.push(TuningDash);
                // plots will have variables associated, add them to sampled variables list
                for (const plot of TuningDash.plots) {
                    for (const plotVar of plot.vars) {
                        //addsampledprop(path);
                        let path = plotVar.path;
                        if (!(path in state.sampledProperties)) {
                            let newPath = path.split('.');
                            newPath.splice(0, 1);
                            state.sampledProperties.push(newPath.join('.'));
                            state.propSamples[newPath.join('.')] = [];
                        }
                        socketio.sendEvent({
                            type: 'sampledVarNames',
                            data: {
                                paths: state.sampledProperties
                            }
                        });
                    }
                }
            }
            state.firstConn = true;
        },
        setOdriveConfigs(state, payload) {
            state.odriveConfigs['full'] = payload.full;
            state.odriveConfigs['functions'] = payload.functions;
            state.odriveConfigs['params'] = payload.params;
            state.odriveConfigs['writeAble'] = payload.writeAble;
            state.odriveConfigs['writeAbleNumeric'] = payload.writeAbleNumeric;
        },
        setODrivesStatus(state, obj) {
            // obj is {"odriveX": true/false}
            for (const odrive of Object.keys(obj)){
                state.ODrivesConnected[odrive] = obj[odrive];
                console.log(state.ODrivesConnected);
            }
        },
        setAxes(state, axes) {
            state.axes = axes;
        },
        setServerAddress(state, address) {
            state.odriveServerAddress = address;
        },
        updateOdriveProp(state, payload) {
            // need to use Vue.set!!!
            // payload is {path, value}
            // 
            const createNestedObject = (odrive, path) => {
                let ref = odrive;
                let keys = path.split('.');
                for (const key of keys) {
                    ref = ref[key];
                }
                return ref;
            };
            let val = payload.val
            if (val == "Infinity") {
                val = Number.POSITIVE_INFINITY;
                console.log("val is infinity");
            }
            else if (val == "-Infinity") {
                val = Number.NEGATIVE_INFINITY;
            }
            Vue.set(createNestedObject(state.odrives, payload.path), "val", val);

        },
        addSampledProperty(state, path) {
            if (!(path in state.sampledProperties)) {
                let newPath = path.split('.');
                newPath.splice(0, 1);
                state.sampledProperties.push(newPath.join('.'));
                state.propSamples[newPath.join('.')] = [];
            }
            socketio.sendEvent({
                type: 'sampledVarNames',
                data: {
                    paths: state.sampledProperties
                }
            });
        },
        removeSampledProperty(state, path) {
            let newPath = path.split('.');
            newPath.splice(0, 1);
            const index = state.sampledProperties.indexOf(newPath.join('.'));
            if (index > -1) {
                state.sampledProperties.splice(index, 1);
            }
        },
        updateSampledProperty(state, payload) {
            // payload is object of paths and values
            for (const path of Object.keys(payload)) {
                state.propSamples[path].push(payload[path]);
                if (state.propSamples[path].length > 250) {
                    state.propSamples[path].splice(0, 1); // emulate circular buffer
                }
            }
            state.propSamples["time"].push((Date.now() - state.timeSampleStart) / 1000);
            if (state.propSamples["time"].length > 250) {
                state.propSamples["time"].splice(0, 1);
            }
            state.newData = true;
        },
        logServerMessage(state, payload) {
            // payload is string
            state.serverOutput.push(payload);
            // cut off to 1000 lines
            if (state.serverOutput.length > 1000) {
                state.serverOutput.splice(0,1);
            }
        },
        setServerStatus(state, val) {
            state.serverConnected = val;
        },
        removeCtrlFromDash(state, obj) {
            // obj is {dash: dashID, path: control path}
            for (const dash of state.dashboards) {
                if (obj.dashID == dash.id) {
                    for (const control of dash.controls) {
                        if (obj.path == control.path) {
                            dash.controls.splice(dash.controls.indexOf(control), 1);
                            break;
                        }
                    }
                    break;
                }
            }
        },
        removeActionFromDash(state, obj) {
            // obj is {dashID: dashID, actionID: action ID}
            for (const dash of state.dashboards) {
                if (obj.dashID == dash.id) {
                    for (const action of dash.actions) {
                        if (obj.actionID == action.id) {
                            dash.actions.splice(dash.actions.indexOf(action), 1);
                            break;
                        }
                    }
                    break;
                }
            }
        },
        removePlotFromDash(state, obj) {
            // obj is {dashID: dash ID, plotID: plot ID}
            for (const dash of state.dashboards) {
                if (obj.dashID == dash.id) {
                    for (const plot of dash.plots) {
                        if (obj.plotID == plot.name) {
                            dash.plots.splice(dash.plots.indexOf(plot), 1);
                            break;
                        }
                    }
                    break;
                }
            }
        },
        setActionVal(state, obj) {
            // obj is {dashID: dash ID, actionID: action ID, val: val}
            for (const dash of state.dashboards) {
                if (obj.dashID == dash.id) {
                    for (const action of dash.actions) {
                        if (obj.actionID == action.id) {
                            action.val = obj.val;
                            break;
                        }
                    }
                    break;
                }
            }
        },
    },
    // actions trigger mutations
    actions: {
        getOdrives() {
            socketio.sendEvent({
                type: 'getODrives',
            });
        },
        getOdriveConfigs(context) {
            // transform ODrive JSON
            function treeParse(odriveObj) {
                let retObj = {};
                for (const key of Object.keys(odriveObj)) {
                    if (typeof odriveObj[key] === 'object' && odriveObj[key] !== null) {
                        // check if "val" is a valid key
                        if (Object.prototype.hasOwnProperty.call(odriveObj[key], "val")) {
                            // parse from string to a type that we care about
                            switch (odriveObj[key]["type"]) {
                                case "str": {
                                    // for handling infinity
                                    let val;
                                    if (odriveObj[key]["val"] == 'Infinity') {
                                        val = Number.POSITIVE_INFINITY;
                                    }
                                    else if (odriveObj[key]["val"] == '-Infinity') {
                                        val = Number.NEGATIVE_INFINITY;
                                    }
                                    retObj[key] = val;
                                    break;
                                }
                                case "float":
                                    retObj[key] = parseFloat(parseFloat(odriveObj[key]["val"]).toFixed(3));
                                    break;
                                case "int8":
                                case "int16":
                                case "int32":
                                case "int64":
                                case "uint8":
                                case "uint16":
                                case "uint32":
                                case "uint64":
                                    retObj[key] = parseInt(odriveObj[key]["val"]);
                                    break;
                                case "bool":
                                    retObj[key] = odriveObj[key]["val"] == 'True';
                                    break;
                                default:
                                    retObj[key] = odriveObj[key]["val"];
                            }
                        }
                        else {
                            retObj[key] = treeParse(odriveObj[key]);
                        }
                    }
                    else if (odriveObj[key] == "function") {
                        retObj[key] = "function";
                    }
                    else {
                        retObj[key] = odriveObj[key];
                    }
                }
                return retObj;
            }
            // full parameter tree
            // create object containing only ODrive functions
            let fullTree = treeParse(context.state.odrives);
            let fcns = filterBy(fullTree, ((o) => o == "function"));
            deleteBy(fcns, ((o) => o.constructor == Object && Object.keys(o).length == 0));

            // create object containing only ODrive parameters
            let params = filterBy(fullTree, ((o) => o != "function" && typeof o != "object"));
            deleteBy(params, ((o) => o.constructor == Object && Object.keys(o).length == 0))

            // create object containing only ODrive parameters with write access
            let writeAble = filterBy(context.state.odrives, ((o) => o['readonly'] == false));
            deleteBy(writeAble, ((o) => o.constructor == Object && Object.keys(o).length == 0))

            // create object of only ODrive parameters that are writeable and numeric
            let writeAbleNumeric = filterBy(context.state.odrives, ((o) => o['readonly'] == false && ['float', 'int', 'bool'].includes(o['type'])));
            deleteBy(writeAbleNumeric, ((o) => o.constructor == Object && Object.keys(o).length == 0))

            context.commit('setOdriveConfigs', {full: fullTree, 
                                                functions: fcns, 
                                                params: params, 
                                                writeAble: treeParse(writeAble),
                                                writeAbleNumeric: treeParse(writeAbleNumeric)});
        },
        getAxes(context) {
            let axes = [];
            //for each connected odrive, collect axes and display them
            for (const odrive of Object.keys(context.state.odrives)) {
                if ('axis0' in context.state.odrives[odrive]) {
                    axes.push({
                        name: `${odrive}.axis0`,
                    });
                }
                if ('axis1' in context.state.odrives[odrive]) {
                    axes.push({
                        name: `${odrive}.axis1`,
                    });
                }
            }
            context.commit('setAxes', axes);
        },
        setServerAddress(context, address) {
            context.commit('setServerAddress', address);
            socketio.setUrl(address);
            socketio.addEventListener({
                type: "connect",
                callback: () => {
                    context.commit("setServerStatus", true);
                    console.log('connected to server');
                    //context.dispatch("getOdrives");
                    socketio.sendEvent({
                        type: "findODrives",
                    });
                }
            });
            socketio.addEventListener({
                type: "odrive-found",
                callback: () => {
                    console.log("odrive-found recieved from server");
                    context.dispatch("getOdrives");
                }
            })
            socketio.addEventListener({
                type: "disconnect",
                callback: () => {
                    context.commit("setServerStatus", false);
                    console.log('server disconnect');
                    socketio.closeSocket();
                    context.commit('setAxes', []);
                }
            });
            socketio.addEventListener({
                type: "sampledData",
                callback: message => {
                    context.commit("updateSampledProperty", JSON.parse(message));
                }
            });
            socketio.addEventListener({
                type: "samplingEnabled",
                callback: () => {
                    socketio.sendEvent({
                        type: "startSampling"
                    });
                }
            });
            socketio.addEventListener({
                type: "odrives",
                callback: odrives => {
                    context.commit('setOdrives', JSON.parse(odrives));
                    context.dispatch('getOdriveConfigs');
                    context.dispatch('getAxes');
                }
            });
            // getVal event gets sent to server, server emits ODriveProperty event with path and val of property
            socketio.addEventListener({
                type: "ODriveProperty",
                callback: retmsg => {
                    // retmsg is {path, val}
                    context.commit('updateOdriveProp', JSON.parse(retmsg));
                }
            });
            socketio.addEventListener({
                type: "odrive-disconnected",
                callback: (odrive_name) => {
                    console.log(odrive_name + " disconnected");
                    //console.log("restarting server...");
                    //window.ipcRenderer.send('kill-server');
                    //window.ipcRenderer.send('start-server');
                    //context.dispatch('setServerAddress', context.state.odriveServerAddress);
                }
            });
            socketio.addEventListener({
                type: "odrives-status",
                callback: (odrives_status) => {
                    console.log("From odrives-status msg " + odrives_status);
                    context.commit('setODrivesStatus', JSON.parse(odrives_status));
                }
            });
        }
    }
})