import Vue from 'vue';
import Vuex from 'vuex';
import ConfigDashOld from "./assets/dashboards/Config_0_4_12.json";
import ConfigDashNew from "./assets/dashboards/Config_0_5_1.json";
const axios = require('axios');
import * as socketio from "./comms/socketio";
//import { v4 as uuidv4 } from "uuid";




Vue.use(Vuex);

export default new Vuex.Store({
    // state is the data for this app
    state: {
        odrives: Object,
        odriveConfigs: Object,
        axes: Array,
        odriveServerAddress: String,
        serverConnected: Boolean,
        serverOutput: [],
        dashboards: [
            {
                name: "Start",
                component: "Start",
            },
            //{ name: "Config", id: uuidv4(), component: "Dashboard", controls: [], actions: [], plots: [] }
        ],
        timeSampleStart: 0,
        sampledProperties: [], // make this an object where the full path is a key and the value is the sampled var
        propSamples: { time: [] }, // {time: [time values], ...path: [path var values]}
        newData: false,
        sampling: false,
        currentDash: "Start",
        firstConn: false,
    },
    // mutations are functions that change the data
    mutations: {
        setDash(state, dashName) {
            state.currentDash = dashName;
        },
        setOdrives(state, odrives) {
            state.odrives = odrives;
            if (state.firstConn == false) {
                // first time we're getting odrive data, add correct config page
                let ConfigDash;
                if (state.odrives.odrive0.fw_version_minor.val == "5") {
                    ConfigDash = ConfigDashNew;
                }
                else if (state.odrives.odrive0.fw_version_minor.val == "4") {
                    ConfigDash = ConfigDashOld;
                }
                else {
                    ConfigDash = ConfigDashNew;
                }
                state.dashboards.push({
                    name: "Wizard",
                    component: "Wizard",
                });
                state.dashboards.push(ConfigDash);
                // plots will have variables associated, add them to sampled variables list
                for (const plot of ConfigDash.plots) {
                    console.log(plot);
                    for (const plotVar of plot.vars) {
                        console.log(plotVar);
                        //addsampledprop(path);
                        let path = plotVar.path;
                        if (!(path in state.sampledProperties)) {
                            let newPath = path.split('.');
                            newPath.splice(0, 1);
                            state.sampledProperties.push(newPath.join('.'));
                            state.propSamples[newPath.join('.')] = [];
                            console.log(state.propSamples);
                        }
                        for (const path of state.sampledProperties) {
                            console.log(path);
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
        setOdriveConfigs(state, odriveConfigs) {
            state.odriveConfigs = odriveConfigs;
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
            Vue.set(createNestedObject(state.odrives, payload.path), "val", payload.value);

        },
        addSampledProperty(state, path) {
            if (!(path in state.sampledProperties)) {
                let newPath = path.split('.');
                newPath.splice(0, 1);
                state.sampledProperties.push(newPath.join('.'));
                state.propSamples[newPath.join('.')] = [];
                console.log(state.propSamples);
            }
            for (const path of state.sampledProperties) {
                console.log(path);
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
                            console.log("Setting action val to " + obj.val);
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
        getOdrives(context) {
            // grab ODrive JSON from odrive_server
            axios.get(context.state.odriveServerAddress + '/api/odrives').then((response) => {
                context.commit('setOdrives', JSON.parse(JSON.stringify(response.data)));
                context.dispatch('getOdriveConfigs');
                context.dispatch('getAxes');
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
                                case "float":
                                    retObj[key] = parseFloat(parseFloat(odriveObj[key]["val"]).toFixed(3));
                                    break;
                                case "int":
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
            context.commit('setOdriveConfigs', treeParse(context.state.odrives));
        },
        getAxes(context) {
            let axes = [];
            //for each connected odrive, collect axes and display them
            for (const odrive of Object.keys(context.state.odrives)) {
                if ('axis0' in context.state.odrives[odrive]) {
                    axes.push({
                        name: `${odrive}.axis0`,
                        ref: context.state.odrives[odrive]['axis0']
                    });
                }
                if ('axis1' in context.state.odrives[odrive]) {
                    axes.push({
                        name: `${odrive}.axis1`,
                        ref: context.state.odrives[odrive]['axis1']
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
                }
            });
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
        }
    }
})