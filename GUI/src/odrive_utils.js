import store from "./store.js";
const axios = require("axios");

// helper functions and utilities for getting ODrive values

// given a path like "odrive0.axis0.config.blah", return the value
export function getParam(path) {
    let keys = path.split('.');
    let odriveObj = store.state.odrives;
    for (const key of keys) {
        odriveObj = odriveObj[key];
    }
    return odriveObj;
}

// wrapper for val field
export function getVal(path) {
    return getParam(path + ".val");
}

// wrapper for readonly field
export function getReadonly(path) {
    return getParam(path + '.readonly');
}

// using axios, send an http request to change an ODrive value
export function putVal(path, value) {
    var params = new URLSearchParams();
    let keys = path.split(".");
    for (const key of keys) {
        params.append("key", key);
    }
    params.append("val", value);
    params.append("type", typeof value);
    let request = {
        params: params,
    };
    console.log(request);
    axios.put(
        store.state.odriveServerAddress + "/api/property",
        null,
        request
    );
}

// path is path to function, args is list of parameters
export function callFcn(path, args) {
    var params = new URLSearchParams();
    let keys = path.split(".");
    for (const key of keys) {
        params.append("key", key);
    }
    if (args) {
        for (const arg of args) {
            params.append("arg", arg);
        }
    }
    console.log(params.toString());
    let request = {
        params: params
    };
    axios.put(
        store.state.odriveServerAddress + "/api/function",
        null,
        request
    );
}

export function odriveMinorVersion(odrive) {
    // given an odrive object, return the minor version
    return odrive.fw_version_minor.val;
}

export function getUnit(odrive, path) {
    let unit = undefined;
    for ( const key of Object.keys(odriveUnits[odriveMinorVersion(odrive)]) ) {
        if (path.includes(key)) {
          unit = odriveUnits[odriveMinorVersion(odrive)][key];
        }
    }
    return unit;
}

// standins for wizard and gui unit displays
// TODO - convert between odrive api units and user-defined units like degrees and rpm
export let odriveUnits = {
    "4": {
        "config.brake_resistance": "ohms",
        "controller.pos_setpoint": "counts",
        "controller.vel_setpoint": "counts/s",
        "controller.current_setpoint": "amps",
        "controller.config.vel_limit": "counts/s",
        "controller.config.vel_ramp_rate": "counts/s^2",
        "encoder.pos_estimate": "counts",
        "encoder.vel_estimate": "counts/s",
        "trap_traj.config.vel_limit": "counts/s",
        "trap_traj.config.accel_limit": "counts/s^2",
        "trap_traj.config.decel_limit": "counts/s^2",
        "motor.config.phase_resistance": "ohms",
        "motor.config.phase_inductance": "henries",
    },
    "5": {
        "config.brake_resistance": "ohms",
        "controller.input_pos": "turns",
        "controller.input_vel": "turns/s",
        "controller.current_setpoint": "amps",
        "controller.config.vel_limit": "turns/s",
        "controller.config.vel_ramp_rate": "turns/s^2",
        "encoder.pos_estimate": "turns",
        "encoder.vel_estimate": "turns/s",
        "trap_traj.config.vel_limit": "turns/s",
        "trap_traj.config.accel_limit": "turns/s^2",
        "trap_traj.config.decel_limit": "turns/s^2",
        "motor.config.phase_resistance": "ohms",
        "motor.config.phase_inductance": "henries",
    }
}