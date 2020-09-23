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