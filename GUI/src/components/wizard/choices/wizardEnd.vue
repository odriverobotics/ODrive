<template>
  <div class="card">
    <div class="diff" v-for="diff in configDiffs" :key="diff.path">
      <span class="diff-path">{{diff.path}}: </span>
      <span class="diff-old">{{diff.oldVal}}</span>
      <span class="diff-seperator"> ⮕ </span>
      <span class="diff-new">{{diff.newVal}}</span>
    </div>
    <button class="wizard-button card" @click="applyConfig">Apply</button>
  </div>
</template>

<script>
import { enumVars } from "../../../assets/wizard/wizard.js";
const axios = require("axios");

export default {
  name: "wizardEnd",
  props: {
    config: Object,
    axis: String,
    // actual odrive config is in this.$store (Vuex store)
  },
  data: function() {
    return {
      configDiffs: [], // array of objects {path, oldval, newval}
      flatpaths: [],
      path: [],
    }
  },
  created() {
    // flatten config tree into array of full variable paths
    this.pathFromTree(this.config);
    for (const path of this.flatpaths){
      console.log(path);
    }
    for (const path of this.flatpaths) {
      let odrvObj = this.$store.state.odrives.odrive0;
      let configObj = this.config;
      for (const key of path.split('.')){
        odrvObj = odrvObj[key];
        configObj = configObj[key];
      }
      if (configObj != null) {
        let keys = path.split('.');
        if (Object.keys(enumVars).includes(keys[keys.length - 1])){
          // print old enum and new enum strings
          console.log(enumVars[keys[keys.length-1]]);
          this.configDiffs.push({path: path, oldVal: enumVars[keys[keys.length-1]][odrvObj["val"]], newVal: enumVars[keys[keys.length-1]][configObj]})
        }
        else {
          // display numeric or boolean value
          console.log("oldVal is " + parseFloat(odrvObj["val"]) + " path is " + path);
          if (Number.isInteger(parseFloat(odrvObj["val"]))){
            this.configDiffs.push({path: path, oldVal: parseFloat(odrvObj["val"]), newVal: configObj});
          }
          else {
            this.configDiffs.push({path: path, oldVal: parseFloat(odrvObj["val"]).toExponential(3), newVal: configObj.toExponential(3)});
          }
        }
      }
    }
  },
  methods: {
    pathFromTree(tree){
      // array path
      for (const key of Object.keys(tree)) {
        this.path.push(key);
        if (tree[key] != null && typeof tree[key] == "object") {
          this.pathFromTree(tree[key]);
        }
        else {
          this.flatpaths.push(this.path.join('.'));
        }
        this.path.pop();
      }
    },
    applyConfig(){
      // flatpaths will already be populated at this point
      for (const diff of this.configDiffs) {
        this.putVal(diff.path, diff.newVal);
        console.log("applying " + diff.newVal + " to " + diff.path);
      }
    },
    putVal(path, val) {
      var params = new URLSearchParams();
      params.append("key", "odrive0");
      let keys = path.split(".");
      for (const key of keys) {
        params.append("key", key);
      }
      params.append("val", val);
      params.append("type", typeof val);
      console.log(path + " = " + " type = " + typeof val);
      //console.log(params.toString());
      let request = {
        params: params,
      };
      console.log(request);
      axios.put(
        this.$store.state.odriveServerAddress + "/api/property",
        null,
        request
      );
    },
  }
}
</script>

<style scoped>
.diff {
  text-align: left;
}

.diff-path {
  font-family: "Roboto Mono", monospace;
}

.diff-old {
  font-weight: bold;
}

.diff-new {
  font-weight: bold;
  color: green;
}

.wizard-button:active {
  background-color: var(--bg-color);
}
</style>