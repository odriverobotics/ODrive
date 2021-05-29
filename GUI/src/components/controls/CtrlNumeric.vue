<template>
  <div class="card">
    <button class="close-button" @click=deleteCtrl>X</button>
    <span class="ctrlName">{{name}}:</span>
    <div class="right">
      <span v-if="!writeAccess" class="ctrlVal">{{value}}</span>
      <input v-if="writeAccess" :placeholder="value" v-on:change="putVal" :value="value" spellcheck="false"/>
      <!-- <span class="unit">[{{unit}}]</span> -->
    </div>
  </div>
</template>

<script>
import { getVal, getReadonly, putVal, fetchParam, getUnit, parseMath } from "../../lib/odrive_utils.js";
import { numberDisplay } from "../../lib/utils.js";

export default {
  name: "CtrlNumeric",
  //type checking here for properties
  props: {
    path: String,
    odrives: Object,
    dashID: String
  },
  computed: {
    value: function () {
      let keys = this.path.split('.');
      keys.shift();
      let val = getVal(keys.join('.'));
      return numberDisplay(val);
    },
    name: function () {
      let keys = this.path.split(".");
      keys.shift();
      return keys.join(".");
    },
    writeAccess: function () {
      let keys = this.path.split(".");
      keys.shift(); // don't need first key here
      return getReadonly(keys.join('.')) === false;
    },
    unit() {
      let keys = this.path.split(".");
      keys.shift();
      return getUnit(this.$store.state.odrives.odrive0,keys.join('.'));
    },
  },
  methods: {
    putVal: function (e) {
      let keys = this.path.split('.');
      keys.shift();
      console.log("input recieved: " + e.target.value);
      let val = parseMath(e.target.value);
      if (val !== false) {
        putVal(keys.join('.'), val);
      }
    },
    deleteCtrl: function() {
      // commit a mutation in the store with the relevant information
      this.$store.commit("removeCtrlFromDash", {dashID: this.dashID, path: this.path});
    }
  },
  created() {
    // update parameter value on component creation
    let keys = this.path.split('.');
    keys.shift();
    fetchParam(keys.join('.'));
  },
};
</script>

<style scoped>
.ctrlVal {
  font-weight: bold;
}

input {
  width: 5rem;
  font-family: inherit;
  border-style: none;
  border-bottom: 1px solid grey;
  text-align: center;
}

.right {
  display: flex;
  flex-direction: row;
  margin-left: auto;
}

.card {
  display: flex;
}

input::-webkit-outer-spin-button,
input::-webkit-inner-spin-button {
    /* display: none; <- Crashes Chrome on hover */
    -webkit-appearance: none;
    margin: 0; /* <-- Apparently some margin are still there even though it's hidden */
}

input[type=number] {
    -moz-appearance:textfield; /* Firefox */
}

.unit {
  font-family: "Roboto Mono", monospace;
}
</style>