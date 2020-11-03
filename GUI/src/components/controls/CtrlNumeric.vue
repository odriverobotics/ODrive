<template>
  <div class="card">
    <button class="close-button" @click=deleteCtrl>X</button>
    <div class="lineDiv">
      <span class="ctrlName">{{name}}:</span>
    </div>
    <div class="lineDiv right">
      <input v-if="writeAccess" v-on:change="putVal" v-model="setValue" />
      <button v-if="writeAccess" v-on:click="putVal" style="padding:0; margin-right:10px; margin-left:10px;" tooltip="Write down">
        <img src="../../assets/images/saveIcon.png"  />
      </button>
      <span class="ctrlVal" >{{value}}</span>
      <span class="unit" v-if="unit !== undefined">[{{unit}}]</span>
    </div>
  </div>
</template>

<script>
import { getVal, getReadonly, putVal, fetchParam, getUnit, parseMath } from "../../lib/odrive_utils.js";

export default {
  name: "CtrlNumeric",
  //type checking here for properties
  props: {
    path: String,
    odrives: Object,
    dashID: String
  },
  data: function(){
    return {
      setValue:"0",
      intervalId:undefined
    }
  },
  computed: {
    value: function () {
      let keys = this.path.split('.');
      keys.shift();
      return parseFloat(getVal(keys.join('.'))).toFixed(3);
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
    putVal: function () {
      let keys = this.path.split('.');
      keys.shift();
      console.log("input recieved: " + this.setValue);
      let val = parseMath(this.setValue);
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
    fetchParam(this.name);
    this.setValue = this.value;

    let update = () => {
      fetchParam(this.name);
    }

    this.intervalId = setInterval(update, 1000);

    update();
  },
  beforeDestroy() {
      clearInterval(this.intervalId);
  }
};
</script>

<style scoped>

.lineDiv {
  height: 25px; 
  line-height: 25px;
}

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