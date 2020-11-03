<template>
  <div class="card">
    <button class="close-button" @click=deleteCtrl>X</button>
    <span class="ctrlName">{{name}}:</span>
    <div class="right">
      <select class="ctrl-enum" v-model="selected" @change="putVal">
          <option v-for="option in options" :key="option.value" :value="option.value">
              {{ option.text }}
          </option>
      </select>
      
      <button v-if="writeAccess" v-on:click="putVal" style="padding:0; margin-right:10px; margin-left:10px;" tooltip="Write down">
        <img src="../../assets/images/saveIcon.png"  />
      </button>

      <span class="ctrlVal" >{{ value | toReadable(options) }}</span>
    </div>
  </div>
</template>

<script>
import { getVal, getReadonly, putVal, fetchParam } from "../../lib/odrive_utils.js";

export default {
  name: "CtrlNumeric",
  //type checking here for properties
  props: {
    path: String,
    odrives: Object,
    dashID: String,
    options: Array,
    parentControl: Object
  },
  data() {
      return {
          selected: undefined
      }
  },
  computed: {
    value: function () {
      return parseFloat(getVal(this.name));
    },
    name: function () {
      let keys = this.path.split(".");
      keys.shift();
      return keys.join(".");
    },
    writeAccess: function () {
      return getReadonly(this.name) === false;
    },
  },
  filters: {
    toReadable: function(val, options) {
      return options[val].text;
    }
  },
  watch: {
      selected: function(newVal) {
        this.parentControl.selectedValue = newVal;
      }
  },
  methods: {
    putVal: function () {
      console.log("Calling putVal from CtrlEnum " + this.selected);
      putVal(this.name, parseFloat(this.selected));
      fetchParam(this.name);
    },
    deleteCtrl: function() {
      // commit a mutation in the store with the relevant information
      this.$store.commit("removeCtrlFromDash", {dashID: this.dashID, path: this.path});
    },
  },
  created() {
      // update parameter value on component creation
      fetchParam(this.name);
  
      if (this.parentControl.selectedValue !== undefined) {
        this.selected = this.parentControl.selectedValue;
      } else {
        this.selected = this.value;
      }
      console.log(this.value);
  }
};
</script>

<style scoped>
.ctrlVal {
  font-weight: bold;
  min-width:180px;
  text-align: right;
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

.ctrl-enum {
    border: none;
    border-bottom: 1px solid grey;
    background-color: var(--fg-color);
    text-align-last: right;
}
</style>