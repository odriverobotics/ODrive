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
  },
  data() {
      return {
          selected: undefined,
      }
  },
  computed: {
    value: function () {
      let keys = this.path.split('.');
      keys.shift();
      return parseFloat(getVal(keys.join('.')));
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
  },
  watch: {
      value: function(newVal) {
          this.selected = newVal;
      }
  },
  methods: {
    putVal: function (e) {
      let keys = this.path.split('.');
      keys.shift();
      console.log("Calling putVal from CtrlEnum");
      putVal(keys.join('.'), parseFloat(e.target.value));
    },
    deleteCtrl: function() {
      // commit a mutation in the store with the relevant information
      this.$store.commit("removeCtrlFromDash", {dashID: this.dashID, path: this.path});
    },
  },
  created() {
      // update parameter value on component creation
      let keys = this.path.split('.');
      keys.shift();
      fetchParam(keys.join('.'));
  
      this.selected = this.value;
      console.log(this.value);
  }
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

.ctrl-enum {
    border: none;
    border-bottom: 1px solid grey;
    background-color: var(--fg-color);
    text-align-last: right;
}
</style>