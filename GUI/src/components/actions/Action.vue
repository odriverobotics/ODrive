<template>
  <div class="card action-card">
    <button class="close-button" @click=deleteAction>X</button>
    <span class="ctrlName">{{path}}:</span>
    <div class="right">
      <input type="number" v-on:change="newVal" :placeholder="initVal"/>
      <button class="action-button close-button" @click="putVal">Go</button>
    </div>
  </div>
</template>

<script>
const axios = require("axios");

export default {
  name: "Action",
  props: {
    id: String,
    path: String,
    initVal: Number,
    dashID: String
  },
  data: function () {
    return {
      value: 0,
    };
  },
  mounted() {
    if (this.initVal !== undefined) {
      this.value = this.initVal;
    }
  },
  methods: {
    newVal: function (e) {
      this.value = parseFloat(e.target.value);
      // emit a signal - signal needs to float all the way up to Dash
      // set value of dash.actions.id.val to this.value
      // this way, action values will be kept when dashes are imported or exported
      //this.$emit('set-action-val', {id: this.id, val: this.value});
      this.$store.commit("setActionVal", {dashID: this.dashID, actionID: this.id, val: this.value});
    },
    putVal: function () {
      var params = new URLSearchParams();
      let keys = this.path.split(".");
      keys.shift();
      for (const key of keys) {
        params.append("key", key);
      }
      params.append("val", this.value);
      params.append("type", "number");
      console.log(params.toString());
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
    deleteAction: function() {
      // commit a mutation to remove this action from the dashboard
      this.$store.commit("removeActionFromDash", {dashID: this.dashID, actionID: this.id});
    }
  },
};
</script>

<style scoped>
input {
  border-style: none;
  border-bottom: 1px solid grey;
  width: 5rem;
  margin-left: 0.5rem;
  margin-right: 0.5rem;
  text-align: center;
}

.action-card {
    display: flex;
    /* border: 1px solid lightcoral; */
    box-sizing: border-box;
}

.action-button {
    margin-right: 0;
}

.right {
    margin-left: auto;
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
</style>