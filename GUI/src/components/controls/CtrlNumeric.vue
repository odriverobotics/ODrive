<template>
  <div class="card">
    <button class="close-button" @click=deleteCtrl>X</button>
    <span class="ctrlName">{{name}}:</span>
    <div class="right">
      <span class="ctrlVal">{{value}}</span>
      <input v-if="writeAccess" type="number" v-on:change="putVal" />
    </div>
  </div>
</template>

<script>
const axios = require("axios");

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
      let keys = this.path.split(".");
      keys.shift(); // don't need first key here
      let odriveObj = this.$store.state.odrives;
      for (const key of keys) {
        odriveObj = odriveObj[key];
      }
      return parseFloat(odriveObj["val"]).toFixed(3);
    },
    name: function () {
      let keys = this.path.split(".");
      keys.shift();
      return keys.join(".");
    },
    writeAccess: function () {
      let keys = this.path.split(".");
      keys.shift(); // don't need first key here
      let odriveObj = this.odrives;
      for (const key of keys) {
        odriveObj = odriveObj[key];
      }
      return odriveObj["readonly"] === false;
    },
  },
  methods: {
    putVal: function (e) {
      var params = new URLSearchParams();
      let keys = this.path.split(".");
      keys.shift();
      for (const key of keys) {
        params.append("key", key);
      }
      params.append("val", e.target.value);
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
    deleteCtrl: function() {
      // commit a mutation in the store with the relevant information
      this.$store.commit("removeCtrlFromDash", {dashID: this.dashID, path: this.path});
    }
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
</style>