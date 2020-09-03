<template>
  <div class="card" @click.self="executeFunction">
    <button class="close-button" @click=deleteCtrl>X</button>
    <button class="execute">{{name}}()</button>
  </div>
</template>

<script>
const axios = require("axios");

export default {
  name: "CtrlFunction",
  //type checking here for properties
  props: {
    path: String,
    odrives: Object,
    dashID: String,
  },
  computed: {
    name: function() {
      let keys = this.path.split(".");
      keys.shift();
      return keys.join(".");
    }
  },
  methods: {
    putVal: function(e) {
      var params = new URLSearchParams();
      let keys = this.path.split(".");
      keys.shift();
      for (const key of keys) {
        params.append("key", key);
      }
      params.append("val", e.target.value);
      console.log(params.toString());
      let request = {
        params: params
      };
      console.log(request);
      axios.put(
        this.$store.state.odriveServerAddress + "/api/property",
        null,
        request
      );
    },
    executeFunction: function(e) {
      //execute this function on the odrive
      console.log(e);
      var params = new URLSearchParams();
      let keys = this.path.split(".");
      keys.shift();
      for (const key of keys) {
        params.append("key", key);
      }
      console.log(params.toString());
      let request = {
        params: params
      };
      axios.put(
        this.$store.state.odriveServerAddress + "/api/function",
        null,
        request
      );
    },
    deleteCtrl: function() {
      // commit a mutation in the store with the relevant information
      this.$store.commit("removeCtrlFromDash", {dashID: this.dashID, path: this.path});
    }
  }
};
</script>

<style scoped>
.card {
  background-color: lightcyan;
}

.execute {
  color: black;
  font-family: "Roboto Mono", monospace;
  margin: 0;
  padding: 0;
  background-color: rgba(0, 0, 0, 0);
}

.card:active {
  background-color: lightblue;
}
</style>