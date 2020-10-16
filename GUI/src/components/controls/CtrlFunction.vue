<template>
  <div class="card" @click.self="executeFunction">
    <button class="close-button" @click=deleteCtrl>X</button>
    <button class="execute" @click="executeFunction">{{name}}()</button>
  </div>
</template>

<script>
import { callFcn } from "../../lib/odrive_utils.js";

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
    executeFunction: function() {
      //execute this function on the odrive
      let keys = this.path.split(".");
      keys.shift();
      callFcn(keys.join('.'));
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