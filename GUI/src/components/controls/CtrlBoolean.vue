<template>
  <div class="card">
    <button class="close-button" @click=deleteCtrl>X</button>
    <span class="ctrlName">{{name}}:</span>
    <div class="right">
      <span class="ctrlVal">{{value}}</span>
      <input
        class="ctrlInput"
        v-if="writeAccess"
        type="checkbox"
        v-bind:value="value"
        @click="putVal"
      />
    </div>
  </div>
</template>

<script>
const axios = require("axios");

export default {
  name: "CtrlBoolean",
  //type checking here for properties
  props: {
    path: String,
    odrives: Object,
    dashID: String,
  },
  computed: {
    value: function () {
      let keys = this.path.split(".");
      keys.shift(); // don't need first key here
      let odriveObj = this.odrives;
      for (const key of keys) {
        odriveObj = odriveObj[key];
      }
      return odriveObj["val"];
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
      return odriveObj["readonly"] == false;
    },
  },
  methods: {
    putVal: function (e) {
      console.log(e.target.checked);
      var params = new URLSearchParams();
      let keys = this.path.split(".");
      keys.shift();
      for (const key of keys) {
        params.append("key", key);
      }
      params.append("val", e.target.checked);
      params.append("type", "boolean");
      console.log(params.toString());
      let request = {
        params: params,
      };
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

.ctrlInput {
  margin-left: 10px;
}

.right {
  display: flex;
  flex-direction: row;
  margin-left: auto;
}

.card {
  display: flex;
}
</style>