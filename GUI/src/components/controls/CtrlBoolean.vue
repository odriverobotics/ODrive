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
        :value="value"
        :checked="value"
        @click="putVal"
      />
    </div>
  </div>
</template>

<script>
import { getVal, getReadonly, putVal, fetchParam } from "../../lib/odrive_utils.js";

export default {
  name: "CtrlBoolean",
  //type checking here for properties
  props: {
    path: String,
    odrives: Object,
    dashID: String,
  },
  data() {
    return {
    }
  },
  computed: {
    value: function () {
      let keys = this.path.split(".");
      keys.shift(); // don't need first key here
      return getVal(keys.join('.'));
    },
    name: function () {
      let keys = this.path.split(".");
      keys.shift();
      return keys.join(".");
    },
    writeAccess: function () {
      let keys = this.path.split(".");
      keys.shift(); // don't need first key here
      return getReadonly(keys.join('.')) == false;
    },
  },
  methods: {
    putVal: function (e) {
      let keys = this.path.split('.');
      keys.shift();
      putVal(keys.join('.'), e.target.checked);
      fetchParam(keys.join('.'));
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
  }
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