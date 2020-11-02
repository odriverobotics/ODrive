<template>
  <div class="card">
    <button class="close-button" @click=deleteCtrl>X</button>
    <span class="ctrlName">{{name}}:</span>
    <div class="right">
        <span v-if="value" class="true">True</span>
        <span v-if="!value" class="false">False</span>
    </div>
  </div>
</template>

<script>
import { getVal, fetchParam } from "../../lib/odrive_utils.js";

export default {
  name: "WatchBoolean",
  //type checking here for properties
  props: {
    path: String,
    odrives: Object,
    dashID: String
  },
  data() {
      return {
          value:false,
          intervalId:undefined
      }
  },
  computed: {
    name: function () {
      let keys = this.path.split(".");
      keys.shift();
      return keys.join(".");
    }
  },
  methods: {
    deleteCtrl: function() {
      // commit a mutation in the store with the relevant information
      this.$store.commit("removeCtrlFromDash", {dashID: this.dashID, path: this.path});
    },
  },
  created() {
      let update = () => {
        fetchParam(this.name);
        this.value = getVal(this.name);
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
.right {
  display: flex;
  flex-direction: row;
  margin-left: auto;
}

.card {
  display: flex;
}

.true {
    color:green;
}

.false {
    color:red;
}
</style>