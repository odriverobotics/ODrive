<template>
  <div class="card action-card">
    <button class="close-button" @click=deleteAction>X</button>
    <span class="ctrlName">{{shortPath}}:</span>
    <div class="right">
      <select class="action-enum" v-model="value" @change="newVal">
          <option v-for="option in options" :key="option.value" :value="option.value">
              {{ option.text }}
          </option>
      </select>
      <button class="action-button close-button" @click="putVal">Send</button>
    </div>
  </div>
</template>

<script>
import { putVal } from "../../lib/odrive_utils.js";

export default {
  name: "ActionEnum",
  props: {
    id: String,
    path: String,
    initVal: Number,
    dashID: String,
    options: Array,
  },
  data: function () {
    return {
      value: 0,
    };
  },
  created() {
    if (this.initVal !== undefined) {
      this.value = this.initVal;
    }
  },
  computed: {
    shortPath() {
      let keys = this.path.split('.');
      keys.shift();
      return keys.join('.');
    }
  },
  methods: {
    newVal: function (e) {
        console.log(this.selected);
      this.value = parseFloat(e.target.value);
      this.$store.commit("setActionVal", {dashID: this.dashID, actionID: this.id, val: this.value});
    },
    putVal: function () {
      let keys = this.path.split(".");
      keys.shift();
      console.log('path = ' + keys.join('.') + ", val = " + this.value);
      putVal(keys.join('.'), this.value);
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

.action-enum {
    border: none;
    border-bottom: 1px solid grey;
    background-color: var(--fg-color);
    text-align-last: right;
}
</style>