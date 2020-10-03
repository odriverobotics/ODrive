<template>
  <div class="card action-card">
    <button class="close-button" @click=deleteAction>X</button>
    <span class="ctrlName">{{shortPath}}:</span>
    <div class="right">
      <input v-on:change="newVal" :placeholder="initVal"/>
      <button class="action-button close-button" @click="putVal">Send</button>
    </div>
  </div>
</template>

<script>
import { putVal } from "../../lib/odrive_utils.js";

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
  computed: {
    shortPath() {
      let keys = this.path.split('.');
      keys.shift();
      return keys.join('.');
    }
  },
  methods: {
    newVal: function (e) {
      let input = e.target.value;
      let allowedChars = "0123456789eE/*-+.()";
      let send = true;
      for (const c of input) {
        if (!allowedChars.includes(c)) {
          send = false;
        }
      }
      if (send) {
        this.value = eval(input);
        console.log("input = " + input + ", val = " + this.value);
      }
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