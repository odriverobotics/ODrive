<template>
  <div class="card">
    <div>
      <button class="close-button" @click=deleteCtrl>X</button>
      <span class="ctrlName">{{name}}</span>
    </div>
    <div class="slider-container">
      <input type="number" :value="min" v-on:change="setMin"/>
      <!-- <vue-slider v-model="value" :min="min" :max="max" :interval="interval" /> -->
      <vue-slider v-model="value" :data="data" @change="putVal"/>
      <input type="number" :value="max" v-on:change="setMax" />
    </div>
  </div>
</template>

<script>
import VueSlider from "vue-slider-component";
import "vue-slider-component/theme/default.css";
import { getVal, putVal } from "../../lib/odrive_utils.js";

export default {
  name: "CtrlSlider",
  components: {
    VueSlider,
  },
  //type checking here for properties
  props: {
    path: String,
    odrives: Object,
    dashID: String,
  },
  data: function () {
    return {
      value: 0,
      min: 0,
      max: 1,
      data: []
    };
  },
  computed: {
    name: function () {
      let keys = this.path.split(".");
      keys.shift();
      return keys.join(".");
    },
    interval: function () {
      return (this.max - this.min) / 100;
    },
    sliderData: function () {
      let interval = (this.max - this.min) / 100;
      return Array.from(Array(101), (_, i) => this.min + interval * i);
    }
  },
  methods: {
    putVal: function (value) {
      console.log(value);
      let keys = this.path.split(".");
      keys.shift();
      putVal(keys.join('.'), value);
    },
    setMin: function (e) {
      this.min = parseFloat(e.target.value);
      this.data = Array.from(Array(101), (_, i) => this.min + (this.max-this.min) / 100 * i);
    },
    setMax: function (e) {
      this.max = parseFloat(e.target.value);
      this.data = Array.from(Array(101), (_, i) => this.min + (this.max-this.min) / 100 * i);
    },
    deleteCtrl: function() {
      // commit a mutation in the store with the relevant information
      this.$store.commit("removeCtrlFromDash", {dashID: this.dashID, path: this.path});
    }
  },
  mounted() {
    let initVal = () => {
      let keys = this.path.split(".");
      keys.shift(); // don't need first key here
      return getVal(keys.join('.'));
    };
    this.value = initVal();
    this.max = parseFloat((this.value * 4).toFixed(3));
    this.min = parseFloat((this.value / 4).toFixed(3));
    this.data = Array.from(Array(101), (_, i) => this.min + (this.max-this.min) / 100 * i);
  },
};
</script>

<style scoped>
.ctrlVal {
  font-weight: bold;
}

.slider-container {
  display: flex;
  flex-direction: row;
  margin-top: 0.4rem;
}

.vue-slider {
  flex-grow: 3;
  margin-left: 0.4rem;
  margin-right: 0.4rem;
  z-index: 0;
}

input {
  width: 5rem;
  font-family: inherit;
  border-style: none;
  border-bottom: 1px solid grey;
  text-align: center;
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