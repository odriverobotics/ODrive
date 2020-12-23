<template>
  <div class="card">
    <div>
      <button class="close-button" @click=deleteCtrl>X</button>
      <span class="ctrlName">{{name}}</span>
    </div>
    <div class="slider-container">
      <input :value="displayMin" :placeholder="displayMin" v-on:change="setMin"/>
      <!-- <vue-slider v-model="value" :min="min" :max="max" :interval="interval" /> -->
      <vue-slider v-model="value" :data="data" :adsorb="true" @change="putVal"/>
      <input :value="displayMax" :placeholder="displayMax" v-on:change="setMax" />
    </div>
  </div>
</template>

<script>
import VueSlider from "vue-slider-component";
import "vue-slider-component/theme/default.css";
import { getVal, putVal, parseMath } from "../../lib/odrive_utils.js";
import { numberDisplay } from "../../lib/utils.js";

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
    },
    displayMin() {
      return numberDisplay(this.min);
    },
    displayMax() {
      return numberDisplay(this.max);
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
      let min = parseMath(e.target.value);
      if (min < this.max) {
        this.min = min;
        this.data = Array.from(Array(101), (_, i) => this.min + (this.max-this.min) / 100 * i);
        this.value = this.findNearest(this.data, this.value);
      }
    },
    setMax: function (e) {
      let max = parseMath(e.target.value);
      if (max > this.min) {
        this.max = max;
        this.data = Array.from(Array(101), (_, i) => this.min + (this.max-this.min) / 100 * i);
        this.value = this.findNearest(this.data, this.value);
      }
    },
    deleteCtrl: function() {
      // commit a mutation in the store with the relevant information
      this.$store.commit("removeCtrlFromDash", {dashID: this.dashID, path: this.path});
    },
    findNearest(data, value) {
      // find item in data closest to value
      let diff = Number.POSITIVE_INFINITY;
      let retVal = value;
      for (const val of data) {
        if (Math.abs(val - value) < diff) {
          diff = Math.abs(val - value);
          retVal = val;
        }
      }
      return retVal;
    }
  },
  mounted() {
    let initVal = () => {
      let keys = this.path.split(".");
      keys.shift(); // don't need first key here
      return parseFloat(getVal(keys.join('.')));
    };
    this.value = initVal();
    this.max = this.value * 4;
    this.min = this.value / 4;
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