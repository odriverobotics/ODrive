<template>
  <div class="card">
    <div>
      <button class="close-button" @click=deleteCtrl>X</button>
      <span class="ctrlName">{{name}}</span>
    </div>
    <div style="text-align:center; position:relative; right:0px;">
      <span>{{ parentControl.sliderValue | applyPrecision }}</span>
      <button v-if="writeAccess" v-on:click="putVal" style="padding:1; margin-right:10px; margin-left:10px;" tooltip="Write down" >
        <img src="../../assets/images/saveIcon.png"  />
      </button>
      <span class="ctrlVal">{{ value }}</span>
    </div>
    <div class="slider-container">
      <input type="number" :value="parentControl.min" v-on:change="setMin"/>
      <vue-slider 
        v-model="parentControl.sliderValue" :min="parentControl.min" :max="parentControl.max" :interval="sliderInterval" :silent="true"
        @change="putVal" @drag-end="putVal" @dragging="updateSliderValue" 
        tooltip="none" tooltipPlacement="bottom" :lazy="true"/>
      <input type="number" :value="parentControl.max" v-on:change="setMax" />
    </div>
  </div>
</template>

<script>
import VueSlider from "vue-slider-component";
import "vue-slider-component/theme/default.css";
import { getVal, putVal, fetchParam, getReadonly } from "../../lib/odrive_utils.js";

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
    parentControl: Object
  },
  data: function () {
    return {
      sliderInterval:0,
      intervalId:undefined
    };
  },
  filters: {
    applyPrecision: function (value) {
      if (typeof value === "string") {
        value = parseFloat(value);
      }
      return value.toFixed(3);
    }
  },
  computed: {
    value: function () {
      return parseFloat(getVal(this.name)).toFixed(3);
    },
    name: function () {
      let keys = this.path.split(".");
      keys.shift();
      return keys.join(".");
    },
    writeAccess: function () {
      return getReadonly(this.name) === false;
    },
  },
  methods: {
    putVal: function () {
      putVal(this.name, this.parentControl.sliderValue);
    },
    updateSliderValue: function(value) {
      this.parentControl.sliderValue = value;
    },
    setMin: function (e) {
      this.parentControl.min = parseFloat(e.target.value);
      this.updateSliderInterval();
    },
    setMax: function (e) {
      this.parentControl.max = parseFloat(e.target.value);
      this.updateSliderInterval();
    },
    deleteCtrl: function() {
      // commit a mutation in the store with the relevant information
      this.$store.commit("removeCtrlFromDash", {dashID: this.dashID, path: this.path});
    },
    updateSliderInterval: function() {
      var step = (this.parentControl.max - this.parentControl.min) / 100.0;
      this.sliderInterval =  Math.round(step * 10000) / 10000;
      console.log(this.sliderInterval + ", " + typeof this.sliderInterval);
    }
  },
  beforeMount() {
    fetchParam(this.name);

    if (this.parentControl.min === undefined || this.parentControl.max === undefined || this.parentControl.sliderValue === undefined) {
      this.parentControl.max = parseFloat((this.value * 4).toFixed(1));
      this.parentControl.min = parseFloat((this.value / 4).toFixed(1));
      this.updateSliderInterval();
      this.parentControl.sliderValue = parseFloat(this.value);
    }
    this.updateSliderInterval();

    let update = () => {
      fetchParam(this.name);
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