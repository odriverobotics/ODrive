<template>
  <div class="card wizard-choice">
    <div class="left">
      <span>Brake resistor value =</span>
      <input type="number" v-on:change="setBR" placeholder="Change Me!" />
      <span> Ohms</span>
    </div>
  </div>
</template>

<script>
export default {
  name: "wizardBrake",
  props: {
    data: Object,
  },
  data: function () {
    return {
      brake_resistance: undefined,
    };
  },
  methods: {
    setBR(e) {
      console.log("from setBR " + e.target.value);
      this.brake_resistance = parseFloat(e.target.value);
      let configStub = undefined;
      configStub = {
        config: {
          brake_resistance: this.brake_resistance,
        },
      };
      this.$emit("choice", {
        choice: "Brake Resistor",
        configStub: configStub,
        hooks: [],
      });
    },
  },
};
</script>

<style scoped>
.wizard-choice {
  display: flex;
  flex-direction: column;
  margin: 2rem;
}
.left {
  margin-left: auto;
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

input[type="number"] {
  -moz-appearance: textfield; /* Firefox */
}
</style>