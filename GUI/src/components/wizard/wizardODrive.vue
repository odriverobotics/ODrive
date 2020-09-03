<template>
  <div class="wizardODrive card">
    <div class="title">What ODrive do you have?</div>
    <div class="choices">
      <div class="choice card" v-bind:class="{ unchosen: !select_24V, chosen: select_24V}" @click="select('24V')">
        <img alt="Odrive 24V" src="../../assets/images/24v_200_200.png" />
        ODrive v3.6 24V
      </div>
      
      <div class="choice card" v-bind:class="{ unchosen: !select_56V, chosen: select_56V}" @click="select('56V')">
        <img alt="Odrive 56V" src="../../assets/images/56v_200_200.png" />
        ODrive v3.6 56V
      </div>
      
    </div>
  </div>
</template>

<script>
export default {
  name: "wizardODrive",
  data: function() {
    return {
      select_24V: false,
      select_56V: false,
    }
  },
  methods: {
    select(choice) {
      this.select_24V = false;
      this.select_56V = false;
      if (choice == '24V') {
        this.select_24V = true;
        this.$emit('choice-odrive-24v');
      }
      if (choice == '56V') {
        this.$emit('choice-odrive-56v');
        this.select_56V = true;
      }
    }
  }
};
</script>

<style scoped>
* {
  box-sizing: border-box;
}

.wizardODrive {
  display: flex;
  flex-direction: column;
  text-align: center;
}

.choices {
  display: flex;
  flex-direction: row;
}

.choice {
  text-align: center;
  display: flex;
  flex-direction: column;
  margin: 2rem;
}

.chosen {
  border: 2px solid black;
}

.unchosen {
  border: 2px solid transparent;
}

img {
  padding: 0px;
  margin: auto;
}
</style>