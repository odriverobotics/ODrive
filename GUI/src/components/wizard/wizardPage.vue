<template>
  <!-- render a list of choices horizontally in cards -->
  <div class="wizard-page card">
    <div class="wizard-title">{{title}}</div>
    <div class="wizard-choices">
      <wizardChoice
        v-for="choice in choices"
        v-bind:class="{chosen: selectedChoice == choice.text, unchosen: selectedChoice != choice.text}"
        :key="choice.id"
        :imageUrl="choice.imageURL"
        :text="choice.text"
        v-on:click.native="selectedChoice=choice.text;$emit('choice', {title: title, choice: choice.text, config: choice.config})"
      />
      <template v-for="customComponent in customComponents">
        <component
          v-bind:class="{chosen: selectedChoice == customComponent.compName, unchosen: selectedChoice != customComponent.compName}"
          v-bind:is="customComponent.compName"
          v-bind:axis="axis"
          :key="customComponent.id"
          v-on:click.native="selectedChoice=customComponent.compName"
          v-on:choice="handleCustomChoice"
        />
      </template>
    </div>
  </div>
</template>

<script>
import wizardChoice from "./wizardChoice.vue";
import wizardLimits from "./wizardLimits.vue";
import wizardMotorCustom from "./wizardMotorCustom.vue";

export default {
  name: "wizardPage",
  props: {
    choices: Array,
    title: String,
    customComponents: Array,
    axis: String,
  },
  components: {
    wizardChoice,
    wizardLimits,
    wizardMotorCustom,
  },
  data: function () {
    return {
      selectedChoice: undefined,
    };
  },
  methods: {
    handleCustomChoice(e) {
      console.log(e);
      this.$emit('choice', e);
    }
  },
};
</script>

<style scoped>
.wizard-page {
  display: flex;
  flex-direction: column;
  text-align: center;
}

.wizard-choices {
  display: flex;
  flex-direction: row;
}

.chosen {
  border: 2px solid black;
}

.unchosen {
  border: 2px solid transparent;
}

.wizard-title {
  font-weight: bold;
}
</style>