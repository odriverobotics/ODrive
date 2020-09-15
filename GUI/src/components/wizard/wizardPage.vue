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
        v-on:click.native="selectedChoice=choice.text;$emit('choice', {title: title, choice: choice.text, configStub: choice.configStub, hooks: choice.hooks})"
      />
      <template v-for="customComponent in customComponents">
        <component
          v-bind:class="{chosen: selectedChoice == customComponent.component, unchosen: selectedChoice != customComponent.component}"
          v-bind:is="customComponent.component"
          v-bind:data="customComponent.data"
          v-bind:config="config"
          v-bind:axis="axis"
          :key="customComponent.id"
          v-on:click.native="selectedChoice=customComponent.component"
          v-on:choice="handleCustomChoice"
        />
      </template>
    </div>
    <div class="page-components">
      <template v-for="pageComponent in pageComponents">
        <component
          v-bind:is="pageComponent.component"
          v-bind:data="pageComponent.data"
          v-bind:key="pageComponent.id"
          v-on:page-comp-event="pageCompEvent"
        />
      </template>
    </div>
  </div>
</template>

<script>
import wizardChoice from "./wizardChoice.vue";
import wizardMisc from "./wizardMisc.vue";
import wizardMotor from "./wizardMotor.vue";
import wizardEncoderIncremental from "./wizardEncoderIncremental.vue";
import wizardEncoderIncrementalIndex from "./wizardEncoderIncrementalIndex.vue";
import wizardEnd from "./wizardEnd.vue";
import wizardClearErrors from "./wizardClearErrors.vue";
import wizardMotorMeasure from "./wizardMotorMeasure.vue";

export default {
  name: "wizardPage",
  props: {
    choices: Array,
    title: String,
    customComponents: Array,
    axis: String,
    config: Object,
    pageComponents: Array,
  },
  components: {
    wizardChoice,
    wizardMisc,
    wizardMotor,
    wizardEncoderIncremental,
    wizardEncoderIncrementalIndex,
    wizardEnd,
    wizardClearErrors,
    wizardMotorMeasure,
  },
  data: function () {
    return {
      selectedChoice: undefined,
    };
  },
  methods: {
    handleCustomChoice(e) {
      console.log(e);
      this.$emit("choice", e);
    },
    pageCompEvent(e) {
      this.$emit('page-comp-event',e);
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
  flex-wrap: wrap;
}

.page-components {
  display: flex;
  flex-direction: row;
  justify-content: center;
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