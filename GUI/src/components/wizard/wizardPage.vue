<template>
  <!-- render a list of choices horizontally in cards -->
  <div class="wizard-page card">
    <div class="wizard-title">{{title}}</div>
    <div class="wizard-choices">
      <wizardChoice
        v-for="choice in choices"
        v-bind:class="{chosen: selectedChoice == choice.text, unchosen: selectedChoice != choice.text}"
        v-tooltip.top="{
          content: choice.tooltip,
          class: 'tooltip-custom tooltip-other-custom fade-in',
          delay: 0,
          visible: choice.tooltip!=null,
        }"
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
          v-tooltip.top="{
            content: customComponent.tooltip,
            class: 'tooltip-custom tooltip-other-custom fade-in',
            delay: 0,
            visible: choice.tooltip!=null,
          }"
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
import wizardChoice from "./choices/wizardChoice.vue";
import wizardMisc from "./choices/wizardMisc.vue";
import wizardMotor from "./choices/wizardMotor.vue";
import wizardEncoderIncremental from "./choices/wizardEncoderIncremental.vue";
import wizardEncoderIncrementalIndex from "./choices/wizardEncoderIncrementalIndex.vue";
import wizardEnd from "./choices/wizardEnd.vue";
import wizardClearErrors from "./page_components/wizardClearErrors.vue";
import wizardMotorMeasure from "./page_components/wizardMotorMeasure.vue";
import wizardEncoderCal from "./page_components/wizardEncoderCal.vue";
import wizardBrake from "./choices/wizardBrake.vue";
import wizardInputFiltered from "./choices/wizardInputFiltered.vue";
import wizardInputVelRamp from "./choices/wizardInputVelRamp.vue";
import wizardInputTrajectory from "./choices/wizardInputTrajectory.vue";

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
    wizardEncoderCal,
    wizardBrake,
    wizardInputFiltered,
    wizardInputVelRamp,
    wizardInputTrajectory,
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
      this.$emit("page-comp-event", e);
    },
  },
};
</script>

<style>
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

.vue-tooltip.tooltip-custom {
  background-color:  lightyellow;/* var(--fg-color); */
  border-radius: 0px;
  box-shadow: 0 4px 8px 0 rgba(0, 0, 0, 0.4);
  font-family: "Roboto", sans-serif;
  color: black;
}

.vue-tooltip.tooltip-custom .tooltip-arrow {
  display: none;
}

.vue-tooltip.fade-in {
  opacity: 0;
  animation: fadeIn ease 0.25s;
  animation-fill-mode: both;
}

@keyframes fadeIn {
  from {opacity: 0}
  to {opacity: 1}
}
</style>