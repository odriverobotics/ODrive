<template>
  <div class="wizard">
    <div class="wizard-container">
      <div class="wizard-page">
        <wizardPage
          :choices="currentStep.choices"
          :customComponents="currentStep.customComponents"
          :title="currentStep.title"
          :config="wizardConfig"
          v-on:choice="choiceHandler"
        />
        <div class="wizard-controls">
          <!-- show breadcrumbs, back, apply, next buttons -->
          <button
            class="wizard-button card"
            @click="back"
          >Back</button>
          <button
            class="wizard-button card"
            @click="finish"
          >Finish</button>
          <button
            class="wizard-button card"
            @click="next"
          >Next</button>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import configTemplate from "../assets/wizard/configTemplate.json";
import wizardPage from "../components/wizard/wizardPage.vue";
// import odriveEnums from "../assets/odriveEnums.json";
import {pages} from "../assets/wizard/wizard.js";

// see wizard.js for what wizard pages exist and what they contain

export default {
  name: "Wizard",
  components: {
    wizardPage,
  },
  props: [],
  data: function () {
    return {
      currentStep: pages.ODrive,
      wizardConfig: configTemplate,
    };
  },
  methods: {
    applyConfig() {
      // send config parameters that aren't null to the connected odrive!
      console.log("attempting to apply config...");
      console.log(JSON.parse(JSON.stringify(this.wizardConfig)));
    },
    choiceHandler(e){
      this.wizardConfig = this.updateConfig(this.wizardConfig, e);
    },
    updateConfig(oldConfig, choice){
      // merge choice.configStub into wizardConfig
    },
    next(){
      console.log("next");
      this.currentStep = pages[this.currentStep.next];
    },
    finish(){
      console.log("finish");
      this.currentStep = pages.End;
    },
    back(){
      console.log("back");
      this.currentStep = pages[this.currentStep.back];
    }
  },
};
</script>

<style scoped>
.wizard {
  background-color: var(--bg-color);
  height: 100vh;
  max-height: 100vh;
  width: 100vw;
  padding-top: var(--top-height);
  padding-bottom: var(--bottom-height);
}

.wizard-container {
  display: flex;
  flex-direction: column;
  height: 95vh;
}

.wizard-page {
  margin: auto;
}

.wizard-controls {
  margin-top: 0;
  display: flex;
  justify-content: center;
}

.active {
  background-color: var(--fg-color);
}

.inactive {
  background-color: var(--bg-color);
  color: white;
}

.wizard-button:active {
  background-color: var(--bg-color);
}
</style>