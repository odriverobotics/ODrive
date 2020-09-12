<template>
  <div class="wizard">
    <div class="wizard-container">
      <div class="wizard-nav card">
        <span class="wizard-nav-title">Wizard Pages</span>
        <span
          v-for="page in wizardPages"
          :key="page.number"
          class="wizard-link"
          v-bind:class="{'active-link': currentStep == page}"
          @click="currentStep=page"
        >{{page.link}}</span>
      </div>
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
          <button class="wizard-button card" @click="back">Back</button>
          <button class="wizard-button card" @click="finish">Finish</button>
          <button class="wizard-button card" @click="next">Next</button>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import configTemplate from "../assets/wizard/configTemplate.json";
import wizardPage from "../components/wizard/wizardPage.vue";
// import odriveEnums from "../assets/odriveEnums.json";
import { pages } from "../assets/wizard/wizard.js";

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
  computed: {
    wizardPages() {
      return pages;
    },
  },
  methods: {
    choiceHandler(e) {
      this.updateConfig(this.wizardConfig, e.configStub);
      console.log(JSON.parse(JSON.stringify(this.wizardConfig)));
    },
    updateConfig(config, configStub) {
      // iterate over keys in configStub
      if (configStub != null) {
        Object.keys(configStub).forEach((key) => {
          if (typeof configStub[key] == "object") {
            this.updateConfig(config[key], configStub[key]);
          } else {
            config[key] = configStub[key];
          }
        });
      }
    },
    next() {
      console.log("next");
      this.currentStep = pages[this.currentStep.next];
    },
    finish() {
      console.log("finish");
      this.currentStep = pages.End;
    },
    back() {
      console.log("back");
      this.currentStep = pages[this.currentStep.back];
    },
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

.wizard-nav {
  display: flex;
  flex-direction: column;
  height: fit-content;
}

.wizard-nav-title {
  text-align: center;
  font-weight: bold;
  margin-bottom: 0.4rem;
}

.wizard-link {
  margin: 0.2rem;
  cursor: pointer;
}

.active-link {
  text-decoration: underline;
}

.wizard-container {
  display: flex;
  flex-direction: row;
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