<template>
  <div class="wizard">
    <div class="wizard-container">
      <div class="wizard-nav card">
        <span class="wizard-nav-title">Wizard Pages</span>
        <span
          v-for="page in wizardPages"
          :key="page.title"
          class="wizard-link"
          v-bind:class="{'active-link': currentStep == page}"
          @click="currentStep = page; choiceMade = false"
        >{{page.link}}</span>
      </div>
      <div class="wizard-page">
        <wizardPage
          :choices="currentStep.choices"
          :customComponents="currentStep.customComponents"
          :pageComponents="currentStep.pageComponents"
          :title="currentStep.title"
          :config="wizardConfig"
          v-on:choice="choiceHandler"
          v-on:page-comp-event="pageEventHandler"
        />
        <div class="wizard-controls">
          <!-- show breadcrumbs, back, apply, next buttons -->
          <button class="wizard-button card" @click="back">Back</button>
          <button class="wizard-button card" @click="finish">Finish</button>
          <button
            class="wizard-button card"
            v-bind:class="{'next-green': choiceMade}"
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
      choiceMade: false,
    };
  },
  computed: {
    wizardPages() {
      return pages;
    },
  },
  methods: {
    pageEventHandler(e) {
      this.choiceMade = false;
      if (e.data == "motor calibration") {
        // set a timeout to grab axis resistance and inductance values
        setTimeout(() => {
          // check for error during calibration
          if (
            this.$store.state.odrives.odrive0.axis0.error.val == "0" &&
            this.$store.state.odrives.odrive0.axis1.error.val == "0"
          ) {
            let configStub = undefined;
            let inductance;
            let resistance;
            let keys_L = [
              "odrive0",
              e.axis,
              "motor",
              "config",
              "phase_inductance",
            ];
            let keys_R = [
              "odrive0",
              e.axis,
              "motor",
              "config",
              "phase_resistance",
            ];
            let odriveObj = this.$store.state.odrives;
            for (const key of keys_L) {
              odriveObj = odriveObj[key];
            }
            inductance = parseFloat(odriveObj["val"]);

            odriveObj = this.$store.state.odrives;
            for (const key of keys_R) {
              odriveObj = odriveObj[key];
            }
            resistance = parseFloat(odriveObj["val"]);
            if (e.axis == "axis0") {
              configStub = {
                axis0: {
                  motor: {
                    config: {
                      phase_resistance: resistance,
                      phase_inductance: inductance,
                    },
                  },
                },
              };
            } else if (e.axis == "axis1") {
              configStub = {
                axis1: {
                  motor: {
                    config: {
                      phase_resistance: resistance,
                      phase_inductance: inductance,
                    },
                  },
                },
              };
            }
            this.choiceHandler({
              choice: "Motor Calibration",
              configStub: configStub,
              hooks: [],
            });
          }
        }, 6000);
      }
    },
    choiceHandler(e) {
      // apply static configStub
      this.updateConfig(this.wizardConfig, e.configStub);

      // run hooks
      for (const fn of e.hooks) {
        this.wizardConfig = fn(this.wizardConfig);
      }

      // ugly, but a special case.
      // for motors, wait for calibration to finish before giving the green light
      if (
        e.choice != "ODrive D5065" &&
        e.choice != "ODrive D6374" &&
        e.choice != "Other motor"
      ) {
        this.choiceMade = true;
      }
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
      if (this.choiceMade == true) {
        this.currentStep = pages[this.currentStep.next];
        this.choiceMade = false;
      }
    },
    finish() {
      console.log("finish");
      this.currentStep = pages.End;
    },
    back() {
      console.log("back");
      this.currentStep = pages[this.currentStep.back];
      this.choiceMade = false;
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

.next-green {
  color: green;
  font-weight: bold;
}
</style>