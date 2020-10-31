<template>
  <div class="wizard">
    <div class="wizard-container">
      <div class="wizard-nav card">
        <span class="wizard-nav-title">Wizard Pages</span>
        <span
          v-for="page in wizardPages"
          :key="page.title"
          class="wizard-link"
          :class="{
            'active-link': currentStep == page,
            'finished-page': page.choiceMade,
          }"
          >{{ page.link }}</span
        >
      </div>
      <div class="wizard-page">
        <wizardPage
          :choices="currentStep.choices"
          :customComponents="currentStep.customComponents"
          :pageComponents="currentStep.pageComponents"
          :title="currentStep.title"
          :config="wizardConfig"
          :calibrating="calibrating"
          :calStatus="calStatus"
          v-on:choice="choiceHandler"
          v-on:undo-choice="undoChoice"
          v-on:page-comp-event="pageEventHandler"
        />
        <div class="wizard-controls">
          <!-- show breadcrumbs, back, apply, next buttons -->
          <button class="wizard-button card" @click="back">Back</button>
          <button class="wizard-button card" @click="finish">Finish</button>
          <button
            class="wizard-button card"
            :class="{ 'next-green': choiceMade }"
            v-tooltip.right="{
              content: currentStep.nextTooltip,
              class: 'tooltip-custom tooltip-other-custom fade-in',
              delay: 0,
              visible: !choiceMade && currentStep.nextTooltip != null,
            }"
            @click="next"
          >
            Next
          </button>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import configTemplate from "../assets/wizard/configTemplate.json";
import wizardPage from "../components/wizard/wizardPage.vue";
import odriveEnums from "../assets/odriveEnums.json";
import { pages } from "../assets/wizard/wizard.js";
import {
  getVal,
  putVal,
  fetchParam,
  clearErrors,
  motorCalibration, 
  encoderCalibration
} from "../lib/odrive_utils.js";
import {wait} from "../lib/utils.js"

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
      calibrating: false, // for indicating that calibration is in progress
      calStatus: undefined, // for indicating the status of a calibration attempt
      odrive: "odrive0.",
    };
  },
  computed: {
    wizardPages() {
      return pages;
    },
  },
  methods: {
    // certain actions, like starting motor or encoder calibration,
    // require special handling. This function is used for those events
    async pageEventHandler(e) {
      this.choiceMade = false;
      if (e.data == "motor calibration") {
        let apply = () => {
          let configStub = {};
          configStub[e.axis] = {
            motor: {
              config: {
                phase_resistance: getVal(this.odrive + e.axis + ".motor.config.phase_resistance"),
                phase_inductance: getVal(this.odrive + e.axis + ".motor.config.phase_inductance"),
              }
            }
          }
          this.choiceHandler({
            choice: "Motor Calibration",
            configStub: configStub,
            hooks: [],
          });
          this.choiceMade = true;
        };
        let updateCalInfo = () => {
          // parse error code, update calibration information
          let motorError = getVal(this.odrive + e.axis + ".motor.error");
          if (
            motorError == odriveEnums.MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE ||
            motorError == odriveEnums.MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE
          ) {
            //we got the expected calibration error
            console.log("Calibration failed.");
            this.calStatus = false;
          }
        };

        // result is motor.error
        this.calibrating = true;
        let result = await motorCalibration(this.odrive, e.axis);
        this.calibrating = false;
        fetchParam(this.odrive + e.axis + '.motor.is_calibrated');
        // no error, we're good!
        if (result == 0) {
          wait(250);
          apply();
          this.calStatus = true;
        } else {
          console.log("motor cal error" + result.data);
          this.calibrating = false;
          this.choiceMade = false;
          // clear errors, display info.
          updateCalInfo();
          clearErrors(this.odrive, e.axis);
        }
      } else if (e.data == "encoder calibration") {
        // get old CPR
        // apply CPR from this.wizardConfig
        // start calibration
        // wait for cal to finish
        // set odrive cpr back to oldVal
        let oldCPR = getVal(this.odrive + e.axis + ".encoder.config.cpr");
        console.log("oldCPR = " + oldCPR);
        console.log("axis = " + e.axis);

        // set up the calib_scan_distance to a value that will work
        let pp = this.wizardConfig[e.axis].motor.config.pole_pairs;

        // smallest multiple of 4pi that is bigger than pole_pairs * 2pi
        let scan_distance = pp % 0 ? pp * 2 * Math.PI : (pp + 1) * 2 * Math.PI;
        putVal(this.odrive + e.axis + ".encoder.config.calib_scan_distance", scan_distance);
        putVal(
          this.odrive + e.axis + ".encoder.config.cpr",
          this.wizardConfig[e.axis].encoder.config.cpr
        );

        await wait(250);

        this.calibrating = true;
        let result = await encoderCalibration(this.odrive, e.axis);
        this.calibrating = false;
        putVal(this.odrive + e.axis + ".encoder.config.cpr", oldCPR);
        fetchParam(this.odrive + e.axis + '.encoder.is_ready');
        if (result == 0){
          // no encoder error, we're good
          this.choiceMade = true;
          this.currentStep.choiceMade = true;
          this.calStatus = true;
        }
        else {
          clearErrors(this.odrive, e.axis);
          this.calStatus = false;
        }
      }
    },
    async choiceHandler(e) {
      // apply static configStub
      this.updateConfig(this.wizardConfig, e.configStub);

      // run hooks
      for (const fn of e.hooks) {
        this.wizardConfig = fn(this.wizardConfig);
      }

      // ugly, but a special case.
      // for motors, wait for calibration to finish before giving the green light unless motor.is_calibrated == true
      // for encoders, wait for calibration to finish unless encoder.is_ready == true
      if (
        this.currentStep == pages.Motor_0 ||
        this.currentStep == pages.Motor_1
      ) {
        let axis;
        if (this.currentStep == pages.Motor_0) axis = "axis0";
        if (this.currentStep == pages.Motor_1) axis = "axis1";
        fetchParam(this.odrive + axis + ".motor.is_calibrated");
        await wait(100);
        if (getVal(this.odrive + axis + ".motor.is_calibrated") == false) {
          this.choiceMade = false;
        }
        else {
          this.choiceMade = true;
        }
      }
      else if (
        this.currentStep == pages.Encoder_0 ||
        this.currentStep == pages.Encoder_1
      ) {
        let axis;
        if (this.currentStep == pages.Encoder_0) axis = "axis0";
        if (this.currentStep == pages.Encoder_1) axis = "axis1";
        fetchParam(this.odrive + axis + ".encoder.is_ready");
        await wait(100);
        if (getVal(this.odrive + axis + ".encoder.is_ready") == false) {
          this.choiceMade = false;
        }
        else {
          this.choiceMade = true;
        }
      }
      else {
        this.choiceMade = true;
      }
      this.currentStep.choiceMade = this.choiceMade;
      console.log(JSON.parse(JSON.stringify(this.wizardConfig)));
    },
    undoChoice(e) {
      this.choiceMade = false;
      this.currentStep.choiceMade = this.choiceMade;
      this.nullConfig(this.wizardConfig, e.configStub);
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
    nullConfig(config, configStub) {
      // iterate over keys in configStub
      if (configStub != null) {
        console.log("NullConfig");
        Object.keys(configStub).forEach((key) => {
          console.log(key);
          if (typeof configStub[key] == "object") {
            this.nullConfig(config[key], configStub[key]);
          } else {
            config[key] = null;
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
      this.calStatus = undefined;
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
  beforeDestroy() {
    for (const page of Object.keys(pages)) {
      pages[page].choiceMade = false;
    }
    let erase = (obj) => {
      Object.keys(obj).forEach((key) => {
        if (typeof obj[key] == "object" && obj[key] != null) {
          erase(obj[key]);
        } else {
          obj[key] = null;
        }
      });
    };
    erase(this.wizardConfig);
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

.vue-tooltip.tooltip-custom {
  background-color: lightyellow; /* var(--fg-color); */
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
  from {
    opacity: 0;
  }
  to {
    opacity: 1;
  }
}

.finished-page {
  color: green;
}
</style>