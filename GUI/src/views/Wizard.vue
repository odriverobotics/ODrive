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
import { getVal, putVal, fetchParam } from "../lib/odrive_utils.js";

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
    pageEventHandler(e) {
      this.choiceMade = false;
      let clear = () => {
        let paths = [
          "odrive0." + e.axis + ".error",
          "odrive0." + e.axis + ".motor.error",
          "odrive0." + e.axis + ".encoder.error",
          "odrive0." + e.axis + ".controller.error",
        ];
        for (const path of paths) {
          putVal(path, 0);
        }
        console.log("clearing error for " + e.axis);
      };
      if (e.data == "motor calibration") {
        // set a timeout to grab axis resistance and inductance values
        fetchParam("odrive0." + e.axis + ".error");
        let apply = () => {
          if (getVal("odrive0." + e.axis + ".error") == 0) {
            let configStub = undefined;
            let inductance = getVal(
              "odrive0." + e.axis + ".motor.config.phase_inductance"
            );
            let resistance = getVal(
              "odrive0." + e.axis + ".motor.config.phase_resistance"
            );
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
            this.choiceMade = true;
          }
        };
        let updateCalInfo = () => {
          // parse error code, update calibration information
          let motorError = getVal("odrive0." + e.axis + ".motor.error");
          if (
            motorError ==
              odriveEnums.MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE ||
            motorError == odriveEnums.MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE
          ) {
            //we got the expected calibration error
            console.log("Calibration failed.");
            this.calStatus = false;
          }
        };
        this.wait = function () {
          fetchParam("odrive0." + e.axis + ".current_state");
          fetchParam("odrive0." + e.axis + ".motor.config.phase_resistance");
          fetchParam("odrive0." + e.axis + ".motor.config.phase_inductance");
          fetchParam("odrive0." + e.axis + ".motor.is_calibrated");
          console.log();
          if (
            getVal("odrive0." + e.axis + ".current_state") ==
            odriveEnums.AXIS_STATE_MOTOR_CALIBRATION
          ) {
            // still calibrating
            console.log("waiting for motor cal to finish");
            setTimeout(() => this.wait(), 100);
            this.calibrating = true;
          } else if (getVal("odrive0." + e.axis + ".error") != 0) {
            console.log("motor cal error");
            this.calibrating = false;
            this.choiceMade = false;
            // clear errors, display info.
            updateCalInfo();
            clear();
          } else if (getVal("odrive0." + e.axis + ".motor.is_calibrated") == true){
            // calibration is over
            apply();
            this.calibrating = false;
            this.calStatus = true;
          }
          else {
            setTimeout(() => this.wait(), 100);
            console.log("waiting for motor cal to finish");
          }
        };
        fetchParam("odrive0." + e.axis + ".current_state");
        fetchParam("odrive0." + e.axis + ".motor.config.phase_resistance");
        fetchParam("odrive0." + e.axis + ".motor.config.phase_inductance");
        fetchParam("odrive0." + e.axis + ".motor.is_calibrated");
        // wait for at least a second for comms to update state of ODrive
        setTimeout(() => this.wait(), 1000);
      } else if (e.data == "encoder calibration") {
        // get old CPR
        // apply CPR from this.wizardConfig
        // start calibration
        // wait for cal to finish
        // set odrive cpr back to oldVal
        let oldCPR = getVal("odrive0." + e.axis + ".encoder.config.cpr");
        console.log("oldCPR = " + oldCPR);
        console.log("axis = " + e.axis);
        putVal(
          "odrive0." + e.axis + ".encoder.config.cpr",
          this.wizardConfig[e.axis].encoder.config.cpr
        );
        putVal(
          "odrive0." + e.axis + ".requested_state",
          odriveEnums.AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        );

        this.wait = function () {
          fetchParam("odrive0." + e.axis + ".current_state");
          fetchParam("odrive0." + e.axis + ".encoder.error");
          fetchParam("odrive0." + e.axis + ".encoder.is_ready");
          if (
            getVal("odrive0." + e.axis + ".current_state") ==
            odriveEnums.AXIS_STATE_ENCODER_OFFSET_CALIBRATION
          ) {
            console.log("waiting for encoder cal to finish...");
            setTimeout(() => this.wait(), 100);
            this.calibrating = true;
          } else if (getVal("odrive0." + e.axis + ".encoder.error") != 0) {
            putVal("odrive0." + e.axis + ".encoder.config.cpr", oldCPR);
            console.log("applying old CPR, error detected");
            clear();
            this.calibrating = false;
            this.calStatus = false;
          } else if (getVal("odrive0." + e.axis + ".encoder.is_ready") == true){
            putVal("odrive0." + e.axis + ".encoder.config.cpr", oldCPR);
            console.log("applying old CPR");
            this.choiceMade = true;
            this.currentStep.choiceMade = true;
            this.calibrating = false;
            this.calStatus = true;
          }
          else {
            console.log("waiting for encoder cal to finish...");
            setTimeout(() => this.wait(), 100);
          }
        };
        fetchParam("odrive0." + e.axis + ".current_state");
        fetchParam("odrive0." + e.axis + ".encoder.error");
        fetchParam("odrive0." + e.axis + ".encoder.is_ready");
        setTimeout(() => this.wait(), 1000);
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
      // for motors, wait for calibration to finish before giving the green light unless motor.is_calibrated == true
      // for encoders, wait for calibration to finish unless encoder.is_ready == true
      this.choiceMade = true;
      if (
        this.currentStep == pages.Motor_0 || this.currentStep == pages.Motor_1
      ) {
        let axis;
        if (this.currentStep == pages.Motor_0) axis = "axis0";
        if (this.currentStep == pages.Motor_1) axis = "axis1";
        if (getVal("odrive0." + axis + ".motor.is_calibrated") == false) {
          this.choiceMade = false;
        }
      }
      if (
        this.currentStep == pages.Encoder_0 || this.currentStep == pages.Encoder_1
      ) {
        let axis;
        if (this.currentStep == pages.Encoder_0) axis = "axis0";
        if (this.currentStep == pages.Encoder_1) axis = "axis1";
        if (getVal("odrive0." + axis + ".encoder.is_ready") == false) {
          this.choiceMade = false;
        }
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
  created() {
    // when wizard is active, we want to poll for certain values
    let update = () => {
      fetchParam("odrive0.axis0.motor.is_calibrated");
      fetchParam("odrive0.axis1.motor.is_calibrated");
      fetchParam("odrive0.axis0.encoder.is_ready");
      fetchParam("odrive0.axis1.encoder.is_ready");
      setTimeout(() => update(), 1000);
    };
    update();
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