<template>
  <div class="wizard">
    <div class="wizard-container">
      <div class="wizard-page">
        <wizardPage
          :choices="currentStep.options.choices"
          :customComponents="currentStep.options.customComponents"
          :title="currentStep.title"
          :axis="currentAxis"
          v-on:choice="choiceHandler"
        />
        <div class="wizard-controls">
          <!-- show breadcrumbs, back, apply, next buttons -->
          <button
            class="wizard-button card"
            v-bind:class="{active: backAllowed, inactive: !backAllowed}"
            @click="back"
          >BACK</button>
          <button
            class="wizard-button card"
            v-bind:class="{active: applyAllowed, inactive: !applyAllowed}"
            @click="apply"
          >APPLY</button>
          <button
            class="wizard-button card"
            v-bind:class="{active: nextAllowed, inactive: !nextAllowed}"
            @click="next"
          >NEXT</button>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import configTemplate from "../assets/wizard/configTemplate.json";
import wizardPage from "../components/wizard/wizardPage.vue";
import odriveEnums from "../assets/odriveEnums.json";

let motorChoices = {
  choices: [
    {
      imageURL: require("../assets/images/D5065_300x300.png"),
      text: "ODrive D5065",
      config: {
        pole_pairs: 7,
        phase_resistance: 0.039, // from test rig
        phase_inductance: 1.57e-5,
        current_lim: 70,
        type: odriveEnums.MOTOR_TYPE_HIGH_CURRENT, // MOTOR_TYPE_HIGH_CURRENT
      },
    },
    {
      imageURL: require("../assets/images/D6374_300x300.png"),
      text: "ODrive D6374",
      config: {
        pole_pairs: 7,
        phase_resistance: 0.041, // measurement from PJ
        phase_inductance: 2.23e-5,
        current_lim: 71,
        type: odriveEnums.MOTOR_TYPE_HIGH_CURRENT, // MOTOR_TYPE_HIGH_CURRENT,
      },
    },
  ],
  customComponents: [
    {
      compName: "wizardMotorCustom",
      id: 0,
    },
  ],
};

let encoderChoices = {
  choices: [
    {
      imageURL: require("../assets/images/amt102-v_300x300.png"),
      text: "CUI AMT102V",
      config: {
        cpr: 8192,
        use_index: true,
        mode: odriveEnums.ENCODER_MODE_INCREMENTAL, // ENCODER_MODE_INCREMENTAL
      },
    },
  ],
  customComponents: [
    /*{
      compName: "wizardEncoderIncremental",
      id: 0,
      config: {
        cpr: null,
        use_index: false,
        mode: odriveEnums.ENCODER_MODE_INCREMENTAL
      }
    },
    {
      compName: "wizardEncoderIncrementalIndex",
      id: 1,
      config: {
        cpr: null,
        use_index: true,
        mode: odriveEnums.ENCODER_MODE_INCREMENTAL
      }
    },
    {
      compName: "wizardEncoderHallEffect",
      id: 2,
      config: {
        cpr: null,
        use_index: null,
        mode: odriveEnums.ENCODER_MODE_HALL
      }
    },*/
  ],
};

let limitChoices = {
  choices: [
    {
      imageURL: "",
      text: "limits stand-in",
      config: {},
    },
  ],
  customComponents: [
    {
      compName: "wizardLimits",
      id: 0,
    },
  ],
};

let odriveChoices = {
  choices: [
    {
      imageURL: require("../assets/images/24v_300x300.png"),
      text: "ODrive v3.6 24V",
    },
    {
      imageURL: require("../assets/images/56v_300x300.png"),
      text: "ODrive v3.6 56V",
    },
  ],
  customComponents: [],
};

let axisChoices = {
  choices: [
    {
      imageURL: require("../assets/images/M0_300x300.png"),
      text: "M0",
    },
    {
      imageURL: require("../assets/images/M1_300x300.png"),
      text: "M1",
    },
    {
      imageURL: require("../assets/images/M0M1_300x300.png"),
      text: "M0 and M1",
    },
  ],
  customComponents: [],
};

let states = {
  PICK_ODRIVE: {
    val: 0,
    page: "wizardODrive",
    title: "Which ODrive do you have?",
    options: odriveChoices,
  },
  PICK_AXIS: {
    val: 1,
    page: "wizardAxis",
    title: "Which axis are you setting up?",
    options: axisChoices,
  },
  MOTOR_0: {
    val: 2,
    page: "wizardMotor",
    title: "What motor are you using for M0?",
    options: motorChoices,
  },
  ENCODER_0: {
    val: 3,
    page: "wizardEncoder",
    title: "What encoder are you using for M0?",
    options: encoderChoices,
  },
  MOTOR_1: {
    val: 4,
    page: "wizardMotor",
    title: "What motor are you using for M1?",
    options: motorChoices,
  },
  ENCODER_1: {
    val: 5,
    page: "wizardEncoder",
    title: "What encoder are you using for M1?",
    options: encoderChoices,
  },
  LIMITS_0: {
    val: 6,
    page: "wizardLimits",
    title: "Set your limits for M0",
    // this page should really be bespoke...
    options: limitChoices,
  },
  LIMITS_1: {
    val: 7,
    title: "Set your limits for M1",
    page: "wizardLimits",
    options: limitChoices,
  },
};

export default {
  name: "Wizard",
  components: {
    wizardPage,
  },
  props: [],
  data: function () {
    return {
      currentStep: states.PICK_ODRIVE,
      currentChoice: undefined,
      nextRequest: false,
      applyRequest: false,
      backRequest: false,
      backAllowed: false,
      applyAllowed: false,
      nextAllowed: false,
      statePath: undefined,
      wizardConfig: configTemplate,
      axis: undefined,
      currentAxis: undefined,
    };
  },
  computed: {
    currentPage() {
      return this.currentStep.page;
    },
    states() {
      return states;
    },
  },
  methods: {
    next() {
      console.log("next page");
      this.nextRequest = true;
      this.nextState();
      this.nextRequest = false;
    },
    apply() {
      console.log("attempting to apply config...");
      console.log(JSON.parse(JSON.stringify(this.wizardConfig)));
      this.applyRequest = true;
      this.nextState();
      this.applyRequest = false;
    },
    back() {
      console.log("previous page");
      this.backRequest = true;
      this.nextState();
      this.backRequest = false;
    },
    choiceHandler(e) {
      console.log(e.choice);
      this.currentChoice = e;
      this.nextState();
    },
    nextState() {
      // given current choice and page, control which actions are allowed and
      // change pages as necessary
      switch (this.currentStep) {
        case states.PICK_ODRIVE:
          if (this.currentChoice == undefined) {
            this.currentChoice = undefined; //do nothing
          }
          else if (
            this.currentChoice.choice ==
            states.PICK_ODRIVE.options.choices[0].text
          ) {
            this.wizardConfig.brake_resistance = 0.5;
            this.nextAllowed = true;
          }
          else if (
            this.currentChoice.choice ==
            states.PICK_ODRIVE.options.choices[1].text
          ) {
            this.wizardConfig.brake_resistance = 2.0;
            this.nextAllowed = true;
          }
          if (this.nextAllowed == true && this.nextRequest == true) {
            this.currentStep = states.PICK_AXIS;
            this.nextAllowed = false;
            this.currentChoice = undefined;
            this.backAllowed = true;
          }
          break;
        case states.PICK_AXIS:
          if (this.currentChoice != undefined) {
            this.axis = this.currentChoice.choice;
            this.nextAllowed = true;
          }
          if (this.nextAllowed == true && this.nextRequest == true) {
            if (this.axis == "M0" || this.axis == "M0 and M1") {
              this.currentStep = states.MOTOR_0;
              this.currentAxis = "axis0";
            } else if (this.axis == "M1") {
              this.currentStep = states.MOTOR_1;
              this.currentAxis = "axis1";
            }
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.backAllowed = true;
            this.applyAllowed = false;
          }
          if (this.backRequest == true) {
            this.currentStep = states.PICK_ODRIVE;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.backAllowed = false;
          }
          break;
        case states.MOTOR_0:
          if (this.currentChoice != undefined) {
            this.wizardConfig.axis0.motor.config = this.currentChoice.config;
            this.nextAllowed = true;
          }
          if (this.nextAllowed == true && this.nextRequest == true) {
            this.currentStep = states.ENCODER_0;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.applyAllowed = false;
            this.backAllowed = true;
          }
          if (this.backRequest == true) {
            this.currentStep = states.PICK_AXIS;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.backAllowed = true;
          }
          break;
        case states.ENCODER_0:
          if (this.currentChoice != undefined) {
            this.wizardConfig.axis0.encoder.config = this.currentChoice.config;
            this.nextAllowed = true;
          }
          if (this.nextAllowed == true && this.nextRequest == true) {
            this.currentStep = states.LIMITS_0;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.applyAllowed = false;
            this.backAllowed = true;
          }
          if (this.backRequest == true) {
            this.currentStep = states.MOTOR_0;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.backAllowed = true;
          }
          break;
        case states.LIMITS_0:
          if (this.axis == "M0") {
            // this is the last page
            this.nextAllowed = false;
            this.applyAllowed = true;
          }
          if (this.axis == "M0 and M1") {
            // not the last page! go to MOTOR_1
            if (this.currentChoice != undefined) {
              this.nextAllowed = true;
            }
            this.applyAllowed = false;
            if (this.nextAllowed == true && this.nextRequest == true) {
              this.currentStep = states.MOTOR_1;
              this.currentAxis = "axis1"
              this.currentChoice = undefined;
              this.nextAllowed = false;
              this.applyAllowed = false;
              this.backAllowed = true;
            }
          }
          if (this.backRequest == true) {
            this.currentStep = states.ENCODER_0;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.applyAllowed = false;
            this.backAllowed = true;
          }
          break;
        case states.MOTOR_1:
          if (this.currentChoice != undefined) {
            this.wizardConfig.axis1.motor.config = this.currentChoice.config;
            this.nextAllowed = true;
          }
          if (this.nextAllowed == true && this.nextRequest == true) {
            this.currentStep = states.ENCODER_1;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.applyAllowed = false;
            this.backAllowed = true;
          }
          if (this.backRequest == true) {
            if (this.axis == "M1") {
              this.currentStep = states.PICK_AXIS;
            } else if (this.axis == "M0 and M1") {
              this.currentStep = states.LIMITS_0;
              this.currentAxis = "axis0";
            }
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.currentChoice = undefined;
            this.backAllowed = true;
          }
          break;
        case states.ENCODER_1:
          if (this.currentChoice != undefined) {
            this.wizardConfig.axis1.encoder.config = this.currentChoice.config;
            this.nextAllowed = true;
          }
          if (this.nextAllowed == true && this.nextRequest == true) {
            this.currentStep = states.LIMITS_1;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.applyAllowed = false;
            this.backAllowed = true;
          }
          if (this.backRequest == true) {
            this.currentStep = states.MOTOR_1;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.backAllowed = true;
          }
          break;
        case states.LIMITS_1:
          if (this.currentChoice != undefined) {
            // this page is always an endpoint
            // apply limits for axis 1 config
            this.applyAllowed = true;
          }
          if (this.backRequest == true) {
            this.currentStep = states.ENCODER_1;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.applyAllowed = false;
            this.backAllowed = true;
          }
          break;
      }
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
  margin-left: 0;
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