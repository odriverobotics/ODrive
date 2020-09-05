<template>
  <div class="wizard">
    <div class="wizard-container">
      <div class="wizard-page">
        <wizardPage
          :choices="currentStep.options.choices"
          :customComponents="currentStep.options.customComponents"
          :title="currentStep.title"
          :axis="currentAxis"
          :config="wizardConfig"
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

// the choices arrays contain all relevant info for a choice
// for example - a picture of a D5065 motor and it's corresponding
// config parameters.
// customComponents is an array that points to a Vue component to
// use instead of a predefined wizardChoice component.
// these components take user input. When all necessary user input
// has been received, they emit a 'choice' event with the corresponding
// config object.

let motorChoices = {
  choices: [
    {
      imageURL: require("../assets/images/D5065_300x300.png"),
      text: "ODrive D5065",
      config: {
        pole_pairs: 7,
        torque_constant: 8.27 / 270,
        phase_resistance: 0.039, // from test rig
        phase_inductance: 1.57e-5,
        type: odriveEnums.MOTOR_TYPE_HIGH_CURRENT, // MOTOR_TYPE_HIGH_CURRENT
      },
    },
    {
      imageURL: require("../assets/images/D6374_300x300.png"),
      text: "ODrive D6374",
      config: {
        pole_pairs: 7,
        torque_constant: 8.27 / 150,
        phase_resistance: 0.041, // measurement from PJ
        phase_inductance: 2.23e-5,
        type: odriveEnums.MOTOR_TYPE_HIGH_CURRENT, // MOTOR_TYPE_HIGH_CURRENT,
      },
    },
  ],
  customComponents: [
    {
      compName: "wizardMotor",
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
    {
      imageURL: "",
      text: "Hall Effect",
      config: {
        cpr: 0,
        use_index: false,
        mode: odriveEnums.ENCODER_MODE_HALL,
      },
    },
  ],
  customComponents: [
    {
      compName: "wizardEncoderIncremental",
      id: 0,
    },
    {
      compName: "wizardEncoderIncrementalIndex",
      id: 1,
    },
  ],
};

let limitChoices = {
  choices: [],
  customComponents: [
    {
      compName: "wizardMisc",
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

let endChoices = {
  choices: [],
  customComponents: [
    {
      compName: "wizardEnd",
      id: 0,
    }
  ]
}

// These are the states for the config wizard state machine

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
  MISC_0: {
    val: 6,
    page: "wizardMisc",
    title: "Finishing touches for M0",
    options: limitChoices,
  },
  MISC_1: {
    val: 7,
    title: "Finishing touches for M1",
    page: "wizardMisc",
    options: limitChoices,
  },
  END: {
    val: 8,
    title: "You're Done!",
    page: "wizardEnd",
    options : endChoices
  }
};

export default {
  name: "Wizard",
  components: {
    wizardPage,
  },
  props: [],
  data: function () {
    return {
      currentStep: states.PICK_ODRIVE, // which state are we in?
      currentChoice: undefined, // choice object from wizardChoice or custom component
      nextRequest: false, // next, apply, and back buttons are handled by the
      // state machine. They are either allowed or disallowed
      applyRequest: false,
      backRequest: false,
      backAllowed: false,
      applyAllowed: false,
      nextAllowed: false,
      statePath: undefined,
      wizardConfig: configTemplate,
      axis: undefined, // state transitions depend on whether we are handling
      // axis0, axis1, or both axes
      currentAxis: undefined, // which axis are we currently configuring?
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
    applyConfig(){
      // send config parameters that aren't null to the connected odrive!
      console.log("attempting to apply config...");
      console.log(JSON.parse(JSON.stringify(this.wizardConfig)));
    },
    nextState() {
      // given current choice and page, control which actions are allowed and
      // change pages as necessary
      switch (this.currentStep) {
        case states.PICK_ODRIVE:
          if (this.currentChoice == undefined) {
            this.currentChoice = undefined; //do nothing
          } else if (
            this.currentChoice.choice ==
            states.PICK_ODRIVE.options.choices[0].text
          ) {
            this.wizardConfig.brake_resistance = 0.5;
            this.nextAllowed = true;
          } else if (
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
            this.wizardConfig.axis0.motor.config = {
              pole_pairs: null,
              torque_constant: null,
              phase_resistance: null,
              phase_inductance: null,
            }
            this.currentStep = states.PICK_AXIS;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.backAllowed = true;
          }
          break;
        case states.ENCODER_0:
          if (this.currentChoice != undefined) {
            if (this.currentChoice.choice == "Hall Effect"){
              this.wizardConfig.axis0.encoder.config = {
                cpr: 6 * this.wizardConfig.axis0.motor.config.pole_pairs,
                use_index: false,
                mode: odriveEnums.ENCODER_MODE_HALL,
              }
            }
            else {
              this.wizardConfig.axis0.encoder.config = this.currentChoice.config;
            }
            this.nextAllowed = true;
          }
          if (this.nextAllowed == true && this.nextRequest == true) {
            this.currentStep = states.MISC_0;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.applyAllowed = false;
            this.backAllowed = true;
          }
          if (this.backRequest == true) {
            this.wizardConfig.axis0.encoder.config = {
              cpr: null,
              use_index: null,
              mode: null,
            }
            this.currentStep = states.MOTOR_0;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.backAllowed = true;
          }
          break;
        case states.MISC_0:
          if (this.axis == "M0") {
            if (this.currentChoice != undefined) {
              this.wizardConfig.axis0.controller.config.vel_limit = this.currentChoice.config.vel_limit;
              this.wizardConfig.axis0.motor.config.current_lim = this.currentChoice.config.current_lim;
              this.nextAllowed = true;
            }
            if (this.nextAllowed == true && this.nextRequest == true){
              this.currentStep = states.END;
              this.applyAllowed = true;
              this.backAllowed = true;
              this.nextAllowed = false;
            }
          }
          if (this.axis == "M0 and M1") {
            // not the last page! go to MOTOR_1
            if (this.currentChoice != undefined) {
              this.nextAllowed = true;
              this.wizardConfig.axis0.controller.config.vel_limit = this.currentChoice.config.vel_limit;
              this.wizardConfig.axis0.motor.config.current_lim = this.currentChoice.config.current_lim;
            }
            this.applyAllowed = false;
            if (this.nextAllowed == true && this.nextRequest == true) {
              this.currentStep = states.MOTOR_1;
              this.currentAxis = "axis1";
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
              this.currentStep = states.MISC_0;
              this.currentAxis = "axis0";
            }
            this.wizardConfig.axis1.motor.config = {
              pole_pairs: null,
              torque_constant: null,
              phase_resistance: null,
              phase_inductance: null,
            }
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.currentChoice = undefined;
            this.backAllowed = true;
          }
          break;
        case states.ENCODER_1:
          if (this.currentChoice != undefined) {
            if (this.currentChoice.choice == "Hall Effect"){
              this.wizardConfig.axis1.encoder.config = {
                cpr: 6 * this.wizardConfig.axis1.motor.config.pole_pairs,
                use_index: false,
                mode: odriveEnums.ENCODER_MODE_HALL,
              }
            }
            else {
              this.wizardConfig.axis1.encoder.config = this.currentChoice.config;
            }
            this.nextAllowed = true;
          }
          if (this.nextAllowed == true && this.nextRequest == true) {
            this.currentStep = states.MISC_1;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.applyAllowed = false;
            this.backAllowed = true;
          }
          if (this.backRequest == true) {
            this.wizardConfig.axis1.encoder.config = {
              cpr: null,
              use_index: null,
              mode: null,
            }
            this.currentStep = states.MOTOR_1;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.backAllowed = true;
          }
          break;
        case states.MISC_1:
          if (this.currentChoice != undefined) {
            // this page is always an endpoint
            // apply limits for axis 1 config
            this.nextAllowed = true;
            this.wizardConfig.axis1.controller.config.vel_limit = this.currentChoice.config.vel_limit;
            this.wizardConfig.axis1.motor.config.current_lim = this.currentChoice.config.current_lim;
          }
          if (this.nextRequest == true && this.nextAllowed == true) {
            this.currentStep = states.END;
            this.applyAllowed = true;
            this.backAllowed = true;
            this.nextAllowed = false;
          }
          if (this.backRequest == true) {
            this.currentStep = states.ENCODER_1;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.applyAllowed = false;
            this.backAllowed = true;
          }
          break;
        case states.END:
          if (this.applyRequest == true && this.applyAllowed == true) {
            this.applyConfig();
          }

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