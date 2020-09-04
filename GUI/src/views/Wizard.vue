<template>
  <div class="wizard">
    <div class="wizard-container">
      <div class="wizard-page">
        <wizardPage
          :choices="currentStep.choices"
          :customComponents="currentStep.customComponents"
          :title="currentStep.title"
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
//import wizardODrive from "../components/wizard/wizardODrive.vue";
//import wizardAxis from "../components/wizard/wizardAxis.vue";
//import wizardMotor from "../components/wizard/wizardMotor.vue";
//import wizardEncoder from "../components/wizard/wizardEncoder.vue";
//import wizardLimits from "../components/wizard/wizardLimits.vue";
import wizardPage from "../components/wizard/wizardPage.vue";

let states = {
  PICK_ODRIVE: {
    val: 0,
    page: "wizardODrive",
    title: "Which ODrive do you have?",
    choices: [
      {
        name: "ODrive v3.6 24V",
        imageURL: require("../assets/images/24v_300x300.png"),
        text: "ODrive v3.6 24V",
      },
      {
        name: "ODrive v3.6 56V",
        imageURL: require("../assets/images/56v_300x300.png"),
        text: "ODrive v3.6 56V",
      },
    ],
  },
  PICK_AXIS: {
    val: 1,
    page: "wizardAxis",
    title: "Which axis are you setting up?",
    choices: [
      {
        name: "M0",
        imageURL: require("../assets/images/M0_300x300.png"),
        text: "M0",
      },
      {
        name: "M1",
        imageURL: require("../assets/images/M1_300x300.png"),
        text: "M1",
      },
      {
        name: "M0 and M1",
        imageURL: require("../assets/images/M0M1_300x300.png"),
        text: "M0 and M1",
      },
    ],
  },
  MOTOR_0: {
    val: 2,
    page: "wizardMotor",
    title: "What motor are you using for M0?",
    choices: [
      {
        name: "ODrive D5065",
        imageURL: require("../assets/images/D5065_300x300.png"),
        text: "ODrive D5065",
      },
      {
        name: "ODrive D6374",
        imageURL: require("../assets/images/D6374_300x300.png"),
        text: "ODrive D6374",
      },
    ],
    customComponents: [
      {
        compName: "wizardMotorCustom",
        id: 0,
      }
    ]
  },
  ENCODER_0: {
    val: 3,
    page: "wizardEncoder",
    title: "What encoder are you using for M0?",
    choices: [
      {
        name: "CUI AMT102V",
        imageURL: require("../assets/images/amt102-v_300x300.png"),
        text: "CUI AMT102V",
      },
      {
        name: "Incremental",
        imageURL: "",
        text: "Incremental",
      },
      {
        name: "Incremental with Index",
        imageURL: "",
        text: "Incremental with Index"
      },
      {
        name: "Hall Effect",
        imageURL: "",
        text: "Hall Effect",
      },
      {
        name: "SPI AMS AS504x",
        imageURL: "",
        text: "SPI AMS AS504x",
      },
      {
        name: "SPI CUI AMT23xx",
        imageURL: "",
        text: "SPI CUI AMT23xx",
      }
    ],
  },
  MOTOR_1: {
    val: 4,
    page: "wizardMotor",
    title: "What motor are you using for M1?",
    choices: [
      {
        name: "ODrive D5065",
        imageURL: require("../assets/images/D5065_300x300.png"),
        text: "ODrive D5065",
      },
      {
        name: "ODrive D6374",
        imageURL: require("../assets/images/D6374_300x300.png"),
        text: "ODrive D6374",
      },
    ],
  },
  ENCODER_1: {
    val: 5,
    page: "wizardEncoder",
    title: "What encoder are you using for M1?",
    choices: [
      {
        name: "CUI AMT102V",
        imageURL: require("../assets/images/amt102-v_300x300.png"),
        text: "CUI AMT102V",
      },
      {
        name: "Incremental",
        imageURL: "",
        text: "Incremental",
      },
      {
        name: "Incremental with Index",
        imageURL: "",
        text: "Incremental with Index"
      },
      {
        name: "Hall Effect",
        imageURL: "",
        text: "Hall Effect",
      },
      {
        name: "SPI AMS AS504x",
        imageURL: "",
        text: "SPI AMS AS504x",
      },
      {
        name: "SPI CUI AMT23xx",
        imageURL: "",
        text: "SPI CUI AMT23xx",
      }
    ],
  },
  LIMITS_0: {
    val: 6,
    page: "wizardLimits",
    title: "Set your limits for M0",
    // this page should really be bespoke...
    choices: [],
    customComponents: [
      {
        compName: "wizardLimits",
        id: 0,
      }
    ]
  },
  LIMITS_1: {
    val: 7,
    title: "Set your limits for M1",
    page: "wizardLimits",
    choices: [],
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
      config: {
        voltage: undefined,
        brake: undefined,
        M0: {
          motor: undefined,
          limits: {
            vel_limit: undefined,
            current_lim: undefined,
          },
          encoder: {
            type: undefined,
            cpr: undefined,
          }
        },
        M1: {
          motor: undefined,
          limits: {
            vel_limit: undefined,
            current_lim: undefined,
          },
          encoder: {
            type: undefined,
            cpr: undefined,
          }
        },
        axes: undefined,
      },
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
      this.currentChoice = e.choice;
      this.nextState();
    },
    nextState() {
      // given current choice and page, control which actions are allowed and
      // change pages as necessary
      switch (this.currentStep) {
        case states.PICK_ODRIVE:
          if (this.currentChoice == states.PICK_ODRIVE.choices[0].name) {
            this.config.voltage = "24V";
            this.nextAllowed = true;
          }
          if (this.currentChoice == states.PICK_ODRIVE.choices[1].name) {
            this.config.voltage = "56V";
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
          if (this.currentChoice != undefined){
            this.config.axes = this.currentChoice;
            this.nextAllowed = true;
          }
          if (this.nextAllowed == true && this.nextRequest == true) {
            if (this.config.axes == "M0" || this.config.axes == "M0M1") {
              this.currentStep = states.MOTOR_0;
            } else if (this.config.axes == "M1") {
              this.currentStep = states.MOTOR_1;
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
            this.config.M0.motor = this.currentChoice;
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
            this.currentChoice = undefined;
            this.backAllowed = false;
          }
          break;
        case states.ENCODER_0:
          if (this.currentChoice != undefined) {
            this.config.M0.encoder = this.currentChoice;
            this.nextAllowed = true;
          }
          if (this.nextAllowed == true && this.nextRequest == true) {
            this.currentStep = states.LIMITS_0;
            this.currentChoice = undefined;
            this.nextAllowed = true;
            this.applyAllowed = false;
            this.backAllowed = true;
          }
          if (this.backRequest == true) {
            this.currentStep = states.MOTOR_0;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.currentChoice = undefined;
            this.backAllowed = false;
          }
          break;
        case states.LIMITS_0:
          if (this.backRequest == true) {
            this.currentStep = states.ENCODER_0;
            this.currentChoice = undefined;
            this.nextAllowed = false;
            this.currentChoice = undefined;
            this.backAllowed = false;
          }
          break;
        case states.MOTOR_1:
          break;
        case states.ENCODER_1:
          break;
        case states.LIMITS_1:
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
</style>