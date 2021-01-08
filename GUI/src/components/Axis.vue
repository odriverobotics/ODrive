<template>
  <div
    class="axis"
    @click.self="showError = !showError"
    :class="{ inactive: !connected, noError: !error, error: error }"
  >
    {{ axis }}
    <div
      v-show="showError"
      class="error-popup card"
      @click.self="showError = !showError"
    >
      <clear-errors :data="{ axis: axis }" />
      axis:
      <span :class="{ noError: !axisError, error: axisError }">{{
        axisErrorMsg
      }}</span>
      <br />motor:
      <span :class="{ noError: !motorError, error: motorError }">{{
        motorErrorMsg
      }}</span>
      <br />encoder:
      <span :class="{ noError: !encoderError, error: encoderError }">{{
        encoderErrorMsg
      }}</span>
      <br />controller:
      <span :class="{ noError: !controllerError, error: controllerError }">{{
        controllerErrorMsg
      }}</span>
    </div>
  </div>
</template>

<script>
import odriveEnums from "../assets/odriveEnums.json";
import clearErrors from "./clearErrors.vue";
import { getVal, fetchParam } from "../lib/odrive_utils";

export default {
  name: "Axis",
  components: {
    clearErrors,
  },
  props: ["axis", "odrives"],
  data() {
    return {
      showError: false,
      axisErr: undefined,
      motorErr: undefined,
      controllerErr: undefined,
      encoderErr: undefined,
    };
  },
  methods: {
    getErrorString(errCode, errType) {
      let retMsg = "none";

      if (errCode != 0) {
        // we got an error!
        // mask errors depending on which page is active
        let errs = [];
        for (const [name, value] of Object.entries(odriveEnums)) {
          if (name.includes(errType) && errCode & value) {
            errs.push(name);
          }
        }
        retMsg = errs.join(", ");
      }

      return retMsg;
    },
  },
  computed: {
    connected() {
      return this.$store.state.ODrivesConnected[this.axis.split(".")[0]];
    },
    axisErrorMsg() {
      let errCode = this.axisErr;

      if (this.$store.state.currentDash == "Wizard") {
        errCode &= ~odriveEnums.AXIS_ERROR_MOTOR_FAILED;
        errCode &= ~odriveEnums.AXIS_ERROR_ENCODER_FAILED;
      }

      return this.getErrorString(errCode, 'AXIS_ERROR');
    },
    motorErrorMsg() {
      let errCode = this.motorErr;

      if (this.$store.state.currentDash == "Wizard") {
        errCode &= ~odriveEnums.MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        errCode &= ~odriveEnums.MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE;
      }

      return this.getErrorString(errCode, 'MOTOR_ERROR');
    },
    encoderErrorMsg() {
      let errCode = this.encoderErr;

      if (this.$store.state.currentDash == "Wizard") {
        errCode &= ~odriveEnums.ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH;
        errCode &= ~odriveEnums.ENCODER_ERROR_NO_RESPONSE;
      }

      return this.getErrorString(errCode, 'ENCODER_ERROR');
    },
    controllerErrorMsg() {
      let errCode = this.controllerErr;

      return this.getErrorString(errCode, 'CONTROLLER_ERROR');
    },
    axisError() {
      let errCode = this.axisErr;
      if (this.$store.state.currentDash == "Wizard") {
        errCode &= ~odriveEnums.AXIS_ERROR_MOTOR_FAILED;
        errCode &= ~odriveEnums.AXIS_ERROR_ENCODER_FAILED;
      }
      return errCode != 0;
    },
    motorError() {
      let errCode = this.motorErr;
      if (this.$store.state.currentDash == "Wizard") {
        errCode &= ~odriveEnums.MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        errCode &= ~odriveEnums.MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE;
      }
      return errCode != 0;
    },
    encoderError() {
      let errCode = this.encoderErr;
      if (this.$store.state.currentDash == "Wizard") {
        errCode &= ~odriveEnums.ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH;
        errCode &= ~odriveEnums.ENCODER_ERROR_NO_RESPONSE;
      }
      return errCode != 0;
    },
    controllerError() {
      let errCode = this.controllerErr;
      return errCode != 0;
    },
    error() {
      return (
        this.axisError ||
        this.motorError ||
        this.encoderError ||
        this.controllerError
      );
    },
  },
  created() {
    // set up timeout loop for grabbing axis error values
    let update = () => {
      // Do we have an active connection to the ODrive that contains this axis?
      if (this.$store.state.ODrivesConnected[this.axis.split(".")[0]]) {
        fetchParam(this.axis + ".error");
        fetchParam(this.axis + ".motor.error");
        fetchParam(this.axis + ".controller.error");
        fetchParam(this.axis + ".encoder.error");
        this.axisErr = getVal(this.axis + ".error");
        this.motorErr = getVal(this.axis + ".motor.error");
        this.controllerErr = getVal(this.axis + ".controller.error");
        this.encoderErr = getVal(this.axis + ".encoder.error");
      }
    };
    this.cyclicUpdate = setInterval(update, 1000);
  },
  beforeDestroy() {
    clearInterval(this.cyclicUpdate);
  }
};
</script>

<style scoped>
.axis {
  padding: 5px 10px;
  border: 2px black;
  cursor: pointer;
}

.axis:active {
  background-color: var(--bg-color);
}

.noError {
  color: green;
}

.error {
  color: red;
  font-weight: bold;
}

.error-popup {
  position: absolute;
  bottom: 2rem;
  color: black;
  margin-left: 0px;
}

.inactive {
  color: grey;
}
</style>