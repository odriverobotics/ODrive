<template>
  <div class="axis" @click="showError = !showError;" v-bind:class="{ noError: !axisError, error: axisError}">
      {{ axis.name }}
      <div v-show="showError" class="error-popup card">
          axis: <span v-bind:class="{ noError: !axisError, error: axisError}">{{axisErrorMsg}}</span> <br>
          motor: <span v-bind:class="{ noError: !motorError, error: motorError}">{{motorErrorMsg}}</span> <br>
          encoder: <span v-bind:class="{ noError: !encoderError, error: encoderError}">{{encoderErrorMsg}}</span> <br>
          controller: <span v-bind:class="{ noError: !controllerError, error: controllerError}">{{controllerErrorMsg}}</span>
      </div>
  </div>
</template>

<script>

const axisErrors = {
    0x00000000: "AXIS_ERROR_NONE",
    0x00000001: "AXIS_ERROR_INVALID_STATE",
    0x00000002: "AXIS_ERROR_DC_BUS_UNDER_VOLTAGE",
    0x00000004: "AXIS_ERROR_DC_BUS_OVER_VOLTAGE",
    0x00000008: "AXIS_ERROR_CURRENT_MEASUREMENT_TIMEOUT",
    0x00000010: "AXIS_ERROR_BRAKE_RESISTOR_DISARMED",
    0x00000020: "AXIS_ERROR_MOTOR_DISARMED",
    0x00000040: "AXIS_ERROR_MOTOR_FAILED",
    0x00000080: "AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED",
    0x00000100: "AXIS_ERROR_ENCODER_FAILED",
    0x00000200: "AXIS_ERROR_CONTROLLER_FAILED",
    0x00000400: "AXIS_ERROR_POS_CTRL_DURING_SENSORLESS",
    0x00000800: "AXIS_ERROR_WATCHDOG_TIMER_EXPIRED",
    0x00001000: "AXIS_ERROR_MIN_ENDSTOP_PRESSED",
    0x00002000: "AXIS_ERROR_MAX_ENDSTOP_PRESSED",
    0x00004000: "AXIS_ERROR_ESTOP_REQUESTED",
    0x00020000: "AXIS_ERROR_HOMING_WITHOUT_ENDSTOP",
    0x00040000: "AXIS_ERROR_OVER_TEMP",
};

const motorErrors = {
    0x00000000: "MOTOR_ERROR_NONE",
    0x00000001: "MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE",
    0x00000002: "MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE",
    0x00000004: "MOTOR_ERROR_ADC_FAILED",
    0x00000008: "MOTOR_ERROR_DRV_FAULT",
    0x00000010: "MOTOR_ERROR_CONTROL_DEADLINE_MISSED",
    0x00000020: "MOTOR_ERROR_NOT_IMPLEMENTED_MOTOR_TYPE",
    0x00000040: "MOTOR_ERROR_BRAKE_CURRENT_OUT_OF_RANGE",
    0x00000080: "MOTOR_ERROR_MODULATION_MAGNITUDE",
    0x00000100: "MOTOR_ERROR_BRAKE_DEADTIME_VIOLATION",
    0x00000200: "MOTOR_ERROR_UNEXPECTED_TIMER_CALLBACK",
    0x00000400: "MOTOR_ERROR_CURRENT_SENSE_SATURATION",
    0x00001000: "MOTOR_ERROR_CURRENT_LIMIT_VIOLATION",
    0x00002000: "MOTOR_ERROR_BRAKE_DUTY_CYCLE_NAN", 
    0x00004000: "MOTOR_ERROR_DC_BUS_OVER_REGEN_CURRENT",
    0x00008000: "MOTOR_ERROR_DC_BUS_OVER_CURRENT",
};

let encoderErrors = {
    0x00000000: "ENCODER_ERROR_NONE",
    0x00000001: "ENCODER_ERROR_UNSTABLE_GAIN",
    0x00000002: "ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH",
    0x00000004: "ENCODER_ERROR_NO_RESPONSE",
    0x00000008: "ENCODER_ERROR_UNSUPPORTED_ENCODER_MODE",
    0x00000010: "ENCODER_ERROR_ILLEGAL_HALL_STATE",
    0x00000020: "ENCODER_ERROR_INDEX_NOT_FOUND_YET",
    0x00000040: "ENCODER_ERROR_ABS_SPI_TIMEOUT",
    0x00000080: "ENCODER_ERROR_ABS_SPI_COM_FAIL",
    0x00000100: "ENCODER_ERROR_ABS_SPI_NOT_READY",
};

let controllerErrors = {
    0x00000000: "ENCODER_ERROR_NONE",
    0x00000001: "ENCODER_ERROR_UNSTABLE_GAIN",
    0x00000002: "ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH",
    0x00000004: "ENCODER_ERROR_NO_RESPONSE",
    0x00000008: "ENCODER_ERROR_UNSUPPORTED_ENCODER_MODE",
    0x00000010: "ENCODER_ERROR_ILLEGAL_HALL_STATE",
    0x00000020: "ENCODER_ERROR_INDEX_NOT_FOUND_YET",
    0x00000040: "ENCODER_ERROR_ABS_SPI_TIMEOUT",
    0x00000080: "ENCODER_ERROR_ABS_SPI_COM_FAIL",
    0x00000100: "ENCODER_ERROR_ABS_SPI_NOT_READY",
}

export default {
    name: "Axis",
    props: ["axis", "odrives"],
    data() {
        return {
            showError: false,
        };
    },
    computed: {
        axisErrorMsg() {
            let retMsg = "none";
            let errCode = parseInt(this.axis.ref.error.val);

            if (errCode != 0) {
                // we got an error!
                let errs = []
                for (const errKey of Object.keys(axisErrors)) {
                    if (errCode & errKey) {
                        errs.push(axisErrors[errKey]);
                    }
                }
                retMsg = "";
                for (const err of errs){
                    retMsg = retMsg + " " + err;
                }
            }

            return retMsg;
        },
        motorErrorMsg() {
            let retMsg = "none";
            let errCode = parseInt(this.axis.ref.motor.error.val);

            if (errCode != 0) {
                // we got an error!
                let errs = []
                for (const errKey of Object.keys(motorErrors)) {
                    if (errCode & errKey) {
                        errs.push(motorErrors[errKey]);
                    }
                }
                retMsg = "";
                for (const err of errs){
                    retMsg = retMsg + " " + err;
                }
            }


            return retMsg;
        },
        encoderErrorMsg() {
            let retMsg = "none";
            let errCode = parseInt(this.axis.ref.encoder.error.val);
            
            if (errCode != 0) {
                // we got an error!
                let errs = []
                for (const errKey of Object.keys(encoderErrors)) {
                    if (errCode & errKey) {
                        errs.push(encoderErrors[errKey]);
                    }
                }
                retMsg = "";
                for (const err of errs){
                    retMsg = retMsg + " " + err;
                }
            }


            return retMsg;
        },
        controllerErrorMsg() {
            let retMsg = "none";
            let errCode = parseInt(this.axis.ref.controller.error.val);

            if (errCode != 0) {
                // we got an error!
                let errs = []
                for (const errKey of Object.keys(controllerErrors)) {
                    if (errCode & errKey) {
                        errs.push(controllerErrors[errKey]);
                    }
                }
                retMsg = "";
                for (const err of errs){
                    retMsg = retMsg + " " + err;
                }
            }


            return retMsg;
        },
        axisError() {
            return this.axis.ref.error.val !== "0";
        },
        motorError() {
            return this.axis.ref.motor.error.val !== "0";
        },
        encoderError() {
            return this.axis.ref.encoder.error.val !== "0";
        },
        controllerError() {
            return this.axis.ref.controller.error.val !== "0";
        }
    }
}
</script>

<style scoped>
.axis{
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

</style>