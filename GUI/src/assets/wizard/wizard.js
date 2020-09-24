import odriveEnums from "../odriveEnums.json";

// wizard is an object of pages.
// each page has a number, title, vue component name, next destination, back destination, and array of choices
// certain choices require a hook - for example, hall effect encoder cpr is 6 * motor.config.pole_pairs
// each hook takes a config object, modifies it, and returns it after running appropriate calculations.
// see choiceHandler() in Wizard.vue for how this happens

// Pages can also have custom components displayed below the choice tiles
// Example - on motor pages, there are buttons to run motor calibration and clear errors.
export let pages = {
    ODrive: {
        title: "Which ODrive do you have?",
        link: "ODrive Version",
        component: "wizardPage",
        next: "Brake",
        back: "ODrive",
        nextTooltip: "Make a choice!",
        choiceMade: false,
        choices: [
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/24v_300x300.png"),
                },
                title: "ODrive v3.6 24V",
                requirements: [],
                hooks: [],
                tooltip: null,
                altTooltip: null,
            },
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/56v_300x300.png"),
                },
                title: "ODrive v3.6 56V",
                requirements: [],
                hooks: [],
                tooltip: null,
                altTooltip: null,
            },
        ],
        customComponents: [],
        pageComponents: [],
    },
    Brake: {
        title: "Brake Resistor",
        link: "Brake Resistor",
        component: "wizardPage",
        next: "Motor_0",
        back: "ODrive",
        nextTooltip: "Make a choice!",
        choiceMade: false,
        choices: [
            {
                component: "wizardBrake",
                data: {},
                title: "Brake Resistor",
                requirements: [],
                hooks: [],
                tooltip: "When you slow down a motor, the energy has to go somewhere.\n\r \
                            It either goes to the brake resistor or back to your power supply.",
                altTooltip: null,
            },
            {
                component: "wizardChoice",
                data: {
                    imageURL: null,
                    configStub: {
                        config: {
                            brake_resistance: 0,
                        }
                    },
                },
                title: "I don't have a brake resistor",
                hooks: [],
                requirements: [],
                tooltip: "Only operate without a brake resistor if your power source can handle reverse current, like a battery."
            }
        ],
    },
    Motor_0: {
        title: "Which motor are you using on Axis 0?",
        link: "Motor 0",
        component: "wizardPage",
        next: "Encoder_0",
        back: "Brake",
        nextTooltip: "ODrive needs to know the inductance and resistance of your motor in order to control it.\
                        Click the Calibrate Motor button to start the measurement process. The motor will beep\
                        during calibration.",
        choiceMade: false,
        choices: [
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/D5065_300x300.png"),
                    configStub: {
                        axis0: {
                            motor: {
                                config: {
                                    pole_pairs: 7,
                                    torque_constant: 8.27 / 270,
                                    // R and L come from "Calibrate Motor" button on page
                                    //phase_resistance: 0.039, // from test rig
                                    //phase_inductance: 1.57e-5,
                                    motor_type: odriveEnums.MOTOR_TYPE_HIGH_CURRENT, // MOTOR_TYPE_HIGH_CURRENT
                                },
                            },
                        },
                    },
                },
                title: "ODrive D5065",
                requirements: [],
                hooks: [],
                tooltip: "D5065 motor from the ODrive shop",
                altTooltip: null,
            },
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/D6374_300x300.png"),
                    configStub: {
                        axis0: {
                            motor: {
                                config: {
                                    pole_pairs: 7,
                                    torque_constant: 8.27 / 150,
                                    // R and L come from "Calibrate Motor" button on page
                                    //phase_resistance: 0.041, // measurement from PJ
                                    //phase_inductance: 2.23e-5,
                                    motor_type: odriveEnums.MOTOR_TYPE_HIGH_CURRENT, // MOTOR_TYPE_HIGH_CURRENT,
                                },
                            },
                        },
                    },
                },
                title: "ODrive D6374",
                requirements: [],
                hooks: [],
                tooltip: "D6374 motor from the ODrive shop",
                altTooltip: null,
            },
            {
                component: "wizardMotor",
                data: {
                    axis: "axis0",
                },
                title: "Other Motor",
                requirements: [],
                hooks: [],
                tooltip: "Bring your own motor!",
                altTooltip: null,
            },
        ],
        pageComponents: [
            {
                component: "wizardMotorCal",
                id: 0,
                data: {
                    axis: "axis0",
                }
            },
            {
                component: "wizardCalStatus",
                id: 1,
                data: {
                    success: "Calibration succeeded!",
                    failure: "Calibration failed",
                    tooltip: "Double check your motor connections. If the connections are good, it is \
                                possible that your motor has a higher resistance or lower inductance than ODrive expects."
                }
            }
        ]
    },
    Encoder_0: {
        title: "Which encoder are you using for Axis 0?",
        link: "Encoder 0",
        component: "wizardPage",
        next: "Control_0",
        back: "Motor_0",
        nextTooltip: "ODrive needs to know the relation between the encoder position and motor position in order to function. \
                        Click the Calibrate Encoder button to start the measurement process. The motor will slowly rotate back \
                        and forth during calibration.",
        choiceMade: false,
        choices: [
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/amt102-v_300x300.png"),
                    configStub: {
                        axis0: {
                            encoder: {
                                config: {
                                    cpr: 8192,
                                    use_index: true,
                                    mode: odriveEnums.ENCODER_MODE_INCREMENTAL, // ENCODER_MODE_INCREMENTAL
                                }
                            }
                        }
                    },
                },
                title: "CUI AMT102V",
                requirements: [],
                hooks: [],
                tooltip: "Incremental 8192cpr encoder from the ODrive shop",
                altTooltip: null,
            },
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/hall_effect_300x300.png"),
                    configStub: {
                        axis0: {
                            encoder: {
                                config: {
                                    use_index: false,
                                    mode: odriveEnums.ENCODER_MODE_HALL,
                                }
                            }
                        }
                    },
                },
                title: "Hall Effect",
                requirements: [],
                hooks: [
                    // hook takes a config object (normally this.wizardConfig in Wizard.vue), creates and returns a copy
                    (configObj) => {
                        let newConfig = configObj;
                        newConfig.axis0.encoder.config.cpr = 6 * newConfig.axis0.motor.config.pole_pairs;
                        return newConfig;
                    },
                ],
                tooltip: "If your motor is 'sensored' and you don't have another encoder, select this option",
                altTooltip: null,
            },
            {
                component: "wizardEncoderIncremental",
                data: {
                    axis: "axis0",
                },
                title: "Incremental Without Index",
                requirements: [],
                hooks: [],
                tooltip: "Generic incremental encoder without index. Set cpr to 4 * PPR (pulses per revolution).",
                altTooltip: null,
            },
            {
                component: "wizardEncoderIncrementalIndex",
                data: {
                    axis: "axis0",
                },
                title: "Incremental With Index",
                requirements: [],
                hooks: [],
                tooltip: "Generic incremental encoder with index. Set cpr to 4 * PPR (pulses per revolution).",
                altTooltip: null,
            },
        ],
        pageComponents: [
            {
                component: "wizardEncoderCal",
                id: 0,
                data: {
                    axis: "axis0",
                }
            },
            {
                component: "wizardCalStatus",
                id: 1,
                data: {
                    success: "Calibration succeeded!",
                    failure: "Calibration failed",
                    tooltip: "Double check your encoder connections. If the connections are good, you \
                                have a wrong setting for motor pole pairs or encoder cpr."
                }
            }
        ]
    },
    Control_0: {
        title: "Control mode for Axis 0",
        link: "Control Modes 0",
        component: "wizardPage",
        next: "Misc_0",
        back: "Encoder_0",
        nextTooltip: "Make a choice!",
        choiceMade: false,
        choices: [
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/temp.png"),
                    configStub: {
                        axis0: {
                            controller: {
                                config: {
                                    control_mode: odriveEnums.CONTROL_MODE_POSITION_CONTROL,
                                },
                            },
                        },
                    },
                },
                title: "Position Control",
                requirements: [],
                hooks: [],
                tooltip: "Use this mode to control the position of the motor",
                altTooltip: null,
            },
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/temp.png"),
                    configStub: {
                        axis0: {
                            controller: {
                                config: {
                                    control_mode: odriveEnums.CONTROL_MODE_VELOCITY_CONTROL,
                                },
                            },
                        },
                    },
                },
                title: "Velocity Control",
                requirements: [],
                hooks: [],
                tooltip: "Use this mode to control the velocity of the motor",
                altTooltip: null,
            },
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/temp.png"),
                    configStub: {
                        axis0: {
                            controller: {
                                config: {
                                    control_mode: odriveEnums.CONTROL_MODE_TORQUE_CONTROL,
                                },
                            },
                        },
                    },
                },
                title: "Torque Control",
                requirements: [],
                hooks: [],
                tooltip: "Use this mode to control the torque output of the motor",
                altTooltip: null,
            },
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/temp.png"),
                    configStub: {
                        axis0: {
                            controller: {
                                config: {
                                    control_mode: odriveEnums.CONTROL_MODE_VOLTAGE_CONTROL,
                                },
                            },
                        },
                    },
                },
                title: "Voltage Control",
                requirements: [],
                hooks: [],
                tooltip: "This mode is used for gimbal motors",
                altTooltip: null,
            },
        ],
        customComponents: [],
        pageComponents: [],
    },
    /*
    Input_0: {
        title: "Input mode selection for Axis 0",
        link: "Input Modes 0",
        component: "wizardPage",
        next: "Misc_0",
        back: "Control_0",
        nextTooltip: "Make a choice!",
        choiceMade: false,
        choices: [
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/temp.png"),
                    configStub: {
                        axis0: {
                            controller: {
                                config: {
                                    input_mode: odriveEnums.INPUT_MODE_PASSTHROUGH,
                                },
                            },
                        },
                    },
                },
                title: "Input Passthrough",
                requirements: [],
                hooks: [],
                tooltip: "This is the default input mode. Input commands are passed to the controller without modification.",
                altTooltip: null,
            },
            {
                component: "wizardInputFiltered",
                data: {
                    axis: "axis0",
                },
                title: "Position Input Filtering",
                requirements: [
                    (configObj) => {
                        return configObj.axis0.controller.config.control_mode == odriveEnums.CONTROL_MODE_POSITION_CONTROL;
                    },
                ],
                hooks: [],
                tooltip: "Filtered position mode. This provides smoother motion than the passthrough mode. \
                            Input position commands are sent through a 2nd order filter. A higher bandwidth will cause quicker position changes.",
                altTooltip: "Control mode must be Position Control for this input mode",
            },
            {
                component: "wizardInputVelRamp",
                data: {
                    axis: "axis0",
                },
                title: "Velocity Input Ramping",
                requirements: [
                    (configObj) => {
                        return configObj.axis0.controller.config.control_mode == odriveEnums.CONTROL_MODE_VELOCITY_CONTROL;
                    },
                ],
                hooks: [],
                tooltip: "For velocity control mode, you can choose to have the input velocities ramped between values. vel_ramp_rate is the \
                            acceleration between velocities.",
                altTooltip: "Control mode must be Velocity Control for this input mode",
            },
            {
                component: "wizardInputTrajectory",
                data: {
                    axis: "axis0",
                },
                title: "Trajectory Planning",
                requirements: [
                    (configObj) => {
                        return configObj.axis0.controller.config.control_mode == odriveEnums.CONTROL_MODE_POSITION_CONTROL;
                    },
                ],
                hooks: [],
                tooltip: "This mode lets you smoothly accelerate, coast, and decelerate from one position to another. vel_limit is your coasting speed, \
                            accel_limit is the maximum acceleration in turns/s^2, decel_limit is the maximum decelaration in turns/s^2 and inertia is an \
                            optional value to correlate acceleration with motor torque.",
                altTooltip: "Control mode must be Position Control for this input mode",
            }
        ],
        pageComponents: [],
    },
    */
    Misc_0: {
        title: "Finishing touches for Axis 0",
        link: "Misc 0",
        component: "wizardPage",
        next: "Motor_1",
        back: "Control_0",
        nextTooltip: "Make a choice!",
        choiceMade: false,
        choices: [
            {
                component: "wizardMisc",
                data: {
                    axis: "axis0",
                },
                title: "Miscellaneous Parameters",
                requirements: [],
                hooks: [],
                tooltip: null,
                altTooltip: null,
            },
        ],
        pageComponents: [],
    },
    Motor_1: {
        title: "Which motor are you using for Axis 1?",
        link: "Motor 1",
        component: "wizardPage",
        next: "Encoder_1",
        back: "Misc_0",
        nextTooltip: "ODrive needs to know the inductance and resistance of your motor in order to control it.\
                        Click the Calibrate Motor button to start the measurement process. The motor will beep\
                        during calibration.",
        choiceMade: false,
        choices: [
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/D5065_300x300.png"),
                    configStub: {
                        axis1: {
                            motor: {
                                config: {
                                    pole_pairs: 7,
                                    torque_constant: 8.27 / 270,
                                    // R and L come from "Calibrate Motor" button on page
                                    //phase_resistance: 0.039, // from test rig
                                    //phase_inductance: 1.57e-5,
                                    motor_type: odriveEnums.MOTOR_TYPE_HIGH_CURRENT, // MOTOR_TYPE_HIGH_CURRENT
                                },
                            },
                        },
                    },
                },
                title: "ODrive D5065",
                requirements: [],
                hooks: [],
                tooltip: "D5065 motor from the ODrive shop",
                altTooltip: null,
            },
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/D6374_300x300.png"),
                    configStub: {
                        axis1: {
                            motor: {
                                config: {
                                    pole_pairs: 7,
                                    torque_constant: 8.27 / 150,
                                    // R and L come from "Calibrate Motor" button on page
                                    //phase_resistance: 0.041, // measurement from PJ
                                    //phase_inductance: 2.23e-5,
                                    motor_type: odriveEnums.MOTOR_TYPE_HIGH_CURRENT, // MOTOR_TYPE_HIGH_CURRENT,
                                },
                            },
                        },
                    },
                },
                title: "ODrive D6374",
                requirements: [],
                hooks: [],
                tooltip: "D6374 motor from the ODrive shop",
                altTooltip: null,
            },
            {
                component: "wizardMotor",
                data: {
                    axis: "axis1",
                },
                title: "Other Motor",
                requirements: [],
                hooks: [],
                tooltip: "Bring your own motor!",
                altTooltip: null,
            },
        ],
        pageComponents: [
            {
                component: "wizardMotorCal",
                id: 0,
                data: {
                    axis: "axis1",
                }
            },
            {
                component: "wizardCalStatus",
                id: 1,
                data: {
                    success: "Calibration succeeded!",
                    failure: "Calibration failed",
                    tooltip: "Double check your motor connections. If the connections are good, it is \
                                possible that your motor has a higher resistance or lower inductance than ODrive expects."
                }
            }
        ]
    },
    Encoder_1: {
        title: "Which encoder are you using for Axis 1?",
        link: "Encoder 1",
        component: "wizardPage",
        next: "Control_1",
        back: "Motor_1",
        nextTooltip: "ODrive needs to know the relation between the encoder position and motor position in order to function. \
                        Click the Calibrate Encoder button to start the measurement process. The motor will slowly rotate back \
                        and forth during calibration.",
        choiceMade: false,
        choices: [
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/amt102-v_300x300.png"),
                    configStub: {
                        axis1: {
                            encoder: {
                                config: {
                                    cpr: 8192,
                                    use_index: true,
                                    mode: odriveEnums.ENCODER_MODE_INCREMENTAL, // ENCODER_MODE_INCREMENTAL
                                }
                            }
                        }
                    },
                },
                title: "CUI AMT102V",
                requirements: [],
                hooks: [],
                tooltip: "Incremental 8192cpr encoder from the ODrive shop",
                altTooltip: null,
            },
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/hall_effect_300x300.png"),
                    configStub: {
                        axis1: {
                            encoder: {
                                config: {
                                    use_index: false,
                                    mode: odriveEnums.ENCODER_MODE_HALL,
                                }
                            }
                        }
                    },
                },
                title: "Hall Effect",
                requirements: [],
                hooks: [
                    // hook takes a config object (normally this.wizardConfig in Wizard.vue), creates and returns a copy
                    (configObj) => {
                        let newConfig = configObj;
                        newConfig.axis1.encoder.config.cpr = 6 * newConfig.axis1.motor.config.pole_pairs;
                        return newConfig;
                    },
                ],
                tooltip: "If your motor is 'sensored' and you don't have another encoder, select this option",
                altTooltip: null,
            },
            {
                component: "wizardEncoderIncremental",
                data: {
                    axis: "axis1",
                },
                title: "Incremental Without Index",
                requirements: [],
                hooks: [],
                tooltip: "Generic incremental encoder without index. Set cpr to 4 * PPR (pulses per revolution).",
                altTooltip: null,
            },
            {
                component: "wizardEncoderIncrementalIndex",
                data: {
                    axis: "axis1",
                },
                title: "Incremental With Index",
                requirements: [],
                hooks: [],
                tooltip: "Generic incremental encoder with index. Set cpr to 4 * PPR (pulses per revolution).",
                altTooltip: null,
            },
        ],
        pageComponents: [
            {
                component: "wizardEncoderCal",
                id: 0,
                data: {
                    axis: "axis1",
                }
            },
            {
                component: "wizardCalStatus",
                id: 1,
                data: {
                    success: "Calibration succeeded!",
                    failure: "Calibration failed",
                    tooltip: "Double check your encoder connections. If the connections are good, you either \
                                have a wrong setting for motor pole pairs or encoder cpr."
                }
            }
        ]
    },
    Control_1: {
        title: "Control mode for Axis 1",
        link: "Control Modes 1",
        component: "wizardPage",
        next: "Misc_1",
        back: "Encoder_1",
        nextTooltip: "Make a choice!",
        choiceMade: false,
        choices: [
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/temp.png"),
                    configStub: {
                        axis1: {
                            controller: {
                                config: {
                                    control_mode: odriveEnums.CONTROL_MODE_POSITION_CONTROL,
                                },
                            },
                        },
                    },
                },
                title: "Position Control",
                requirements: [],
                hooks: [],
                tooltip: "Use this mode to control the position of the motor",
                altTooltip: null,
            },
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/temp.png"),
                    configStub: {
                        axis1: {
                            controller: {
                                config: {
                                    control_mode: odriveEnums.CONTROL_MODE_VELOCITY_CONTROL,
                                },
                            },
                        },
                    },
                },
                title: "Velocity Control",
                requirements: [],
                hooks: [],
                tooltip: "Use this mode to control the velocity of the motor",
                altTooltip: null,
            },
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/temp.png"),
                    configStub: {
                        axis1: {
                            controller: {
                                config: {
                                    control_mode: odriveEnums.CONTROL_MODE_TORQUE_CONTROL,
                                },
                            },
                        },
                    },
                },
                title: "Torque Control",
                requirements: [],
                hooks: [],
                tooltip: "Use this mode to control the torque output of the motor",
                altTooltip: null,
            },
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/temp.png"),
                    configStub: {
                        axis1: {
                            controller: {
                                config: {
                                    control_mode: odriveEnums.CONTROL_MODE_VOLTAGE_CONTROL,
                                },
                            },
                        },
                    },
                },
                title: "Voltage Control",
                requirements: [],
                hooks: [],
                tooltip: "This mode is used for gimbal motors",
                altTooltip: null,
            },
        ],
        customComponents: [],
        pageComponents: [],
    },
    /*
    Input_1: {
        title: "Input mode selection for Axis 1",
        link: "Input Modes 1",
        component: "wizardPage",
        next: "Misc_1",
        back: "Control_1",
        nextTooltip: "Make a choice!",
        choiceMade: false,
        choices: [
            {
                component: "wizardChoice",
                data: {
                    imageURL: require("../images/temp.png"),
                    configStub: {
                        axis1: {
                            controller: {
                                config: {
                                    input_mode: odriveEnums.INPUT_MODE_PASSTHROUGH,
                                },
                            },
                        },
                    },
                },
                title: "Input Passthrough",
                requirements: [],
                hooks: [],
                tooltip: "This is the default input mode. Input commands are passed to the controller without modification.",
                altTooltip: null,
            },
            {
                component: "wizardInputFiltered",
                data: {
                    axis: "axis1",
                },
                title: "Position Input Filtering",
                requirements: [
                    (configObj) => {
                        return configObj.axis1.controller.config.control_mode == odriveEnums.CONTROL_MODE_POSITION_CONTROL;
                    },
                ],
                hooks: [],
                tooltip: "Filtered position mode. This provides smoother motion than the passthrough mode. \
                            Input position commands are sent through a 2nd order filter. A higher bandwidth will cause quicker position changes.",
                altTooltip: "Control mode must be Position Control for this input mode",
            },
            {
                component: "wizardInputVelRamp",
                data: {
                    axis: "axis1",
                },
                title: "Velocity Input Ramping",
                requirements: [
                    (configObj) => {
                        return configObj.axis1.controller.config.control_mode == odriveEnums.CONTROL_MODE_VELOCITY_CONTROL;
                    },
                ],
                hooks: [],
                tooltip: "For velocity control mode, you can choose to have the input velocities ramped between values. vel_ramp_rate is the \
                            acceleration between velocities.",
                altTooltip: "Control mode must be Velocity Control for this input mode",
            },
            {
                component: "wizardInputTrajectory",
                data: {
                    axis: "axis1",
                },
                title: "Trajectory Planning",
                requirements: [
                    (configObj) => {
                        return configObj.axis1.controller.config.control_mode == odriveEnums.CONTROL_MODE_POSITION_CONTROL;
                    },
                ],
                hooks: [],
                tooltip: "This mode lets you smoothly accelerate, coast, and decelerate from one position to another. vel_limit is your coasting speed, \
                            accel_limit is the maximum acceleration in turns/s^2, decel_limit is the maximum decelaration in turns/s^2 and inertia is an \
                            optional value to correlate acceleration with motor torque.",
                altTooltip: "Control most must be Position Control for this input mode",
            }
        ],
        pageComponents: [],
    },
    */
    Misc_1: {
        title: "Finishing touches for Axis 1",
        link: "Misc 1",
        component: "wizardPage",
        next: "End",
        back: "Control_1",
        nextTooltip: "Make a choice!",
        choiceMade: false,
        choices: [
            {
                component: "wizardMisc",
                data: {
                    axis: "axis1",
                },
                title: "Miscellaneous Parameters",
                requirements: [],
                hooks: [],
                tooltip: null,
                altTooltip: null,
            },
        ],
        pageComponents: [],
    },
    End: {
        title: "Review choices",
        link: "Review and Apply",
        component: "wizardPage",
        next: "End",
        back: "Misc_1",
        choiceMade: false,
        choices: [
            {
                component: "wizardEnd",
                data: {},
                title: null,
                requirements: [],
                hooks: [],
                tooltip: "Old config values are on the left, new config values are on the right in green",
                altTooltip: null,
            }
        ],
        pageComponents: [],
    }
}

export let enumVars = {
    // for wizardEnd.vue
    motor_type: {
        0: "MOTOR_TYPE_HIGH_CURRENT",
        2: "MOTOR_TYPE_GIMBAL",
        3: "MOTOR_TYPE_ACIM",
    },
    mode: {
        0: "ENCODER_MODE_INCREMENTAL",
        1: "ENCODER_MODE_HALL",
        2: "ENCODER_MODE_SINCOS",
        256: "ENCODER_MODE_SPI_ABS_CUI",
        257: "ENCODER_MODE_SPI_ABS_AMS",
        258: "ENCODER_MODE_SPI_ABS_AEAT",
        259: "ENCODER_MODE_SPI_ABS_RLS",
    },
    input_mode: {
        0: "INPUT_MODE_INACTIVE",
        1: "INPUT_MODE_PASSTHROUGH",
        2: "INPUT_MODE_VEL_RAMP",
        3: "INPUT_MODE_POS_FILTER",
        4: "INPUT_MODE_MIX_CHANNELS",
        5: "INPUT_MODE_TRAP_TRAJ",
        6: "INPUT_MODE_TORQUE_RAMP",
        7: "INPUT_MODE_MIRROR",
    },
    control_mode: {
        0: "CONTROL_MODE_VOLTAGE_CONTROL",
        1: "CONTROL_MODE_TORQUE_CONTROL",
        2: "CONTROL_MODE_VELOCITY_CONTROL",
        3: "CONTROL_MODE_POSITION_CONTROL",
    },
}