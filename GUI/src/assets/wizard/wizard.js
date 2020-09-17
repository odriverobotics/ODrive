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
        requirements: [],
        choices: [
            {
                imageURL: require("../images/24v_300x300.png"),
                text: "ODrive v3.6 24V",
                hooks: [],
                tooltip: null,
            },
            {
                imageURL: require("../images/56v_300x300.png"),
                text: "ODrive v3.6 56V",
                hooks: [],
                tooltip: null,
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
        requirements: [],
        choices: [],
        customComponents: [
            {
                component: "wizardBrake",
                id: 0,
                data: {},
                tooltip: "When you slow down a motor, the energy has to go somewhere.\n\r \
                            It either goes to the brake resistor or back to your power supply."
            }
        ],
    },
    Motor_0: {
        title: "Which motor are you using on Axis 0?",
        link: "Motor 0",
        component: "wizardPage",
        next: "Encoder_0",
        back: "Brake",
        requirements: [],
        choices: [
            {
                imageURL: require("../images/D5065_300x300.png"),
                text: "ODrive D5065",
                hooks: [],
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
                tooltip: "D5065 motor from the ODrive shop",
            },
            {
                imageURL: require("../images/D6374_300x300.png"),
                text: "ODrive D6374",
                hooks: [],
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
                tooltip: "D6374 motor from the ODrive shop",
            },
        ],
        customComponents: [
            {
                component: "wizardMotor",
                id: 0,
                data: {
                    axis: "axis0",
                },
                tooltip: "Bring your own motor!",
            },
        ],
        pageComponents: [
            {
                component: "wizardMotorMeasure",
                id: 0,
                data: {
                    axis: "axis0",
                }
            },
            {
                component: "wizardClearErrors",
                id: 1,
                data: {
                    axis: "axis0"
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
        requirements: [],
        choices: [
            {
                imageURL: require("../images/amt102-v_300x300.png"),
                text: "CUI AMT102V",
                hooks: [],
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
                tooltip: "Incremental 8192cpr encoder from the ODrive shop",
            },
            {
                imageURL: require("../images/hall_effect_300x300.png"),
                text: "Hall Effect",
                hooks: [
                    // hook takes a config object (normally this.wizardConfig in Wizard.vue), creates and returns a copy
                    (configObj) => {
                        let newConfig = configObj;
                        newConfig.axis0.encoder.config.cpr = 6 * newConfig.axis0.motor.config.pole_pairs;
                        return newConfig;
                    },
                ],
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
                tooltip: "If your motor is 'sensored' and you don't have another encoder, select this option",
            },
        ],
        customComponents: [
            {
                component: "wizardEncoderIncremental",
                id: 0,
                data: {
                    axis: "axis0",
                },
                tooltip: "Generic incremental encoder without index. Set cpr to 4 * PPR (pulses per revolution).",
            },
            {
                component: "wizardEncoderIncrementalIndex",
                id: 1,
                data: {
                    axis: "axis0",
                },
                tooltip: "Generic incremental encoder with index. Set cpr to 4 * PPR (pulses per revolution).",
            },
        ],
        pageComponents: [
            /* This needs some thought - encoder cpr must be applied to ODrive before encoder calibration can be performed
            What is desired is a "sanity check" to make sure that cpr is set correctly before proceeding
            {
                component: "wizardEncoderCal",
                id: 0,
                data: {
                    axis: "axis0",
                }
            },
            {
                component: "wizardClearErrors",
                id: 1,
                data: {
                    axis: "axis0"
                }
            }
            */
        ]
    },
    Control_0: {
        title: "Control mode for Axis 0",
        link: "Control Modes 0",
        component: "wizardPage",
        next: "Input_0",
        back: "Encoder_0",
        requirements: [],
        choices: [
            {
                imageURL: require("../images/temp.png"),
                text: "Position Control",
                hooks: [],
                configStub: {
                    axis0: {
                        controller: {
                            config: {
                                control_mode: odriveEnums.CONTROL_MODE_POSITION_CONTROL,
                            },
                        },
                    },
                },
                tooltip: "Use this mode to control the position of the motor"
            },
            {
                imageURL: require("../images/temp.png"),
                text: "Velocity Control",
                hooks: [],
                configStub: {
                    axis0: {
                        controller: {
                            config: {
                                control_mode: odriveEnums.CONTROL_MODE_VELOCITY_CONTROL,
                            },
                        },
                    },
                },
                tooltip: "Use this mode to control the velocity of the motor"
            },
            {
                imageURL: require("../images/temp.png"),
                text: "Torque Control",
                hooks: [],
                configStub: {
                    axis0: {
                        controller: {
                            config: {
                                control_mode: odriveEnums.CONTROL_MODE_TORQUE_CONTROL,
                            },
                        },
                    },
                },
                tooltip: "Use this mode to control the torque output of the motor"
            },
            {
                imageURL: require("../images/temp.png"),
                text: "Voltage Control",
                hooks: [],
                configStub: {
                    axis0: {
                        controller: {
                            config: {
                                control_mode: odriveEnums.CONTROL_MODE_VOLTAGE_CONTROL,
                            },
                        },
                    },
                },
                tooltip: "This mode is used for gimbal motors"
            },
        ],
        customComponents: [],
        pageComponents: [],
    },
    Input_0: {
        title: "Input mode selection for Axis 0",
        link: "Input Modes 0",
        component: "wizardPage",
        next: "Misc_0",
        back: "Control_0",
        requirements: [],
        choices: [
            {
                imageURL: require("../images/temp.png"),
                text: "Input Passthrough",
                hooks: [],
                configStub: {
                    axis0: {
                        controller: {
                            config: {
                                input_mode: odriveEnums.INPUT_MODE_PASSTHROUGH,
                            },
                        },
                    },
                },
                tooltip: "This is the default input mode. Input commands are passed to the controller without modification."
            },
        ],
        customComponents: [
            {
                component: "wizardInputFiltered",
                id: 0,
                data: {
                    axis: "axis0",
                },
                tooltip: "Filtered position mode. This provides smoother motion than the passthrough mode. \
                            Input position commands are sent through a 2nd order filter. A higher bandwidth will cause quicker position changes.",
            },
            {
                component: "wizardInputVelRamp",
                id: 1,
                data: {
                    axis: "axis0",
                },
                tooltip: "For velocity control mode, you can choose to have the input velocities ramped between values. vel_ramp_rate is the \
                            acceleration between velocities."
            },
            {
                component: "wizardInputTrajectory",
                id: 2,
                data: {
                    axis: "axis0",
                },
                tooltip: "This mode lets you smoothly accelerate, coast, and decelerate from one position to another. vel_limit is your coasting speed, \
                            accel_limit is the maximum acceleration in turns/s^2, decel_limit is the maximum decelaration in turns/s^2 and inertia is an \
                            optional value to correlate acceleration with motor torque."
            }
        ],
        pageComponents: [],
    },
    Misc_0: {
        title: "Finishing touches for Axis 0",
        link: "Misc 0",
        component: "wizardPage",
        next: "Motor_1",
        back: "Encoder_0",
        requirements: [],
        choices: [],
        customComponents: [
            {
                component: "wizardMisc",
                id: 0,
                data: {
                    axis: "axis0",
                },
                tooltip: "These config options don't fit anywhere else",
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
        requirements: [],
        choices: [
            {
                imageURL: require("../images/D5065_300x300.png"),
                text: "ODrive D5065",
                hooks: [],
                configStub: {
                    axis1: {
                        motor: {
                            config: {
                                pole_pairs: 7,
                                torque_constant: 8.27 / 270,
                                phase_resistance: 0.039, // from test rig
                                phase_inductance: 1.57e-5,
                                motor_type: odriveEnums.MOTOR_TYPE_HIGH_CURRENT, // MOTOR_TYPE_HIGH_CURRENT
                            },
                        },
                    },
                },
                tooltip: "D5065 motor from the ODrive shop",
            },
            {
                imageURL: require("../images/D6374_300x300.png"),
                text: "ODrive D6374",
                hooks: [],
                configStub: {
                    axis1: {
                        motor: {
                            config: {
                                pole_pairs: 7,
                                torque_constant: 8.27 / 150,
                                phase_resistance: 0.041, // measurement from PJ
                                phase_inductance: 2.23e-5,
                                motor_type: odriveEnums.MOTOR_TYPE_HIGH_CURRENT, // MOTOR_TYPE_HIGH_CURRENT,
                            },
                        },
                    },
                },
                tooltip: "D6374 motor from the ODrive shop",
            },
        ],
        customComponents: [
            {
                component: "wizardMotor",
                id: 0,
                data: {
                    axis: "axis1",
                },
                tooltip: "Bring your own motor!",
            },
        ],
        pageComponents: [
            {
                component: "wizardMotorMeasure",
                id: 0,
                data: {
                    axis: "axis1",
                }
            },
            {
                component: "wizardClearErrors",
                id: 1,
                data: {
                    axis: "axis1"
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
        requirements: [],
        choices: [
            {
                imageURL: require("../images/amt102-v_300x300.png"),
                text: "CUI AMT102V",
                hooks: [],
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
                tooltip: "Incremental 8192cpr encoder from the ODrive shop",
            },
            {
                imageURL: require("../images/hall_effect_300x300.png"),
                text: "Hall Effect",
                hooks: [
                    // hook takes a config object, creates and returns a copy
                    (configObj) => {
                        let newConfig = configObj;
                        newConfig.axis1.encoder.config.cpr = 6 * newConfig.axis1.motor.config.pole_pairs;
                        return newConfig;
                    },
                ],
                configStub: {
                    axis0: {
                        encoder: {
                            config: {
                                use_index: false,
                                mode: odriveEnums.ENCODER_MODE_HALL, // ENCODER_MODE_INCREMENTAL
                            }
                        }
                    }
                },
                tooltip: "If your motor is 'sensored' and you don't have another encoder, select this option",
            },
        ],
        customComponents: [
            {
                component: "wizardEncoderIncremental",
                id: 0,
                data: {
                    axis: "axis1",
                },
                tooltip: "Generic incremental encoder without index. Set cpr to 4 * PPR (pulses per revolution).",
            },
            {
                component: "wizardEncoderIncrementalIndex",
                id: 1,
                data: {
                    axis: "axis1",
                },
                tooltip: "Generic incremental encoder without index. Set cpr to 4 * PPR (pulses per revolution).",
            },
        ],
        pageComponents: [
            /*
            {
                component: "wizardEncoderCal",
                id: 0,
                data: {
                    axis: "axis1",
                }
            },
            {
                component: "wizardClearErrors",
                id: 1,
                data: {
                    axis: "axis1"
                }
            }
            */
        ]
    },
    Control_1: {
        title: "Control mode for Axis 1",
        link: "Control Modes 1",
        component: "wizardPage",
        next: "Input_1",
        back: "Encoder_1",
        requirements: [],
        choices: [
            {
                imageURL: require("../images/temp.png"),
                text: "Position Control",
                hooks: [],
                configStub: {
                    axis1: {
                        controller: {
                            config: {
                                control_mode: odriveEnums.CONTROL_MODE_POSITION_CONTROL,
                            },
                        },
                    },
                },
                tooltip: "Use this mode to control the position of the motor"
            },
            {
                imageURL: require("../images/temp.png"),
                text: "Velocity Control",
                hooks: [],
                configStub: {
                    axis1: {
                        controller: {
                            config: {
                                control_mode: odriveEnums.CONTROL_MODE_VELOCITY_CONTROL,
                            },
                        },
                    },
                },
                tooltip: "Use this mode to control the velocity of the motor"
            },
            {
                imageURL: require("../images/temp.png"),
                text: "Torque Control",
                hooks: [],
                configStub: {
                    axis1: {
                        controller: {
                            config: {
                                control_mode: odriveEnums.CONTROL_MODE_TORQUE_CONTROL,
                            },
                        },
                    },
                },
                tooltip: "Use this mode to control the torque output of the motor"
            },
            {
                imageURL: require("../images/temp.png"),
                text: "Voltage Control",
                hooks: [],
                configStub: {
                    axis1: {
                        controller: {
                            config: {
                                control_mode: odriveEnums.CONTROL_MODE_VOLTAGE_CONTROL,
                            },
                        },
                    },
                },
                tooltip: "This mode is used for gimbal motors"
            },
        ],
        customComponents: [],
        pageComponents: [],
    },
    Input_1: {
        title: "Input mode selection for Axis 1",
        link: "Input Modes 1",
        component: "wizardPage",
        next: "Misc_1",
        back: "Control_1",
        requirements: [],
        choices: [
            {
                imageURL: require("../images/temp.png"),
                text: "Input Passthrough",
                hooks: [],
                configStub: {
                    axis0: {
                        controller: {
                            config: {
                                input_mode: odriveEnums.INPUT_MODE_PASSTHROUGH,
                            },
                        },
                    },
                },
                tooltip: "This is the default input mode. Input commands are passed to the controller without modification."
            },
        ],
        customComponents: [
            {
                component: "wizardInputFiltered",
                id: 0,
                data: {
                    axis: "axis1",
                },
                tooltip: "Filtered position mode. This provides smoother motion than the passthrough mode. \
                            Input position commands are sent through a 2nd order filter. A higher bandwidth will cause quicker position changes.",
            },
            {
                component: "wizardInputVelRamp",
                id: 1,
                data: {
                    axis: "axis1",
                },
                tooltip: "For velocity control mode, you can choose to have the input velocities ramped between values. vel_ramp_rate is the \
                            acceleration between velocities."
            },
            {
                component: "wizardInputTrajectory",
                id: 2,
                data: {
                    axis: "axis1",
                },
                tooltip: "This mode lets you smoothly accelerate, coast, and decelerate from one position to another. vel_limit is your coasting speed, \
                            accel_limit is the maximum acceleration in turns/s^2, decel_limit is the maximum decelaration in turns/s^2 and inertia is an \
                            optional value to correlate acceleration with motor torque."
            }
        ],
        pageComponents: [],
    },
    Misc_1: {
        title: "Finishing touches for Axis 1",
        link: "Misc 1",
        component: "wizardPage",
        next: "End",
        back: "Input_1",
        requirements: [],
        choices: [],
        customComponents: [
            {
                component: "wizardMisc",
                id: 0,
                data: {
                    axis: "axis1",
                },
                tooltip: null,
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
        requirements: [],
        choices: [],
        customComponents: [
            {
                component: "wizardEnd",
                id: 0,
                tooltip: "Old config values are on the left, new config values are on the right",
            }
        ],
        pageComponents: [],
    }
}