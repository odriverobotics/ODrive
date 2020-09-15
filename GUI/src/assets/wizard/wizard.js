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
                tooltip: null,
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
                tooltip: null,
            },
        ],
        customComponents: [
            {
                component: "wizardMotor",
                id: 0,
                data: {
                    axis: "axis0",
                },
                tooltip: null,
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
                tooltip: null,
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
                tooltip: null,
            },
        ],
        customComponents: [
            {
                component: "wizardEncoderIncremental",
                id: 0,
                data: {
                    axis: "axis0",
                },
                tooltip: null,
            },
            {
                component: "wizardEncoderIncrementalIndex",
                id: 1,
                data: {
                    axis: "axis0",
                },
                tooltip: null,
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
        link: "Control 0",
        component: "wizardPage",
        next: "Misc_0",
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
                tooltip: null,
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
                tooltip: null,
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
                tooltip: null,
            },
        ],
        customComponents: [
            {
                component: "wizardMotor",
                id: 0,
                data: {
                    axis: "axis1",
                },
                tooltip: null,
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
                tooltip: null,
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
                tooltip: null,
            },
        ],
        customComponents: [
            {
                component: "wizardEncoderIncremental",
                id: 0,
                data: {
                    axis: "axis1",
                },
                tooltip: null,
            },
            {
                component: "wizardEncoderIncrementalIndex",
                id: 1,
                data: {
                    axis: "axis1",
                },
                tooltip: null,
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
        link: "Control 1",
        component: "wizardPage",
        next: "Misc_1",
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
    Misc_1: {
        title: "Finishing touches for Axis 1",
        link: "Misc 1",
        component: "wizardPage",
        next: "End",
        back: "Encoder_1",
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
                tooltip: null,
            }
        ],
        pageComponents: [],
    }
}