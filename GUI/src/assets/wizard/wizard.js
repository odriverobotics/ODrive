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
        number: 0,
        title: "Which ODrive do you have?",
        link: "ODrive Version",
        component: "wizardPage",
        next: "Motor_0",
        back: "ODrive",
        requirements: [],
        choices: [
            {
                imageURL: require("../images/24v_300x300.png"),
                text: "ODrive v3.6 24V",
                hooks: [],
            },
            {
                imageURL: require("../images/56v_300x300.png"),
                text: "ODrive v3.6 56V",
                hooks: [],
            },
        ],
        customComponents: [],
        pageComponents: [],
    },
    Motor_0: {
        number: 1,
        title: "Which motor are you using on Axis 0?",
        link: "Motor 0",
        component: "wizardPage",
        next: "Encoder_0",
        back: "ODrive",
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
            },
        ],
        customComponents: [
            {
                component: "wizardMotor",
                id: 0,
                data: {
                    axis: "axis0",
                }
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
        number: 2,
        title: "Which encoder are you using for Axis 0?",
        link: "Encoder 0",
        component: "wizardPage",
        next: "Misc_0",
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
            },
        ],
        customComponents: [
            {
                component: "wizardEncoderIncremental",
                id: 0,
                data: {
                    axis: "axis0",
                }
            },
            {
                component: "wizardEncoderIncrementalIndex",
                id: 1,
                data: {
                    axis: "axis0",
                }
            },
        ],
        pageComponents: [],
    },
    Misc_0: {
        number: 3,
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
                }
            },
        ],
        pageComponents: [],
    },
    Motor_1: {
        number: 4,
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
            },
        ],
        customComponents: [
            {
                component: "wizardMotor",
                id: 0,
                data: {
                    axis: "axis1",
                }
            },
        ],
        pageComponents: [],
    },
    Encoder_1: {
        number: 5,
        title: "Which encoder are you using for Axis 1?",
        link: "Encoder 1",
        component: "wizardPage",
        next: "Misc_1",
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
            },
        ],
        customComponents: [
            {
                component: "wizardEncoderIncremental",
                id: 0,
                data: {
                    axis: "axis1",
                }
            },
            {
                component: "wizardEncoderIncrementalIndex",
                id: 1,
                data: {
                    axis: "axis1",
                }
            },
        ],
        pageComponents: [],
    },
    Misc_1: {
        number: 6,
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
                }
            },
        ],
        pageComponents: [],
    },
    End: {
        number: 7,
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
            }
        ],
        pageComponents: [],
    }
}