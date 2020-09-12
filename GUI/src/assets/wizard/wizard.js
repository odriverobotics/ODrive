import odriveEnums from "../odriveEnums.json";

// wizard is an object of pages.
// each page has a number, title, vue component name, next destination, back destination, and array of choices
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
            },
            {
                imageURL: require("../images/56v_300x300.png"),
                text: "ODrive v3.6 56V",
            },
        ],
        customComponents: [],
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
                configStub: {
                    axis0: {
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
                configStub: {
                    axis0: {
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
                    axis: "axis0",
                }
            },
        ],
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
                configStub: {
                    axis0: {
                        encoder: {
                            config: {
                                cpr: 8192,
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
                configStub: {
                    axis0: {
                        encoder: {
                            config: {
                                cpr: 8192,
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
        ]
    }
}