from cantools.database import *
from odrive.enums import *

msgList = []
nodes = [can.Node('Master')]
buses = [can.Bus('ODrive', None, 100000)]

for axisID in range(0, 8):
    newNode = can.Node(f"ODrive_Axis{axisID}")
    nodes.append(newNode)

    # 0x00 - NMT Message (Reserved)

    # 0x001 - Heartbeat
    axisError = can.Signal("Axis_Error", 0, 32, receivers=['Master'], choices={error.value: error.name for error in AxisError})
    axisState = can.Signal("Axis_State", 32, 8, receivers=['Master'], choices={state.value: state.name for state in AxisState})
    motorErrorFlag = can.Signal("Motor_Error_Flag", 40, 1, receivers=['Master'])
    encoderErrorFlag = can.Signal("Encoder_Error_Flag", 48, 1, receivers=['Master'])
    controllerErrorFlag = can.Signal("Controller_Error_Flag", 56, 1, receivers=['Master'])
    trajectoryDoneFlag = can.Signal("Trajectory_Done_Flag", 63, 1, receivers=['Master'])

    heartbeatMsg = can.Message(
        0x001, "Heartbeat", 8, 
        [
            axisError, 
            axisState, 
            motorErrorFlag, 
            encoderErrorFlag, 
            controllerErrorFlag,
            trajectoryDoneFlag
        ], send_type='cyclic', cycle_time=100, senders=[newNode.name]
    )

    # 0x002 - E-Stop Message
    estopMsg = can.Message(0x002, "Estop", 0, [], senders=['Master'])

    # 0x003 - Motor Error
    motorError = can.Signal("Motor_Error", 0, 32, receivers=['Master'], choices={error.value: error.name for error in MotorError})
    motorErrorMsg = can.Message(0x003, "Get_Motor_Error", 8, [motorError], senders=[newNode.name])

    # 0x004 - Encoder Error
    encoderError = can.Signal("Encoder_Error", 0, 32, receivers=['Master'], choices={error.value: error.name for error in EncoderError})
    encoderErrorMsg = can.Message(
        0x004, "Get_Encoder_Error", 8, [encoderError], senders=[newNode.name]
    )

    # 0x005 - Sensorless Error
    sensorlessError = can.Signal("Sensorless_Error", 0, 32, receivers=['Master'], choices={error.value: error.name for error in SensorlessEstimatorError})
    sensorlessErrorMsg = can.Message(
        0x005, "Get_Sensorless_Error", 8, [sensorlessError], senders=[newNode.name]
    )

    # 0x006 - Axis Node ID
    axisNodeID = can.Signal("Axis_Node_ID", 0, 32, receivers=[newNode.name])
    axisNodeMsg = can.Message(0x006, "Set_Axis_Node_ID", 8, [axisNodeID], senders=['Master'])

    # 0x007 - Requested State
    axisRequestedState = can.Signal("Axis_Requested_State", 0, 32, receivers=[newNode.name], choices={state.value: state.name for state in AxisState})
    setAxisState = can.Message(
        0x007, "Set_Axis_State", 8, [axisRequestedState], senders=['Master']
    )

    # 0x008 - Startup Config (Reserved)

    # 0x009 - Encoder Estimates
    encoderPosEstimate = can.Signal("Pos_Estimate", 0, 32, is_float=True, receivers=['Master'], unit='rev')
    encoderVelEstimate = can.Signal("Vel_Estimate", 32, 32, is_float=True, receivers=['Master'], unit='rev/s')
    encoderEstimates = can.Message(
        0x009, "Get_Encoder_Estimates", 8, [encoderPosEstimate, encoderVelEstimate], senders=[newNode.name], send_type='cyclic', cycle_time=10
    )

    # 0x00A - Get Encoder Count
    encoderShadowCount = can.Signal("Shadow_Count", 0, 32, receivers=['Master'], unit='counts')
    encoderCountInCPR = can.Signal("Count_in_CPR", 32, 32, receivers=['Master'], unit='counts')
    encoderCountMsg = can.Message(
        0x00A, "Get_Encoder_Count", 8, [encoderShadowCount, encoderCountInCPR], senders=[newNode.name]
    )

    # 0x00B - Set Controller Modes
    controlMode = can.Signal("Control_Mode", 0, 32, receivers=[newNode.name], choices={state.value: state.name for state in ControlMode})
    inputMode = can.Signal("Input_Mode", 32, 32, receivers=[newNode.name], choices={state.value: state.name for state in InputMode})
    setControllerModeMsg = can.Message(
        0x00B, "Set_Controller_Mode", 8, [controlMode, inputMode], senders=['Master']
    )

    # 0x00C - Set Input Pos
    inputPos = can.Signal("Input_Pos", 0, 32, is_float=True, receivers=[newNode.name], unit='rev')
    velFF = can.Signal("Vel_FF", 32, 16, is_signed=True, scale=0.001, receivers=[newNode.name], unit='rev/s')
    torqueFF = can.Signal("Torque_FF", 48, 16, is_signed=True, scale=0.001, receivers=[newNode.name], unit='Nm')
    setInputPosMsg = can.Message(
        0x00C, "Set_Input_Pos", 8, [inputPos, velFF, torqueFF], senders=['Master']
    )

    # 0x00D - Set Input Vel
    inputVel = can.Signal("Input_Vel", 0, 32, is_float=True, receivers=[newNode.name], unit='rev')
    inputTorqueFF = can.Signal("Input_Torque_FF", 32, 32, is_float=True, receivers=[newNode.name], unit='rev/s')
    setInputVelMsg = can.Message(
        0x00D, "Set_Input_Vel", 8, [inputVel, inputTorqueFF], senders=['Master']
    )

    # 0x00E - Set Input Torque
    inputTorque = can.Signal("Input_Torque", 0, 32, is_float=True, receivers=[newNode.name], unit='Nm')
    setInputTqMsg = can.Message(
        0x00E, "Set_Input_Torque", 8, [inputTorque], senders=['Master']
    )

    # 0x00F - Set Velocity Limit
    velLimit = can.Signal("Velocity_Limit", 0, 32, is_float=True, receivers=[newNode.name], unit='rev/s')
    currentLimit = can.Signal("Current_Limit", 32, 32, is_float=True, receivers=[newNode.name], unit='A')
    setVelLimMsg = can.Message(
        0x00F, "Set_Limits", 8, [velLimit, currentLimit], senders=['Master']
    )

    # 0x010 - Start Anticogging
    startAnticoggingMsg = can.Message(0x010, "Start_Anticogging", 0, [], senders=['Master'])

    # 0x011 - Set Traj Vel Limit
    trajVelLim = can.Signal("Traj_Vel_Limit", 0, 32, is_float=True, receivers=[newNode.name], unit='rev/s')
    setTrajVelMsg = can.Message(
        0x011, "Set_Traj_Vel_Limit", 8, [trajVelLim], senders=['Master']
    )

    # 0x012 - Set Traj Accel Limits
    trajAccelLim = can.Signal("Traj_Accel_Limit", 0, 32, is_float=True, receivers=[newNode.name], unit='rev/s^2')
    trajDecelLim = can.Signal("Traj_Decel_Limit", 32, 32, is_float=True, receivers=[newNode.name], unit='rev/s^2')
    setTrajAccelMsg = can.Message(
        0x012, "Set_Traj_Accel_Limits", 8, [trajAccelLim, trajDecelLim], senders=['Master']
    )

    # 0x013 - Set Traj Inertia
    trajInertia = can.Signal("Traj_Inertia", 0, 32, is_float=True, receivers=[newNode.name], unit='Nm / (rev/s^2)')
    trajInertiaMsg = can.Message(
        0x013, "Set_Traj_Inertia", 8, [trajInertia], senders=['Master']
    )

    # 0x014 - Get Iq
    iqSetpoint = can.Signal("Iq_Setpoint", 0, 32, is_float=True, receivers=['Master'], unit='A')
    iqMeasured = can.Signal("Iq_Measured", 32, 32, is_float=True, receivers=['Master'], unit='A')
    getIqMsg = can.Message(0x014, "Get_Iq", 8, [iqSetpoint, iqMeasured], senders=[newNode.name])

    # 0x015 - Get Sensorless Estimates
    sensorlessPosEstimate = can.Signal("Sensorless_Pos_Estimate", 0, 32, is_float=True, receivers=['Master'], unit='rev')
    sensorlessVelEstimate = can.Signal("Sensorless_Vel_Estimate", 32, 32, is_float=True, receivers=['Master'], unit='rev/s')
    getSensorlessEstMsg = can.Message(0x015, "Get_Sensorless_Estimates", 8, [sensorlessPosEstimate, sensorlessVelEstimate], senders=[newNode.name])

    # 0x016 - Reboot ODrive
    rebootMsg = can.Message(0x016, "Reboot", 0, [], senders=['Master'])

    # 0x017 - Get vbus Voltage and Current
    busVoltage = can.Signal("Bus_Voltage", 0, 32, is_float=True, receivers=['Master'], unit='V')
    busCurrent = can.Signal("Bus_Current", 32, 32, is_float=True, receivers=['Master'], unit='A')
    getVbusVCMsg = can.Message(0x017, "Get_Bus_Voltage_Current", 8, [busVoltage, busCurrent], senders=[newNode.name])

    # 0x018 - Clear Errors
    clearErrorsMsg = can.Message(0x018, "Clear_Errors", 0, [], senders=['Master'])

    # 0x019 - Set Linear Count
    position = can.Signal("Position", 0, 32, is_signed=True, receivers=[newNode.name], unit='counts')
    setLinearCountMsg = can.Message(0x019, "Set_Linear_Count", 8, [position], senders=['Master'])

    # 0x01A - Set Pos gain
    posGain = can.Signal("Pos_Gain", 0, 32, is_float=True, receivers=[newNode.name], unit='(rev/s) / rev')
    setPosGainMsg = can.Message(0x01A, "Set_Pos_Gain", 8, [posGain], senders=['Master'])

    # 0x01B - Set Vel Gains
    velGain = can.Signal("Vel_Gain", 0, 32, is_float=True, receivers=[newNode.name], unit='Nm / (rev/s)')
    velIntGain = can.Signal("Vel_Integrator_Gain", 32, 32, is_float=True, receivers=[newNode.name], unit='(Nm / (rev/s)) / s')
    setVelGainsMsg = can.Message(0x01B, "Set_Vel_Gains", 8, [velGain, velIntGain], senders=['Master'])

    # 0x01C - Get ADC Voltage
    adcVoltage = can.Signal("ADC_Voltage", 0, 32, is_float=True, receivers=['Master'], unit='V')
    getADCVoltageMsg = can.Message(0x01C, "Get_ADC_Voltage", 8, [adcVoltage], senders=[newNode.name])

    # 0x01D - Controller Error
    controllerError = can.Signal("Controller_Error", 0, 32, receivers=['Master'], choices={error.value: error.name for error in ControllerError})
    controllerErrorMsg = can.Message(
        0x01D, "Get_Controller_Error", 8, [controllerError], senders=[newNode.name]
    )

    axisMsgs = [
        heartbeatMsg,
        motorErrorMsg,
        encoderErrorMsg,
        sensorlessErrorMsg,
        axisNodeMsg,
        setAxisState,
        encoderEstimates,
        encoderCountMsg,
        setControllerModeMsg,
        setInputPosMsg,
        setInputVelMsg,
        setInputTqMsg,
        setVelLimMsg,
        startAnticoggingMsg,
        setTrajVelMsg,
        setTrajAccelMsg,
        trajInertiaMsg,
        getIqMsg,
        getSensorlessEstMsg,
        rebootMsg,
        getVbusVCMsg,
        clearErrorsMsg,
        setLinearCountMsg,
        setPosGainMsg,
        setVelGainsMsg,
        getADCVoltageMsg,
        controllerErrorMsg,
    ]

    masterMsgs = [
        estopMsg,
    ]

    # Prepend Axis ID to each message name
    for msg in axisMsgs:
        msg.name = f"Axis{axisID}_{msg.name}"
        msg.frame_id |= (axisID << 5)
        # for signal in msg.signals:
        #     signal.name = f"{signal.name}_Axis{axisID}"
        #     signal.name = f"Axis{axisID}_{signal.name}"

    msgList.append(axisMsgs)
    

from itertools import chain
msgList = list(chain.from_iterable(msgList))

db = can.Database(msgList, nodes, buses, version='0.5.6')

dump_file(db, "odrive-cansimple.dbc")
db = load_file("odrive-cansimple.dbc")
# print(db)
