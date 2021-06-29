import cantools

# 0x00 - NMT Message (Reserved)

# 0x001 - Heartbeat
axisError = cantools.database.can.Signal("Axis_Error", 0, 32)
axisState = cantools.database.can.Signal("Axis_State", 32, 32)
heartbeatMsg = cantools.database.can.Message(
    0x001, "Heartbeat", 8, [axisError, axisState]
)

# 0x002 - E-Stop Message
estopMsg = cantools.database.can.Message(0x002, "Estop", 0, [])

# 0x003 - Motor Error
motorError = cantools.database.can.Signal("Motor_Error", 0, 32)
motorErrorMsg = cantools.database.can.Message(0x003, "Get_Motor_Error", 8, [motorError])

# 0x004 - Encoder Error
encoderError = cantools.database.can.Signal("Encoder_Error", 0, 32)
encoderErrorMsg = cantools.database.can.Message(
    0x004, "Get_Encoder_Error", 8, [encoderError]
)

# 0x005 - Sensorless Error
sensorlessError = cantools.database.can.Signal("Sensorless_Error", 0, 32)
sensorlessErrorMsg = cantools.database.can.Message(
    0x005, "Get_Sensorless_Error", 8, [sensorlessError]
)

# 0x006 - Axis Node ID
axisNodeID = cantools.database.can.Signal("Axis_Node_ID", 0, 32)
axisNodeMsg = cantools.database.can.Message(0x006, "Set_Axis_Node_ID", 8, [axisNodeID])

# 0x007 - Requested State
axisRequestedState = cantools.database.can.Signal("Axis_Requested_State", 0, 32)
setAxisState = cantools.database.can.Message(
    0x007, "Set_Axis_State", 8, [axisRequestedState]
)

# 0x008 - Startup Config (Reserved)

# 0x009 - Encoder Estimates
encoderPosEstimate = cantools.database.can.Signal("Pos_Estimate", 0, 32, is_float=True)
encoderVelEstimate = cantools.database.can.Signal("Vel_Estimate", 32, 32, is_float=True)
encoderEstimates = cantools.database.can.Message(
    0x009, "Get_Encoder_Estimates", 8, [encoderPosEstimate, encoderVelEstimate]
)


# 0x00A - Get Encoder Count
encoderShadowCount = cantools.database.can.Signal("Shadow_Count", 0, 32)
encoderCountInCPR = cantools.database.can.Signal("Count_in_CPR", 32, 32)
encoderCountMsg = cantools.database.can.Message(
    0x00A, "Get_Encoder_Count", 8, [encoderShadowCount, encoderCountInCPR]
)

# 0x00B - Set Controller Modes
controlMode = cantools.database.can.Signal("Control_Mode", 0, 32)
inputMode = cantools.database.can.Signal("Input_Mode", 32, 32)
setControllerModeMsg = cantools.database.can.Message(
    0x00B, "Set_Controller_Mode", 8, [controlMode, inputMode]
)

# 0x00C - Set Input Pos
inputPos = cantools.database.can.Signal("Input_Pos", 0, 32, is_float=True)
velFF = cantools.database.can.Signal("Vel_FF", 32, 16, is_signed=True, scale=0.001)
torqueFF = cantools.database.can.Signal(
    "Torque_FF", 48, 16, is_signed=True, scale=0.001
)
setInputPosMsg = cantools.database.can.Message(
    0x00C, "Set_Input_Pos", 8, [inputPos, velFF, torqueFF]
)

# 0x00D - Set Input Vel
inputVel = cantools.database.can.Signal("Input_Vel", 0, 32, is_float=True)
inputTorqueFF = cantools.database.can.Signal("Input_Torque_FF", 32, 32, is_float=True)
setInputVelMsg = cantools.database.can.Message(
    0x00D, "Set_Input_Vel", 8, [inputVel, inputTorqueFF]
)

# 0x00E - Set Input Torque
inputTorque = cantools.database.can.Signal("Input_Torque", 0, 32, is_float=True)
setInputTqMsg = cantools.database.can.Message(
    0x00E, "Set_Input_Torque", 8, [inputTorque]
)

# 0x00F - Set Velocity Limit
velLimit = cantools.database.can.Signal("Velocity_Limit", 0, 32, is_float=True)
currentLimit = cantools.database.can.Signal("Current_Limit", 32, 32, is_float=True)
setVelLimMsg = cantools.database.can.Message(
    0x00F, "Set_Limits", 8, [velLimit, currentLimit]
)

# 0x010 - Start Anticogging
startAnticoggingMsg = cantools.database.can.Message(0x010, "Start_Anticogging", 0, [])

# 0x011 - Set Traj Vel Limit
trajVelLim = cantools.database.can.Signal("Traj_Vel_Limit", 0, 32, is_float=True)
setTrajVelMsg = cantools.database.can.Message(
    0x011, "Set_Traj_Vel_Limit", 8, [trajVelLim]
)

# 0x012 - Set Traj Accel Limits
trajAccelLim = cantools.database.can.Signal("Traj_Accel_Limit", 0, 32, is_float=True)
trajDecelLim = cantools.database.can.Signal("Traj_Decel_Limit", 32, 32, is_float=True)
setTrajAccelMsg = cantools.database.can.Message(
    0x012, "Set_Traj_Accel_Limits", 8, [trajAccelLim, trajDecelLim]
)

# 0x013 - Set Traj Inertia
trajInertia = cantools.database.can.Signal("Traj_Inertia", 0, 32, is_float=True)
trajInertiaMsg = cantools.database.can.Message(
    0x013, "Set_Traj_Inertia", 8, [trajInertia]
)

# 0x014 - Get Iq
iqSetpoint = cantools.database.can.Signal("Iq_Setpoint", 0, 32, is_float=True)
iqMeasured = cantools.database.can.Signal("Iq_Measured", 32, 32, is_float=True)
getIqMsg = cantools.database.can.Message(0x014, "Get_Iq", 8, [iqSetpoint, iqMeasured])

# 0x015 - Get Sensorless Estimates
sensorlessPosEstimate = cantools.database.can.Signal(
    "Sensorless_Pos_Estimate", 0, 32, is_float=True
)
sensorlessVelEstimate = cantools.database.can.Signal(
    "Sensorless_Vel_Estimate", 32, 32, is_float=True
)
getSensorlessEstMsg = cantools.database.can.Message(
    0x015, "Get_Sensorless_Estimates", 8, [sensorlessPosEstimate, sensorlessVelEstimate]
)

# 0x016 - Reboot ODrive
rebootMsg = cantools.database.can.Message(0x016, "Reboot", 0, [])

# 0x017 - Get vbus Voltage
vbusVoltage = cantools.database.can.Signal("Vbus_Voltage", 0, 32, is_float=True)
getVbusVMsg = cantools.database.can.Message(0x017, "Get_Vbus_Voltage", 8, [vbusVoltage])

# 0x018 - Clear Errors
clearErrorsMsg = cantools.database.can.Message(0x018, "Clear_Errors", 0, [])

# 0x019 - Set Linear Count
position = cantools.database.can.Signal("Position", 0, 32, is_signed=True)
setLinearCountMsg = cantools.database.can.Message(0x019, "Set_Linear_Count", 8, [position])

db = cantools.database.can.Database(
    [
        heartbeatMsg,
        estopMsg,
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
        getVbusVMsg,
        clearErrorsMsg,
        setLinearCountMsg,
    ]
)

cantools.database.dump_file(db, "odrive-cansimple.dbc")
db = cantools.database.load_file("odrive-cansimple.dbc")
print(db)
