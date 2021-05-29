from __future__ import print_function

import sys
import time
import threading
import platform
import subprocess
import os
from fibre.utils import Event
import odrive.enums
from odrive.enums import *

try:
    if platform.system() == 'Windows':
        import win32console
        import colorama
        colorama.init()
except ImportError:
    print("Could not init terminal features.")
    print("Refer to install instructions at http://docs.odriverobotics.com/#downloading-and-installing-tools")
    sys.stdout.flush()
    pass

if sys.version_info < (3, 0):
    input = raw_input

_VT100Colors = {
    'green': '\x1b[92;1m',
    'cyan': '\x1b[96;1m',
    'yellow': '\x1b[93;1m',
    'red': '\x1b[91;1m',
    'default': '\x1b[0m'
}

async def get_serial_number_str(device):
    if hasattr(device, '_serial_number_property'):
        return format(await device._serial_number_property.read(), 'x').upper()
    else:
        return "[unknown serial number]"

def get_serial_number_str_sync(device):
    if hasattr(device, '_serial_number_property'):
        return format(device._serial_number_property.read(), 'x').upper()
    else:
        return "[unknown serial number]"

def calculate_thermistor_coeffs(degree, Rload, R_25, Beta, Tmin, Tmax, thermistor_bottom = False, plot = False):
    import numpy as np
    T_25 = 25 + 273.15 #Kelvin
    temps = np.linspace(Tmin, Tmax, 1000)
    tempsK = temps + 273.15

    # https://en.wikipedia.org/wiki/Thermistor#B_or_%CE%B2_parameter_equation
    r_inf = R_25 * np.exp(-Beta/T_25)
    R_temps = r_inf * np.exp(Beta/tempsK)
    if thermistor_bottom:
        V = R_temps / (Rload + R_temps)
    else:
        V = Rload / (Rload + R_temps)

    fit = np.polyfit(V, temps, degree)
    p1 = np.poly1d(fit)
    fit_temps = p1(V)

    if plot:
        import matplotlib.pyplot as plt
        print(fit)
        plt.plot(V, temps, label='actual')
        plt.plot(V, fit_temps, label='fit')
        plt.xlabel('normalized voltage')
        plt.ylabel('Temp [C]')
        plt.legend(loc=0)
        plt.show()

    return p1

class OperationAbortedException(Exception):
    pass

def set_motor_thermistor_coeffs(axis, Rload, R_25, Beta, Tmin, Tmax, thermistor_bottom = False):
    coeffs = calculate_thermistor_coeffs(3, Rload, R_25, Beta, Tmin, Tmax, thermistor_bottom)
    axis.motor.motor_thermistor.config.poly_coefficient_0 = float(coeffs[3])
    axis.motor.motor_thermistor.config.poly_coefficient_1 = float(coeffs[2])
    axis.motor.motor_thermistor.config.poly_coefficient_2 = float(coeffs[1])
    axis.motor.motor_thermistor.config.poly_coefficient_3 = float(coeffs[0])

def dump_errors(odrv, clear=False, printfunc = print):
    axes = [(name, getattr(odrv, name)) for name in dir(odrv) if name.startswith('axis')]
    axes.sort()

    def dump_errors_for_module(indent, name, obj, path, errorcodes):
        prefix = indent + name.strip('0123456789') + ": "
        for elem in path.split('.'):
            if not hasattr(obj, elem):
                printfunc(prefix + _VT100Colors['yellow'] + "not found" + _VT100Colors['default'])
                return
            parent = obj
            obj = getattr(obj, elem)
        if obj != 0:
            printfunc(indent + name + ": " + _VT100Colors['red'] + "Error(s):" + _VT100Colors['default'])
            for bit in range(64):
                if obj & (1 << bit) != 0:
                    printfunc(indent + "  " + errorcodes.get((1 << bit), 'UNKNOWN ERROR: 0x{:08X}'.format(1 << bit)))
        else:
            printfunc(indent + name + ": " + _VT100Colors['green'] + "no error" + _VT100Colors['default'])

    system_error_codes = {v: k for k, v in odrive.enums.__dict__ .items() if k.startswith("ODRIVE_ERROR_")}
    dump_errors_for_module("", "system", odrv, 'error', system_error_codes)

    for name, axis in axes:
        printfunc(name)

        # Flatten axis and submodules
        # (name, obj, path, errorcode)
        module_decode_map = [
            ('axis', axis, 'error', {v: k for k, v in odrive.enums.__dict__ .items() if k.startswith("AXIS_ERROR_")}),
            ('motor', axis, 'motor.error', {v: k for k, v in odrive.enums.__dict__ .items() if k.startswith("MOTOR_ERROR_")}),
            ('sensorless_estimator', axis, 'sensorless_estimator.error', {v: k for k, v in odrive.enums.__dict__ .items() if k.startswith("SENSORLESS_ESTIMATOR_ERROR")}),
            ('encoder', axis, 'encoder.error', {v: k for k, v in odrive.enums.__dict__ .items() if k.startswith("ENCODER_ERROR_")}),
            ('controller', axis, 'controller.error', {v: k for k, v in odrive.enums.__dict__ .items() if k.startswith("CONTROLLER_ERROR_")}),
        ]

        for name, obj, path, errorcodes in module_decode_map:
            dump_errors_for_module("  ", name, obj, path, errorcodes)

    if clear:
        odrv.clear_errors()

def oscilloscope_dump(odrv, num_vals, filename='oscilloscope.csv'):
    with open(filename, 'w') as f:
        for x in range(num_vals):
            f.write(str(odrv.oscilloscope.get_val(x)))
            f.write('\n')

data_rate = 200
plot_rate = 10
num_samples = 500
def start_liveplotter(get_var_callback):
    """
    Starts a liveplotter.
    The variable that is plotted is retrieved from get_var_callback.
    This function returns immediately and the liveplotter quits when
    the user closes it.
    """

    import matplotlib.pyplot as plt

    cancellation_token = Event()

    global vals
    vals = []
    def fetch_data():
        global vals
        while not cancellation_token.is_set():
            try:
                data = get_var_callback()
            except Exception as ex:
                print(str(ex))
                time.sleep(1)
                continue
            vals.append(data)
            if len(vals) > num_samples:
                vals = vals[-num_samples:]
            time.sleep(1/data_rate)

    # TODO: use animation for better UI performance, see:
    # https://matplotlib.org/examples/animation/simple_anim.html
    def plot_data():
        global vals

        plt.ion()

        # Make sure the script terminates when the user closes the plotter
        def closed(evt):
            cancellation_token.set()
        fig = plt.figure()
        fig.canvas.mpl_connect('close_event', closed)

        while not cancellation_token.is_set():
            plt.clf()
            plt.plot(vals)
            plt.legend(list(range(len(vals))))
            fig.canvas.draw()
            fig.canvas.start_event_loop(1/plot_rate)

    fetch_t = threading.Thread(target=fetch_data)
    fetch_t.daemon = True
    fetch_t.start()
    
    plot_t = threading.Thread(target=plot_data)
    plot_t.daemon = True
    plot_t.start()

    return cancellation_token;
    #plot_data()


class BulkCapture:
    '''
    Asynchronously captures a bulk set of data when instance is created.

    get_var_callback: a function that returns the data you want to collect (see the example below)
    data_rate: Rate in hz
    length: Length of time to capture in seconds

    Example Usage:
        capture = BulkCapture(lambda :[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])
        # Do stuff while capturing (like sending position commands)
        capture.event.wait() # When you're done doing stuff, wait for the capture to be completed.
        print(capture.data) # Do stuff with the data
        capture.plot_data() # Helper method to plot the data
    '''

    def __init__(self,
                 get_var_callback,
                 data_rate=500.0,
                 duration=2.0):
        from threading import Event, Thread
        import numpy as np

        self.get_var_callback = get_var_callback
        self.event = Event()
        def loop():
            vals = []
            start_time = time.monotonic()
            period = 1.0 / data_rate
            while time.monotonic() - start_time < duration:
                try:
                    data = get_var_callback()
                except Exception as ex:
                    print(str(ex))
                    print("Waiting 1 second before next data point")
                    time.sleep(1)
                    continue
                relative_time = time.monotonic() - start_time
                vals.append([relative_time] + data)
                time.sleep(period - (relative_time % period)) # this ensures consistently timed samples
            self.data = np.array(vals) # A lock is not really necessary due to the event
            print("Capture complete")
            achieved_data_rate = len(self.data) / self.data[-1, 0]
            if achieved_data_rate < (data_rate * 0.9):
                print("Achieved average data rate: {}Hz".format(achieved_data_rate))
                print("If this rate is significantly lower than what you specified, consider lowering it below the achieved value for more consistent sampling.")
            self.event.set() # tell the main thread that the bulk capture is complete
        Thread(target=loop, daemon=True).start()
    
    def plot(self):
        import matplotlib.pyplot as plt
        import inspect
        from textwrap import wrap
        plt.plot(self.data[:,0], self.data[:,1:])
        plt.xlabel("Time (seconds)")
        title = (str(inspect.getsource(self.get_var_callback))
                .strip("['\\n']")
                .split(" = ")[1])
        plt.title("\n".join(wrap(title, 60)))
        plt.legend(range(self.data.shape[1]-1))
        plt.show()


def step_and_plot(  axis,
                    step_size=100.0,
                    settle_time=0.5,
                    data_rate=500.0,
                    ctrl_mode=CONTROL_MODE_POSITION_CONTROL):
    
    if ctrl_mode is CONTROL_MODE_POSITION_CONTROL:
        get_var_callback = lambda :[axis.encoder.pos_estimate, axis.controller.pos_setpoint]
        initial_setpoint = axis.encoder.pos_estimate
        def set_setpoint(setpoint):
            axis.controller.pos_setpoint = setpoint
    elif ctrl_mode is CONTROL_MODE_VELOCITY_CONTROL:
        get_var_callback = lambda :[axis.encoder.vel_estimate, axis.controller.vel_setpoint]
        initial_setpoint = 0
        def set_setpoint(setpoint):
            axis.controller.vel_setpoint = setpoint
    else:
        print("Invalid control mode")
        return
    
    initial_settle_time = 0.5
    initial_control_mode = axis.controller.config.control_mode # Set it back afterwards
    print(initial_control_mode)
    axis.controller.config.control_mode = ctrl_mode
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
    capture = BulkCapture(get_var_callback,
                          data_rate=data_rate,
                          duration=initial_settle_time + settle_time)

    set_setpoint(initial_setpoint)
    time.sleep(initial_settle_time)
    set_setpoint(initial_setpoint + step_size) # relative/incremental movement

    capture.event.wait() # wait for Bulk Capture to be complete

    axis.requested_state = AXIS_STATE_IDLE
    axis.controller.config.control_mode = initial_control_mode
    capture.plot()


def print_drv_regs(name, motor):
    """
    Dumps the current gate driver regisers for the specified motor
    """
    fault = motor.gate_driver.drv_fault
    status_reg_1 = motor.gate_driver.status_reg_1
    status_reg_2 = motor.gate_driver.status_reg_2
    ctrl_reg_1 = motor.gate_driver.ctrl_reg_1
    ctrl_reg_2 = motor.gate_driver.ctrl_reg_2
    print(name + ": " + str(fault))
    print("DRV Fault Code: " + str(fault))
    print("Status Reg 1: " + str(status_reg_1) + " (" + format(status_reg_1, '#010b') + ")")
    print("Status Reg 2: " + str(status_reg_2) + " (" + format(status_reg_2, '#010b') + ")")
    print("Control Reg 1: " + str(ctrl_reg_1) + " (" + format(ctrl_reg_1, '#013b') + ")")
    print("Control Reg 2: " + str(ctrl_reg_2) + " (" + format(ctrl_reg_2, '#09b') + ")")

def show_oscilloscope(odrv):
    size = 18000
    values = []
    for i in range(size):
        values.append(odrv.oscilloscope.get_val(i))

    import matplotlib.pyplot as plt
    plt.plot(values)
    plt.show()

def rate_test(device):
    """
    Tests how many integers per second can be transmitted
    """

    # import matplotlib.pyplot as plt
    # plt.ion()

    print("reading 10000 values...")
    numFrames = 10000
    vals = []
    for _ in range(numFrames):
        vals.append(device.n_evt_control_loop)

    loopsPerFrame = (vals[-1] - vals[0])/numFrames
    loopsPerSec = (168000000/(6*3500))
    FramePerSec = loopsPerSec/loopsPerFrame
    print("Frames per second: " + str(FramePerSec))

    # plt.plot(vals)
    # plt.show(block=True)

def usb_burn_in_test(get_var_callback, cancellation_token):
    """
    Starts background threads that read a values form the USB device in a spin-loop
    """

    def fetch_data():
        global vals
        i = 0
        while not cancellation_token.is_set():
            try:
                get_var_callback()
                i += 1
            except Exception as ex:
                print(str(ex))
                time.sleep(1)
                i = 0
                continue
            if i % 1000 == 0:
                print("read {} values".format(i))
    threading.Thread(target=fetch_data, daemon=True).start()

def yes_no_prompt(question, default=None):
    if default is None:
        question += " [y/n] "
    elif default == True:
        question += " [Y/n] "
    elif default == False:
        question += " [y/N] "

    while True:
        print(question, end='')

        choice = input().lower()
        if choice in {'yes', 'y'}:
            return True
        elif choice in {'no', 'n'}:
            return False
        elif choice == '' and default is not None:
            return default

def dump_interrupts(odrv):
    interrupts = [
        (-12, "MemoryManagement_IRQn"),
        (-11, "BusFault_IRQn"),
        (-10, "UsageFault_IRQn"),
        (-5, "SVCall_IRQn"),
        (-4, "DebugMonitor_IRQn"),
        (-2, "PendSV_IRQn"),
        (-1, "SysTick_IRQn"),
        (0, "WWDG_IRQn"),
        (1, "PVD_IRQn"),
        (2, "TAMP_STAMP_IRQn"),
        (3, "RTC_WKUP_IRQn"),
        (4, "FLASH_IRQn"),
        (5, "RCC_IRQn"),
        (6, "EXTI0_IRQn"),
        (7, "EXTI1_IRQn"),
        (8, "EXTI2_IRQn"),
        (9, "EXTI3_IRQn"),
        (10, "EXTI4_IRQn"),
        (11, "DMA1_Stream0_IRQn"),
        (12, "DMA1_Stream1_IRQn"),
        (13, "DMA1_Stream2_IRQn"),
        (14, "DMA1_Stream3_IRQn"),
        (15, "DMA1_Stream4_IRQn"),
        (16, "DMA1_Stream5_IRQn"),
        (17, "DMA1_Stream6_IRQn"),
        (18, "ADC_IRQn"),
        (19, "CAN1_TX_IRQn"),
        (20, "CAN1_RX0_IRQn"),
        (21, "CAN1_RX1_IRQn"),
        (22, "CAN1_SCE_IRQn"),
        (23, "EXTI9_5_IRQn"),
        (24, "TIM1_BRK_TIM9_IRQn"),
        (25, "TIM1_UP_TIM10_IRQn"),
        (26, "TIM1_TRG_COM_TIM11_IRQn"),
        (27, "TIM1_CC_IRQn"),
        (28, "TIM2_IRQn"),
        (29, "TIM3_IRQn"),
        (30, "TIM4_IRQn"),
        (31, "I2C1_EV_IRQn"),
        (32, "I2C1_ER_IRQn"),
        (33, "I2C2_EV_IRQn"),
        (34, "I2C2_ER_IRQn"),
        (35, "SPI1_IRQn"),
        (36, "SPI2_IRQn"),
        (37, "USART1_IRQn"),
        (38, "USART2_IRQn"),
        (39, "USART3_IRQn"),
        (40, "EXTI15_10_IRQn"),
        (41, "RTC_Alarm_IRQn"),
        (42, "OTG_FS_WKUP_IRQn"),
        (43, "TIM8_BRK_TIM12_IRQn"),
        (44, "TIM8_UP_TIM13_IRQn"),
        (45, "TIM8_TRG_COM_TIM14_IRQn"),
        (46, "TIM8_CC_IRQn"),
        (47, "DMA1_Stream7_IRQn"),
        (48, "FMC_IRQn"),
        (49, "SDMMC1_IRQn"),
        (50, "TIM5_IRQn"),
        (51, "SPI3_IRQn"),
        (52, "UART4_IRQn"),
        (53, "UART5_IRQn"),
        (54, "TIM6_DAC_IRQn"),
        (55, "TIM7_IRQn"),
        (56, "DMA2_Stream0_IRQn"),
        (57, "DMA2_Stream1_IRQn"),
        (58, "DMA2_Stream2_IRQn"),
        (59, "DMA2_Stream3_IRQn"),
        (60, "DMA2_Stream4_IRQn"),
        (61, "ETH_IRQn"),
        (62, "ETH_WKUP_IRQn"),
        (63, "CAN2_TX_IRQn"),
        (64, "CAN2_RX0_IRQn"),
        (65, "CAN2_RX1_IRQn"),
        (66, "CAN2_SCE_IRQn"),
        (67, "OTG_FS_IRQn"),
        (68, "DMA2_Stream5_IRQn"),
        (69, "DMA2_Stream6_IRQn"),
        (70, "DMA2_Stream7_IRQn"),
        (71, "USART6_IRQn"),
        (72, "I2C3_EV_IRQn"),
        (73, "I2C3_ER_IRQn"),
        (74, "OTG_HS_EP1_OUT_IRQn"),
        (75, "OTG_HS_EP1_IN_IRQn"),
        (76, "OTG_HS_WKUP_IRQn"),
        (77, "OTG_HS_IRQn"),
        # gap
        (80, "RNG_IRQn"),
        (81, "FPU_IRQn"),
        (82, "UART7_IRQn"),
        (83, "UART8_IRQn"),
        (84, "SPI4_IRQn"),
        (85, "SPI5_IRQn"),
        # gap
        (87, "SAI1_IRQn"),
        # gap
        (91, "SAI2_IRQn"),
        (92, "QUADSPI_IRQn"),
        (93, "LPTIM1_IRQn"),
        # gap
        (103, "SDMMC2_IRQn")
    ]

    print("|   # | Name                    | Prio | En |   Count |")
    print("|-----|-------------------------|------|----|---------|")
    for irqn, irq_name in interrupts:
        status = odrv.get_interrupt_status(irqn)
        if (status != 0):
            print("| {} | {} | {} | {} | {} |".format(
                    str(irqn).rjust(3),
                    irq_name.ljust(23),
                    str(status & 0xff).rjust(4),
                    " *" if (status & 0x80000000) else "  ",
                    str((status >> 8) & 0x7fffff).rjust(7)))

def dump_threads(odrv):
    prefixes = ["max_stack_usage_", "stack_size_", "prio_"]
    keys = [k[len(prefix):] for k in dir(odrv.system_stats) for prefix in prefixes if k.startswith(prefix)]
    good_keys = set([k for k in set(keys) if keys.count(k) == len(prefixes)])
    if len(good_keys) > len(set(keys)):
        print("Warning: incomplete thread information for threads {}".format(set(keys) - good_keys))

    print("| Name    | Stack Size [B] | Max Ever Stack Usage [B] | Prio |")
    print("|---------|----------------|--------------------------|------|")
    for k in sorted(good_keys):
        sz = getattr(odrv.system_stats, "stack_size_" + k)
        use = getattr(odrv.system_stats, "max_stack_usage_" + k)
        print("| {} | {} | {} | {} |".format(
            k.ljust(7),
            str(sz).rjust(14),
            "{} ({:.1f}%)".format(use, use / sz * 100).rjust(24),
            str(getattr(odrv.system_stats, "prio_" + k)).rjust(4)
        ))


def dump_dma(odrv):
    if odrv.hw_version_major == 3:
        dma_functions = [[
            # https://www.st.com/content/ccc/resource/technical/document/reference_manual/3d/6d/5a/66/b4/99/40/d4/DM00031020.pdf/files/DM00031020.pdf/jcr:content/translations/en.DM00031020.pdf Table 42
            ["SPI3_RX",          "-",                  "SPI3_RX",           "SPI2_RX",            "SPI2_TX",            "SPI3_TX",     "-",                  "SPI3_TX"],
            ["I2C1_RX",          "-",                  "TIM7_UP",           "-",                  "TIM7_UP",            "I2C1_RX",     "I2C1_TX",            "I2C1_TX"],
            ["TIM4_CH1",         "-",                  "I2S3_EXT_RX",       "TIM4_CH2",           "I2S2_EXT_TX",        "I2S3_EXT_TX", "TIM4_UP",            "TIM4_CH3"],
            ["I2S3_EXT_RX",      "TIM2_UP/TIM2_CH3",   "I2C3_RX",           "I2S2_EXT_RX",        "I2C3_TX",            "TIM2_CH1",    "TIM2_CH2/TIM2_CH4",  "TIM2_UP/TIM2_CH4"],
            ["UART5_RX",         "USART3_RX",          "UART4_RX",          "USART3_TX",          "UART4_TX",           "USART2_RX",   "USART2_TX",          "UART5_TX"],
            ["UART8_TX",         "UART7_TX",           "TIM3_CH4/TIM3_UP",  "UART7_RX",           "TIM3_CH1/TIM3_TRIG", "TIM3_CH2",    "UART8_RX",           "TIM3_CH3"],
            ["TIM5_CH3/TIM5_UP", "TIM5_CH4/TIM5_TRIG", "TIM5_CH1",          "TIM5_CH4/TIM5_TRIG", "TIM5_CH2",           "-",           "TIM5_UP",            "-"],
            ["-",                "TIM6_UP",            "I2C2_RX",           "I2C2_RX",            "USART3_TX",          "DAC1",        "DAC2",               "I2C2_TX"],
        ], [
            # https://www.st.com/content/ccc/resource/technical/document/reference_manual/3d/6d/5a/66/b4/99/40/d4/DM00031020.pdf/files/DM00031020.pdf/jcr:content/translations/en.DM00031020.pdf Table 43
            ["ADC1",      "SAI1_A",      "TIM8_CH1/TIM8_CH2/TIM8_CH3",    "SAI1_A",      "ADC1",                          "SAI1_B",      "TIM1_CH1/TIM1_CH2/TIM1_CH3",    "-"],
            ["-",         "DCMI",        "ADC2",                          "ADC2",        "SAI1_B",                        "SPI6_TX",     "SPI6_RX",                       "DCMI"],
            ["ADC3",      "ADC3",        "-",                             "SPI5_RX",     "SPI5_TX",                       "CRYP_OUT",    "CRYP_IN",                       "HASH_IN"],
            ["SPI1_RX",   "-",           "SPI1_RX",                       "SPI1_TX",     "-",                             "SPI1_TX",     "-",                             "-"],
            ["SPI4_RX",   "SPI4_TX",     "USART1_RX",                     "SDIO",        "-",                             "USART1_RX",   "SDIO",                          "USART1_TX"],
            ["-",         "USART6_RX",   "USART6_RX",                     "SPI4_RX",     "SPI4_TX",                       "-",           "USART6_TX",                     "USART6_TX"],
            ["TIM1_TRIG", "TIM1_CH1",    "TIM1_CH2",                      "TIM1_CH1",    "TIM1_CH4/TIM1_TRIG/TIM1_COM",   "TIM1_UP",     "TIM1_CH3",                      "-"],
            ["-",         "TIM8_UP",     "TIM8_CH1",                      "TIM8_CH2",    "TIM8_CH3",                      "SPI5_RX",     "SPI5_TX",                       "TIM8_CH4/TIM8_TRIG/TIM8_COM"],
        ]]
    elif odrv.hw_version_major == 4:
        dma_functions = [[
            # https://www.st.com/resource/en/reference_manual/dm00305990-stm32f72xxx-and-stm32f73xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf Table 26
            ["SPI3_RX",          "-",                  "SPI3_RX",           "SPI2_RX",            "SPI2_TX",            "SPI3_TX",     "-",                  "SPI3_TX"],
            ["I2C1_RX",          "I2C3_RX",            "TIM7_UP",           "-",                  "TIM7_UP",            "I2C1_RX",     "I2C1_TX",            "I2C1_TX"],
            ["TIM4_CH1",         "-",                  "-",                 "TIM4_CH2",           "-",                  "-",           "TIM4_UP",            "TIM4_CH3"],
            ["-",                "TIM2_UP/TIM2_CH3",   "I2C3_RX",           "-",                  "I2C3_TX",            "TIM2_CH1",    "TIM2_CH2/TIM2_CH4",  "TIM2_UP/TIM2_CH4"],
            ["UART5_RX",         "USART3_RX",          "UART4_RX",          "USART3_TX",          "UART4_TX",           "USART2_RX",   "USART2_TX",          "UART5_TX"],
            ["UART8_TX",         "UART7_TX",           "TIM3_CH4/TIM3_UP",  "UART7_RX",           "TIM3_CH1/TIM3_TRIG", "TIM3_CH2",    "UART8_RX",           "TIM3_CH3"],
            ["TIM5_CH3/TIM5_UP", "TIM5_CH4/TIM5_TRIG", "TIM5_CH1",          "TIM5_CH4/TIM5_TRIG", "TIM5_CH2",           "-",           "TIM5_UP",            "-"],
            ["-",                "TIM6_UP",            "I2C2_RX",           "I2C2_RX",            "USART3_TX",          "DAC1",        "DAC2",               "I2C2_TX"],
        ], [
            # https://www.st.com/resource/en/reference_manual/dm00305990-stm32f72xxx-and-stm32f73xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf Table 27
            ["ADC1",      "SAI1_A",      "TIM8_CH1/TIM8_CH2/TIM8_CH3",    "SAI1_A",      "ADC1",                          "SAI1_B",      "TIM1_CH1/TIM1_CH2/TIM1_CH3",    "SAI2_B"],
            ["-",         "-",           "ADC2",                          "ADC2",        "SAI1_B",                        "-",           "-",                             "-"],
            ["ADC3",      "ADC3",        "-",                             "SPI5_RX",     "SPI5_TX",                       "AES_OUT",     "AES_IN",                        "-"],
            ["SPI1_RX",   "-",           "SPI1_RX",                       "SPI1_TX",     "SAI2_A",                        "SPI1_TX",     "SAI2_B",                        "QUADSPI"],
            ["SPI4_RX",   "SPI4_TX",     "USART1_RX",                     "SDMMC1",      "-",                             "USART1_RX",   "SDMMC1",                        "USART1_TX"],
            ["-",         "USART6_RX",   "USART6_RX",                     "SPI4_RX",     "SPI4_TX",                       "-",           "USART6_TX",                     "USART6_TX"],
            ["TIM1_TRIG", "TIM1_CH1",    "TIM1_CH2",                      "TIM1_CH1",    "TIM1_CH4/TIM1_TRIG/TIM1_COM",   "TIM1_UP",     "TIM1_CH3",                      "-"],
            ["-",         "TIM8_UP",     "TIM8_CH1",                      "TIM8_CH2",    "TIM8_CH3",                      "SPI5_RX",     "SPI5_TX",                       "TIM8_CH4/TIM8_TRIG/TIM8_COM"],
            None,
            None,
            None,
            ["SDMMC2",    "-",           "-",                             "-",           "-",                             "SDMMC2",      "-",                             "-"],
        ]]

    print("| Name         | Prio | Channel                          | Configured |")
    print("|--------------|------|----------------------------------|------------|")
    for stream_num in range(16):
        status = odrv.get_dma_status(stream_num)
        if (status != 0):
            channel = (status >> 2) & 0x7
            ch_name = dma_functions[stream_num >> 3][channel][stream_num & 0x7]
            print("| DMA{}_Stream{} |    {} | {} {} |          {} |".format(
                     (stream_num >> 3) + 1,
                     (stream_num & 0x7),
                     (status & 0x3),
                     channel,
                     ("(" + ch_name + ")").ljust(30),
                     "*" if (status & 0x80000000) else " "))

def dump_timing(odrv, n_samples=100, path='/tmp/timings.png'):
    import matplotlib.pyplot as plt
    import re
    import numpy as np
    
    timings = []
    
    for attr in dir(odrv.task_times):
        if not attr.startswith('_'):
            timings.append((attr, getattr(odrv.task_times, attr), [], [])) # (name, obj, start_times, lengths)
    for k in dir(odrv):
        if re.match(r'axis[0-9]+', k):
            for attr in dir(getattr(odrv, k).task_times):
                if not attr.startswith('_'):
                    timings.append((k + '.' + attr, getattr(getattr(odrv, k).task_times, attr), [], [])) # (name, obj, start_times, lengths)

    # Take a couple of samples
    print("sampling...")
    for i in range(n_samples):
        odrv.task_timers_armed = True # Trigger sample and wait for it to finish
        while odrv.task_timers_armed: pass
        for name, obj, start_times, lengths in timings:
            start_times.append(obj.start_time)
            lengths.append(obj.length)
    print("done")

    # sort by start time
    timings = sorted(timings, key = lambda x: np.mean(x[2]))

    plt.rcParams['figure.figsize'] = 21, 9
    plt.figure()
    plt.grid(True)
    plt.barh(
        [-i for i in range(len(timings))], # y positions
        [np.mean(lengths) for name, obj, start_times, lengths in timings], # lengths
        left = [np.mean(start_times) for name, obj, start_times, lengths in timings], # starts
        xerr = (
            [np.std(lengths) for name, obj, start_times, lengths in timings], # error bars to the left side
            [(min(obj.max_length, 20100) - np.mean(lengths)) for name, obj, start_times, lengths in timings], # error bars to the right side  - TODO: remove artificial min()
        ),
        tick_label = [name for name, obj, start_times, lengths in timings], # labels
    )
    plt.savefig(path, bbox_inches='tight')
