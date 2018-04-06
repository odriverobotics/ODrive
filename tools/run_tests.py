#!/bin/env python3
#
# This script tests various functions of the ODrive firmware and
# the ODrive Python library.
#
# Usage:
# 1. adapt test-rig.yaml for your test rig.
# 2. ./run_tests.py

import yaml
import os
import sys
import threading
import traceback
from odrive.tests import *
from odrive.utils import Logger, for_all_parallel


all_tests = [
#    TestFlashAndErase(),
#    TestSetup(),
#    TestMotorCalibration(),
#    # TODO: test encoder index search
#    TestEncoderOffsetCalibration(),
#    # TODO: hold down one motor while the other one does an index search (should fail)
#    TestClosedLoopControl(),
#    TestStoreAndReboot(),
#    TestEncoderOffsetCalibration(), # need to find offset _or_ index after reboot
#    TestClosedLoopControl(),
    TestDiscoverAndGotoIdle(), # for testing
    TestEncoderOffsetCalibration(pass_if_ready=True),
    TestVelCtrlVsPosCtrl()
    # TODO: test step/dir
    # TODO: test sensorless
    # TODO: test ASCII protocol
    # TODO: test protocol over UART
]


logger = Logger()

script_path=os.path.dirname(os.path.realpath(__file__))
with open(script_path + '/test-rig.yaml', 'r') as file_stream:
    test_rig_yaml = yaml.load(file_stream)

os.chdir(script_path + '/../Firmware')

# Build a dictionary of odrive test contexts by name
odrives_by_name = {}
for odrv_idx, odrv_yaml in enumerate(test_rig_yaml['odrives']):
    name = odrv_yaml['name'] if 'name' in odrv_yaml else 'odrive{}'.format(odrv_idx)
    odrives_by_name[name] = ODriveTestContext(name, odrv_yaml)

# Build a dictionary of axis test contexts by name (e.g. odrive0.axis0)
axes_by_name = {}
for odrv_ctx in odrives_by_name.values():
    for axis_idx, axis_ctx in enumerate(odrv_ctx.axes):
        axes_by_name[axis_ctx.name] = axis_ctx

# Ensure mechanical couplings are valid
couplings = []
if test_rig_yaml['couplings'] is None:
    test_rig_yaml['couplings'] = {}
else:
    for coupling in test_rig_yaml['couplings']:
        couplings.append([axes_by_name[axis_name] for axis_name in coupling])


try:
    for test in all_tests:
        if isinstance(test, ODriveTest):
            def odrv_test_thread(odrv_name):
                odrv_ctx = odrives_by_name[odrv_name]
                logger.info('● running {} on {}...'.format(type(test).__name__, odrv_name))
                try:
                    test.check_preconditions(odrv_ctx,
                              logger.indent('  {}: '.format(odrv_name)))
                except:
                    raise PreconditionsNotMet()
                test.run_test(odrv_ctx,
                              logger.indent('  {}: '.format(odrv_name)))

            if test._exclusive:
                for odrv in odrives_by_name:
                    odrv_test_thread(odrv)
            else:
                for_all_parallel(odrives_by_name, lambda x: x, odrv_test_thread)

        elif isinstance(test, AxisTest):
            def axis_test_thread(axis_name):
                # Get all axes that are mechanically coupled with the axis specified by axis_name
                conflicting_axes = sum([c for c in couplings if (axis_name in [a.name for a in c])], [])
                # Remove duplicates
                conflicting_axes = list(set(conflicting_axes))
                # Acquire lock for all conflicting axes
                conflicting_axes.sort(key=lambda x: x.name) # prevent deadlocks
                axis_ctx = axes_by_name[axis_name]
                for conflicting_axis in conflicting_axes:
                    conflicting_axis.lock.acquire()
                try:
                    # Run test on this axis
                    logger.info('● running {} on {}...'.format(type(test).__name__, axis_name))
                    try:
                        test.check_preconditions(axis_ctx,
                                    logger.indent('  {}: '.format(axis_name)))
                    except:
                        raise PreconditionsNotMet()
                    test.run_test(axis_ctx,
                                  logger.indent('  {}: '.format(axis_name)))
                finally:
                    # Release all conflicting axes
                    for conflicting_axis in conflicting_axes:
                        conflicting_axis.lock.release()

            for_all_parallel(axes_by_name, lambda x: x, axis_test_thread)

        elif isinstance(test, DualAxisTest):
            def dual_axis_test_thread(coupling):
                coupling_name = "...".join([a.name for a in coupling])
                # Remove duplicates
                coupled_axes = list(set(coupling))
                # Acquire lock for all conflicting axes
                coupled_axes.sort(key=lambda x: x.name) # prevent deadlocks
                for axis_ctx in coupled_axes:
                    axis_ctx.lock.acquire()
                try:
                    # Run test on this axis
                    logger.info('● running {} on {}...'.format(type(test).__name__, coupling_name))
                    try:
                        test.check_preconditions(coupled_axes[0], coupled_axes[1],
                                    logger.indent('  {}: '.format(coupling_name)))
                    except:
                        raise PreconditionsNotMet()
                    test.run_test(coupled_axes[0], coupled_axes[1],
                                  logger.indent('  {}: '.format(coupling_name)))
                finally:
                    # Release all conflicting axes
                    for axis_ctx in coupled_axes:
                        axis_ctx.lock.release()

            for_all_parallel(couplings, lambda x: "..".join([a.name for a in x]), dual_axis_test_thread)

        else:
            logger.warn("ignoring unknown test type {}".format(type(test)))

except:
    logger.error(traceback.format_exc())
    logger.debug('=> Test failed. Please wait while I secure the test rig...')
    try:
        dont_secure_after_failure = True # TODO: disable
        if not dont_secure_after_failure:
            def odrv_reset_thread(odrv_name):
                odrv_ctx = odrives_by_name[odrv_name]
                run("make erase PROGRAMMER='" + odrv_ctx.yaml['programmer'] + "'", logger, timeout=30)
            for_all_parallel(odrives_by_name, lambda x: x['name'], odrv_reset_thread)
    except:
        logger.error('///////////////////////////////////////////')
        logger.error('/// CRITICAL: COULD NOT SECURE TEST RIG ///')
        logger.error('///     CUT THE POWER IMMEDIATELY!      ///')
        logger.error('///////////////////////////////////////////')
    else:
        logger.error('some test failed!')
else:
    logger.success('All tests succeeded!')
