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
    TestFlashAndErase(),
    TestSetup(),
    TestMotorCalibration(),
    # TODO: test encoder index search
    TestEncoderOffsetCalibration(),
    TestClosedLoopControl(),
    TestStoreAndReboot(),
    TestEncoderOffsetCalibration(), # need to find offset _or_ index after reboot
    TestClosedLoopControl()
    # TODO: test step/dir
    # TODO: test sensorless
    # TODO: test ASCII protocol
    # TODO: test protocol over UART
]


logger = Logger()

with open('test-rig.yaml', 'r') as file_stream:
    test_rig_yaml = yaml.load(file_stream)

# Ensure every device has a name
for idx, odrv_yaml in enumerate(test_rig_yaml['odrives']):
    if not 'name' in odrv_yaml:
        odrv_yaml['name'] = 'odrive{}'.format(idx)

# Build a dictionary of axes by name (e.g. odrive0.axis0)
# Also ensure every axis has a name and mutex
axes_by_name = {}
for odrv_yaml in test_rig_yaml['odrives']:
    for axis_idx, axis_yaml in enumerate(odrv_yaml['axes']):
        if not 'name' in axis_yaml:
            axis_yaml['name'] = '{}.axis{}'.format(odrv_yaml['name'], axis_idx)
        axis_yaml['lock'] = threading.Lock()
        axes_by_name[axis_yaml['name']] = axis_yaml

# Ensure mechanical couplings are valid
if test_rig_yaml['couplings'] is None:
    test_rig_yaml['couplings'] = {}
else:
    for axis in sum(test_rig_yaml['couplings'], []):
        if not axis in axes_by_name:
            logger.error('Unknown axis {} in list of mechanical couplings'.format(axis))


try:
    for test in all_tests:
        if isinstance(test, ODriveTest):
            def odrv_test_thread(odrv_yaml):
                test_subject_name = odrv_yaml['name']
                logger.info('● running {} on {}...'.format(type(test).__name__, test_subject_name))
                odrv = odrv_yaml['odrv'] if 'odrv' in odrv_yaml else None
                test.run_test(odrv, odrv_yaml,
                              logger.indent('  {}: '.format(test_subject_name)))

            for_all_parallel(test_rig_yaml['odrives'], lambda x: x['name'], odrv_test_thread)

        elif isinstance(test, AxisTest):
            def axis_test_thread(axis_name):
                # Get all axes that are mechanically coupled with the axis specified by axis_name
                conflicting_axes = sum([c for c in test_rig_yaml['couplings'] if (axis_name in c)], [])
                # Remove duplicates
                conflicting_axes = list(set(conflicting_axes))
                # Acquire lock for all conflicting axes
                conflicting_axes.sort() # prevent deadlocks
                for conflicting_axis in conflicting_axes:
                    axes_by_name[conflicting_axis]['lock'].acquire()
                try:
                    # Run test on this axis
                    logger.info('● running {} on {}...'.format(type(test).__name__, axis_name))
                    axis_yaml = axes_by_name[axis_name]
                    test.run_test(axis_yaml['axis'], axis_yaml,
                                  logger.indent('  {}: '.format(axis_name)))
                finally:
                    # Release all conflicting axes
                    for conflicting_axis in conflicting_axes:
                        axes_by_name[conflicting_axis]['lock'].release()

            for_all_parallel(axes_by_name, lambda x: x, axis_test_thread)

        else:
            logger.warn("ignoring unknown test type {}".format(type(test)))

except:
    logger.error(traceback.format_exc())
    logger.debug('=> Test failed. Please wait while I secure the test rig...')
    try:
        dont_secure_after_failure = True # TODO: disable
        if not dont_secure_after_failure:
            def odrv_reset_thread(odrv_yaml):
                run("make erase PROGRAMMER='" + odrv_yaml['programmer'] + "'", logger, timeout=30)
            for_all_parallel(test_rig_yaml['odrives'], lambda x: x['name'], odrv_reset_thread)
    except:
        logger.error('///////////////////////////////////////////')
        logger.error('/// CRITICAL: COULD NOT SECURE TEST RIG ///')
        logger.error('///     CUT THE POWER IMMEDIATELY!      ///')
        logger.error('///////////////////////////////////////////')
    else:
        logger.error('some test failed!')
else:
    logger.success('All tests succeeded!')
