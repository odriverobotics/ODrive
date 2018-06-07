# Automated Testing

This section describes how to use the automated testing facilities.
You don't have to do this as an end user.

They test the following aspects:
 - System functions (communication interfaces, configuration storage)
 - Functionality of the motor controller and state machine
 - High speed and high load conditions

The testing facility consists of the following components:
 * **Test rig:** In the simplest case this can be a single ODrive with a single motor and encoder pair. Can also be multiple ODrives with multiple axes, some of which may be mechanically coupled.
 * **Test host:** The PC on which the test script runs. All ODrives must be connected to the test host via USB.
 * **test-rig.yaml:** Describes your test rig. Make sure all values are correct. Incorrect values may physically break or fry your test setup.
 * **run_tests.py:** This is the main script that runs all the tests.

## How to run

Example:

```
./run_tests.py --skip-boring-tests --ignore top-odrive.yellow bottom-odrive.yellow
```
