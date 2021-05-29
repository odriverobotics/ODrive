#!/usr/bin/env bash
set -euo pipefail

declare -a tests=('analog_input_test.py'
                  'calibration_test.py'
                  'can_test.py'
                  'closed_loop_test.py'
                  'encoder_test.py'
                  'fibre_test.py'
                  'integration_test.py'
                  'nvm_test.py'
                  'pwm_input_test.py'
                  'sensor_test.py'
                  'step_dir_test.py'
                  'uart_ascii_test.py'
                  )
summary=""

for test in "${tests[@]}"; do
    (ipython3 "$test" || true) | tee /tmp/odrivetest.log
    if grep "All tests passed!" /tmp/odrivetest.log; then
        summary="$summary - $test: passed"$'\n'
    else
        summary="$summary - $test: failed"$'\n'
    fi

    echo "########################"
    echo "Current status:"
    echo -n "$summary"
    echo "########################"
done
