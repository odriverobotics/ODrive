#!/bin/bash
openocd -d3 -f board/stm32f4discovery.cfg  -c "hla_serial wrong_serial" 2>&1 | \
    xxd -p | \
    tr -d '\n' | \
    sed -n 's/^.*6e756d6265722027\([0-9a-f]*\)2720646f65736e27.*$/\1/p' | sed -e 's/.\{2\}/\\x&/g'; echo
