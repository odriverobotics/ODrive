#!/usr/bin/python2

# run openocd (0.9.0) with :
# $ openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg &> /dev/null &
# then run
# $ python sampler.py path_to_myelf_with_symbols
# ctrl-c to stop sampling.
# To terminate the openocd session, enter command "fg" then do ctrl-c.

import sys
import time
import telnetlib
import subprocess
from bisect import bisect_right
import operator

class OpenOCDCMSampler(object):

    def __init__(self, host='localhost', port=4444):

        self.net = telnetlib.Telnet(host, port)
        self.net.read_very_eager()

        self.table = []
        self.indexes = set()

    def __del__(self):
        self.net.write(b'exit\r\n')
        self.net.read_until(b'exit\r\n', 1)
        self.net.close()

    def getpc(self):

        self.net.write(b'mrw 0xE000101C\r\n')
        res = self.net.read_until(b'\r\n\r> ', 1)

        if res:
            prefix = res[0:16]
            num    = res[16:-5]
            res    = res[-15:0]

            if prefix == b'mrw 0xE000101C\r\n':
                return int(num)

        return 0


    def initSymbols(self, elf, symbol_dump_cmd='arm-none-eabi-nm'):
        proc = subprocess.Popen([symbol_dump_cmd, '-CS', '--size-sort', elf], stdout=subprocess.PIPE)
        for line in proc.stdout.readlines():
            field = line.split()

            try:
                # For using nm -CS
                if field[2] in (b't', b'T', b'w', b'W'):
                    addr = int(field[0], 16)
                    func = b' '.join(field[3:])
                    size = int(field[1], 16)
                # # For using readelf -s
                # if field[3] == b'FUNC':
                #     addr = int(field[1], 16) - 1 # For some reason readelf dumps the func addr off by 1
                #     func = field[7]
                #     size = int(field[2])
                    if addr not in self.indexes:
                        self.table.append((addr, func, size))
                        self.indexes.add(addr)
            except IndexError:
                pass

        self.table.sort()
        self.addrs = [ x for (x, y, z) in self.table ]


    def func(self, pc):

        if pc == 0 or pc == 0xFFFFFFFF:
            return ('', 0)

        i = bisect_right(self.addrs, pc)
        if i:
            addr, symb, size = self.table[i-1]
            if pc >= addr and pc <= addr + size:
                return (symb, addr)

        return ('', 0)


if __name__ == '__main__':

    sampler = OpenOCDCMSampler('localhost', 4444)
    sampler.initSymbols(sys.argv[1])

    total = 0
    countmap = { }
    pcmap = { }
    funcmap = { }
    start = time.time()

    try:
        while True:
            pc = sampler.getpc()

            if pc in pcmap:
                pcmap[pc] += 1
            else:
                pcmap[pc] = 1

            func, addr = sampler.func(pc)

            if(func == 'ADC_IRQ_Dispatch'):
                funcmap[pc] = 1

            if not addr:
                continue

            if func in countmap:
                countmap[func] += 1
                total += 1
            else:
                countmap[func] = 1
                total += 1

            cur = time.time()
            if cur - start > 5.0:

                # tmp = sorted(funcmap)
                # for k in tmp:
                #     print(hex(k))

                tmp = sorted(countmap.items(), key=operator.itemgetter(1)) #, reverse=True)
                for k, v in tmp:
                    print('{:05.2f}% {}'.format((v * 100.) / total, k.decode('UTF-8')))
                    # print('{:06.2f} clocks : {}'.format((v * 10500) / total, k))
                start = cur
                print('{} Samples'.format(total))
                print('')
                # total = 0
                # countmap = { }
                # pcmap = { }

    except KeyboardInterrupt:
        pcmap = sorted(pcmap.items(), key=operator.itemgetter(1), reverse=True)
        pcmap = [(hex(addr), count) for addr, count in pcmap]
