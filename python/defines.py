#!/usr/bin/env python3
import re
import sys
import pickle

defines = open('../atmelStudio/V2.0/datagram.h', 'r')

enum = False
enumval = None

enumdict = {}

enumdef = re.compile(r'''
        \s*
        (?P<symb>[A-Z_][A-Z0-9_]*)   # enum symbol
        \s*
        (=\s*
            (?P<val>(0x[0-9a-fA-F]+)|([0-9]+))  # hex or decimal constant
        )? # optional assignment to constant
        ,?      # optional comma at end of line
        ''', re.VERBOSE)

with defines:
    while True:
        line = defines.readline()
        if not line:
            break
        if line == '\n':
            continue
        if line.startswith('enum'):
            enum = True
            continue
        if line.startswith('}'):
            enum = False
            continue

        if not enum:
            continue

        m = enumdef.match(line)
        if not m:
            continue;

        if m.group('val'):
            val = int(m.group('val'), 0)
        else:
            val = enumval + 1;
        enumval = val

        enumdict[m.group('symb')] = val

for sym in sorted(enumdict.keys()):
    print(sym)

with open('atmel-symbols.pickle', 'wb') as file:
    pickle.Pickler(file).dump(enumdict)
