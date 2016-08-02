#!/usr/bin/env python

import os, signal, sys, subprocess, re




usage = """
{0} <parent pid>
"""

pspid = re.compile('\s+([0-9a-f]+)\s+', re.IGNORECASE)

def children(pid):
    bairns = []
    try:
        data = subprocess.check_output(["ps", "--ppid", pid])
        lines = data.split('\n')
        for line in lines:
            m = pspid.match(line)
            if m:
                bairns.append(m.group(1))
    except:
        pass
    return bairns


def descendents(pidlist, offspring):
    if not pidlist:
        return offspring
    else:
        pid = pidlist[0]
        cdr = pidlist[1:]
        bairns = children(pid)
        cdr.extend(bairns)
        offspring.insert(0, int(pid))
        return descendents(cdr, offspring)
        

def infanticide(pid):
    if pid is None:
        pid = str(os.getpid())
    bairns = descendents([str(pid), ], [])
    for pid in bairns:
        os.kill(pid, signal.SIGKILL)
    

    
def main():
    args = sys.argv
    if len(sys.argv) != 2:
        print usage.format(sys.argv[0])
    else:
        pid = sys.argv[1]
        print 'parent = {0}\n'.format(pid)
        print descendents([pid, ], [])
        infanticide(pid)
    return 0


if __name__ == '__main__':
    sys.exit(main())
