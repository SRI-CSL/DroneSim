#!/usr/bin/env python

from subprocess import Popen, PIPE, STDOUT

from threading import Thread

import sys



def echo(child):
    for line in child.stdout:
        sys.stderr.write(line)

    if not child.stdout.closed:
        child.stdout.close()
        

class SandBox(object):

    def __init__(self, name, argv, stdout2stderr):
        self.name = name
        self.argv = argv
        self.stdout2stderr = stdout2stderr
        self.child = None

    def start(self):
        if self.stdout2stderr:
            self.child = Popen(self.argv, stdin=PIPE, stdout=sys.stderr, stderr=sys.stderr)
        else:
            self.child = Popen(self.argv, stdin=PIPE, stdout=PIPE, stderr=sys.stderr)
            thread = Thread(target=echo, name='echo_of_{0}'.format(self.name), args=(self.child, ))
            #thread needs to be a daemon so that the actor itself can die in peace.
            thread.daemon = True 
            thread.start()


    def stop(self):
        if self.child is not None:
            self.child.terminate()
            self.child.kill()
            

    def wait(self):
        if self.child is not None:
            self.child.wait()

    def getpid(self):
        if self.child is not None:
            return self.child.pid

        
def test():
    mp_argv = [ 'mavproxy.py',
                '--master',
                'tcp:127.0.0.1:5760',
                '--sitl=127.0.0.1:5501',
                '--out=127.0.0.1:14550',
                '--out=127.0.0.1:14551',
                '--map',
                '--console',
                '--aircraft',
                'test' ]
    

    dk_argv = [ 'dronekit-sitl',  'copter',  '--home=-7.162675,-34.817705,36,250' ]
    SandBox('dronekit', dk_argv, False).start()
    SandBox('mavproxy', mp_argv, True).start()

    
if __name__ == '__main__':
    test()

            
    
