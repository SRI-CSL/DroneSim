#!/usr/bin/env python

from subprocess import Popen, PIPE, STDOUT

from threading import Thread

import os, sys, psutil, signal

debug = False

def infanticide(pid):
    try:
      parent = psutil.Process(pid)
    except psutil.NoSuchProcess:
      return
    children = parent.children(recursive=True)
    if debug:
        sys.stderr.write('The children of {0} are {1}\n'.format(pid, children))
    for p in children:
        os.kill(p.pid, signal.SIGKILL)
        if debug:
            sys.stderr.write('Sent signal {1} to {0}\n'.format(p.pid, signal.SIGKILL))


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
            infanticide(self.child.pid)
            self.child.terminate()
            self.child.kill()
            self.child.wait()


    def wait(self):
        if self.child is not None:
            self.child.wait()

    def getpid(self):
        if self.child is not None:
            return self.child.pid
