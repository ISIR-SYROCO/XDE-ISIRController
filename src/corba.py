#!/xde

""" Module to make a corba connection with some orcisir_ISIRController classes.

Mainly to connect some modelState (SegmentFrame, TargetFrame, CoMFrame) and some
model between a proxy part (the master script for control) and a remote part
(the slave script for control).

The remote script should be run first, to register elements in the corba server,
Then the proxy script is run to get the element from the corba server, and to
initialize the proxy component

"""


import sys
import rtt_interface_corba
rtt_interface_corba.Init(sys.argv)

import xdefw.rtt

def getProxyTaskContext(proxyName):
    proxyTC  = rtt_interface_corba.GetProxy(proxyName, False) #IDK what the boolean stands for?!?
    if proxyTC is None:
        raise ValueError("proxy TaskContext is None; check connection with corba server, or the proxy element name you want to reach")
    return xdefw.rtt.Task(proxyTC)

