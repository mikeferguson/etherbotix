#!/usr/bin/env python

import time
from etherbotix_python.etherbotix import *

def main(args=None):
    e = Etherbotix()
    boot = [ord(c) for c in "BOOT"]
    e.write(253, 192, boot)
    time.sleep(3.0)

if __name__=="__main__":
    main()
