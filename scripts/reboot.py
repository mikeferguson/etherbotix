#!/usr/bin/env python

import time
from etherbotix.etherbotix import *

if __name__ == "__main__":
    e = Etherbotix()
    boot = [ord(c) for c in "BOOT"]
    e.write(253, 192, boot)
