#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import time

def main():
    # allow some time for the Crazyflies to connect and setup parameters
    time.sleep(22.0)  # wait for the Crazyflies to connect
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # disable LED (one by one)
    for cf in allcfs.crazyflies:
        cf.setParam('led.bitmask', 128)
        timeHelper.sleep(1.0)
        if cf.getParam('led.bitmask') != 128:
            print('LED is not disabled!')

    timeHelper.sleep(2.0)

    # enable LED (broadcast)
    allcfs.setParam('led.bitmask', 0)
    timeHelper.sleep(5.0)


if __name__ == '__main__':
    main()
