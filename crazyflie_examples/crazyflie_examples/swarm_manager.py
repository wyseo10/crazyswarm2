#!/usr/bin/env python3

import argparse
try:
	import Tkinter
except ImportError:
	import tkinter as Tkinter
from ruamel.yaml import YAML
import pathlib
import os
import math
import numpy as np
from crazyflie_py import Crazyswarm
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import cm
import collections

ns = 'com1'
# ns = ''
Z = 1.0

class CFWidget(Tkinter.Frame):
    def __init__(self, parent, name, swarm_manager):
        Tkinter.Frame.__init__(self, parent)
        self.checked = Tkinter.BooleanVar()
        checkbox = Tkinter.Checkbutton(self, variable=self.checked, command=swarm_manager.save,
                                       padx=0, pady=0)
        checkbox.grid(row=0, column=0, sticky='E')
        nameLabel = Tkinter.Label(self, text=name, padx=0, pady=0)
        nameLabel.grid(row=0, column=1, sticky='W')
        self.batteryLabel = Tkinter.Label(self, text="", fg="#999999", padx=0, pady=0)
        self.batteryLabel.grid(row=1, column=0, columnspan=2, sticky='E')
        self.versionLabel = Tkinter.Label(self, text="", fg="#999999", padx=0, pady=0)
        self.versionLabel.grid(row=2, column=0, columnspan=2, sticky='E')

class SwarmManager():
    def __init__(self, configpath):
        # dragging functionality - TODO alt-drag to deselect
        self.drag_start = None
        self.drag_startstate = None

        # flags
        self.planning_started = False
        self.get_key_input = False

        # cfg 
        self.configpath = configpath
        yaml = YAML()
        self.cfg = yaml.load(pathlib.Path(configpath))
        cfTypes = self.cfg["robot_types"]
        enabled = [name for name in self.cfg["robots"].keys() if self.cfg["robots"][name]["enabled"] == True]

        print("Waiting for the cfserver (ros2 launch crazyflie launch.py)")
        swarm = Crazyswarm(enabled, ns)
        self.timeHelper = swarm.timeHelper
        self.allcfs = swarm.allcfs
        print("Connected to the cfserver")

        # compute absolute pixel coordinates from the initial positions
        positions = [node["initial_position"] for node in self.cfg["robots"].values()]
        DOWN_DIR = [-1, 0]
        RIGHT_DIR = [0, -1]
        pixel_x = [120 * self.dot(pos, RIGHT_DIR) for pos in positions]
        pixel_y = [120 * self.dot(pos, DOWN_DIR) for pos in positions]
        xmin, ymin = min(pixel_x), min(pixel_y)
        xmax, ymax = max(pixel_x), max(pixel_y)

        # construct the main window
        top = Tkinter.Tk()
        top.title('Swarm Manager')

        # construct the frame containing the absolute-positioned checkboxes
        width = xmax - xmin + 50  # account for checkbox + text width
        height = ymax - ymin + 50  # account for checkbox + text height
        frame = Tkinter.Frame(top, width=width, height=height)

        # construct all the checkboxes
        self.widgets = {}
        for (id, node), x, y in zip(self.cfg["robots"].items(), pixel_x, pixel_y):
            w = CFWidget(frame, str(id), self)
            w.place(x=x - xmin, y=y - ymin)
            w.checked.set(id in enabled)
            self.widgets[id] = w

        top.bind('<ButtonPress-1>', self.mouseDown)
        top.bind('<ButtonPress-3>', self.mouseDown)
        top.bind('<B1-Motion>', lambda event: self.drag(event, True))
        top.bind('<B3-Motion>', lambda event: self.drag(event, False))
        top.bind('<ButtonRelease-1>', self.mouseUp)
        top.bind('<ButtonRelease-3>', self.mouseUp)
        top.focus_set()	
        top.bind('<Key>', self.handleKey)

        buttons = Tkinter.Frame(top)
        self.mkbutton(buttons, "Clear", self.clear)
        self.mkbutton(buttons, "Fill", self.fill)

        scriptButtons = Tkinter.Frame(top)
        self.mkbutton(scriptButtons, "Takeoff", self.takeoff)
        self.mkbutton(scriptButtons, "GoToGoal", self.goToGoal)
        self.mkbutton(scriptButtons, "Patrol", self.patrol)
        self.mkbutton(scriptButtons, "GoToStart", self.goToStart)
        self.mkbutton(scriptButtons, "Land", self.land)
        self.mkbutton(scriptButtons, "Kill", self.emergencyStop)

        buttons.pack()
        frame.pack(padx=10, pady=10)
        scriptButtons.pack()
        top.mainloop()    


    def selected_cfs(self):
        nodes = {name: node for name, node in self.cfg["robots"].items() if self.widgets[name].checked.get()}
        cfs = [self.allcfs.crazyfliesByName[i] for i in nodes]
        return cfs


    def save(self):
        yaml = YAML()
        for name, node in self.cfg["robots"].items():
            if self.widgets[name].checked.get():
                node["enabled"] = True
            else:
                node["enabled"] = False
        with open(self.configpath, 'w') as outfile:
            yaml.dump(self.cfg, outfile)


    def dot(self, a, b):
        return a[0] * b[0] + a[1] * b[1]


    def minmax(self, a, b):
        return min(a, b), max(a, b)


    def mouseDown(self, event):
        self.drag_start = (event.x_root, event.y_root)
        self.drag_startstate = [cf.checked.get() for cf in self.widgets.values()]


    def mouseUp(self, event):
        self.save()


    def drag(self, event, select):
        x, y = event.x_root, event.y_root
        dragx0, dragx1 = self.minmax(self.drag_start[0], x)
        dragy0, dragy1 = self.minmax(self.drag_start[1], y)

        def dragcontains(widget):
            x0 = widget.winfo_rootx()
            y0 = widget.winfo_rooty()
            x1 = x0 + widget.winfo_width()
            y1 = y0 + widget.winfo_height()
            return not (x0 > dragx1 or x1 < dragx0 or y0 > dragy1 or y1 < dragy0)

        # depending on interation over dicts being consistent
        for initial, cf in zip(self.drag_startstate, self.widgets.values()):
            if dragcontains(cf):
                cf.checked.set(select)
            else:
                cf.checked.set(initial)


    # buttons for clearing/filling all checkboxes
    def clear(self):
        for box in self.widgets.values():
            box.checked.set(False)
        self.save()


    def fill(self):
        for box in self.widgets.values():
            box.checked.set(True)
        self.save()


    def takeoff(self):
        cfs = self.selected_cfs()
        for cf in cfs:
            cf.takeoff(targetHeight=Z, duration=1.0 + Z)
        self.timeHelper.sleep(1.0 + Z)
        print("Take off")


    def activatePlanner(self):
        cfs = self.selected_cfs()
        if not self.planning_started:
            for cf in cfs:
                cf.initializePlanningServices()
            self.planning_started = True


    def goToGoal(self):
        self.activatePlanner()
        cfs = self.selected_cfs()
        for cf in cfs:
            cf.goToGoal()
        print("Go to the goal point")


    def patrol(self):
        self.activatePlanner()
        cfs = self.selected_cfs()
        for cf in cfs:
            cf.patrol()
        print("Patrol between the start and goal points")


    def goToStart(self):
        self.activatePlanner()
        cfs = self.selected_cfs()
        for cf in cfs:
            cf.goToStart()
        print("Go to the start point")


    def land(self):
        cfs = self.selected_cfs()
        if self.planning_started:
            self.planning_started = False
            for cf in cfs:
                cf.stopPlanning()
                cf.notifySetpointsStop()

        for cf in cfs:
            cf.land(targetHeight=0.02, duration=1.0 + Z)
        print("Land")


    def emergencyStop(self):
        self.planning_started = False
        self.allcfs.emergency()
        print("Emergency stop")

    def mkbutton(self, parent, name, command):
        button = Tkinter.Button(parent, text=name, command=command)
        button.pack(side='left')


    def handleKey(self, event):
        if self.planning_started:
            return

        cfs = self.selected_cfs()
        k = event.char
        if k == 'w':
            for cf in cfs:
                cf.goTo(np.array([0.2, 0, 0]), 0, 0.05, relative=True)
            print("Front")
        elif k == 's':
            for cf in cfs:
                cf.goTo(np.array([-0.2, 0, 0]), 0, 0.05, relative=True)
            print("Back")
        elif k == 'd':
            for cf in cfs:
                cf.goTo(np.array([0, -0.2, 0]), 0, 0.05, relative=True)
            print("Right")
        elif k == 'a':
            for cf in cfs:
                cf.goTo(np.array([0, 0.2, 0]), 0, 0.05, relative=True)
            print("Left")
        elif k == 'z':
            for cf in cfs:
                cf.goTo(np.array([0, 0, 0.1]), 0, 0.03, relative=True)
            print("Up")
        elif k == 'c':
            for cf in cfs:
                cf.goTo(np.array([0, 0, -0.1]), 0, 0.03, relative=True)
            print("Down")
        elif k == 'q':
            for cf in cfs:
                cf.goTo(np.array([0, 0, 0]), math.pi / 8, 0.1, relative=True)
            print("Turn left")
        elif k == 'e':
            for cf in cfs:
                cf.goTo(np.array([0, 0, 0]), -math.pi / 8, 0.1, relative=True)
            print("Turn right")
        elif k == 'k':
            self.emergencyStop()
        elif k == 't':
            self.takeoff()
        elif k == 'l':
            self.land()
        elif k == 'g':
            self.goToGoal()
        elif k == 'p':
            self.patrol()
        elif k == 'b':
            self.goToStart()
        else:
            print("Invalid key")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
	    "--configpath",
	    type=str,
	    default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../crazyflie/config/crazyflies.yaml"),
	    help="Path to the configuration .yaml file")
    args = parser.parse_args()

    if not os.path.exists(args.configpath):
        print("ERROR: Could not find yaml configuration file in configpath ({}).".format(args.configpath))
        exit()

    manager = SwarmManager(args.configpath)


if __name__ == '__main__':
    main()