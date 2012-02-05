#!/usr/bin/env python

# import wxpython GUI libraries
import wx

# import system utilities module
import psutil

# import general utilities
import re

# import ROS
import roslib; roslib.load_manifest('rover')
import rospy
from rover.msg import Enabled, Battery, Encoder

class GUIWindow(wx.Frame):
    ID_ROS_TIMER = 1
    ID_SYS_TIMER = 2
    leftvel = 0.0
    rightvel = 0.0
    leftcount = 0.0
    rightcount = 0.0
    voltage = 0.0
    current = 0.0
    isenabled = False

    def __init__(self, *args, **kwargs):
        super(GUIWindow, self).__init__(*args, **kwargs)
        rospy.init_node('rover_gui', anonymous = True)
        rospy.Subscriber('enabled', Enabled, self.enablecb)
        rospy.Subscriber('encoders', Encoder, self.enccb)
        rospy.Subscriber('battery/motor', Battery, self.battcb)
        self.enablepub = rospy.Publisher('enable', Enabled, latch = True)
        self.initUI()
        self.initTimerLoop()

    def initTimerLoop(self):
        self.rostimer = wx.Timer(self, GUIWindow.ID_ROS_TIMER)
        self.rostimer.Start(1000/30) # 30 Hz
        self.Bind(wx.EVT_TIMER, self.onROSTimerFire, id = GUIWindow.ID_ROS_TIMER)
        self.systemtimer = wx.Timer(self, GUIWindow.ID_SYS_TIMER)
        self.systemtimer.Start(1000/4) # 4 Hz
        self.Bind(wx.EVT_TIMER, self.onSystemTimerFire, id = GUIWindow.ID_SYS_TIMER)

    def onROSTimerFire(self, e):
        self.encoder_speed_label.SetLabel("Encoder Velocities: [ %.04f m/s, %.04f m/s ]" % (self.leftvel, self.rightvel))
        self.encoder_count_label.SetLabel("Encoder Counts: [ %d, %d ]" % (self.leftcount, self.rightcount))
        self.motor_battery_label.SetLabel("Motor Battery: %.02f volts, %.02f amperes" % (self.voltage, self.current))

        self.enable_toggle.SetValue(self.isenabled)

        if self.enable_toggle.GetValue():
            self.enable_toggle.SetLabel('Disable')
        else:
            self.enable_toggle.SetLabel('Enable')


    def enablecb(self, msg):
        self.isenabled = msg.motorsEnabled
    
    def enccb(self, msg):
        self.leftvel = msg.left
        self.rightvel = msg.right
        self.leftcount = msg.leftCount
        self.rightcount = msg.rightCount

    def battcb(self, msg):
        self.voltage = msg.voltage
        self.current = msg.current

    def onSystemTimerFire(self, e):
        # update CPU
        self.cpu_usage = psutil.cpu_percent()
        self.cpu_gauge.SetValue(self.cpu_usage)
        self.cpu_label.SetLabel("CPU Usage: %d%%" % self.cpu_usage)
        
        # update RAM
        self.ram_usage = psutil.phymem_usage().percent
        self.ram_gauge.SetValue(self.ram_usage)
        self.ram_label.SetLabel("RAM Usage: %d%%" % self.ram_usage)
        
        # update battery
        remaining_capacity_re = re.compile(r'^remaining capacity:\W+(\d+) mAh')
        full_capacity_re = re.compile(r'^design capacity:\W+(\d+) mAh')

        b_state_file = open('/proc/acpi/battery/BAT1/state', 'r')
        b_state = b_state_file.readlines()
        b_state_file.close()

        for line in b_state:
            m = remaining_capacity_re.match(line)
            if m:
                self.main_battery_remaining_capacity = float(m.group(1))


        b_info_file = open('/proc/acpi/battery/BAT1/info', 'r')
        b_info = b_info_file.readlines()
        b_info_file.close()

        for line in b_info:
            m = full_capacity_re.match(line)
            if m:
                self.main_battery_full_capacity = float(m.group(1))
        self.main_battery_remaining =  (100 * self.main_battery_remaining_capacity/self.main_battery_full_capacity)
        self.main_battery_label.SetLabel("Main Battery Remaining: %d%%" % self.main_battery_remaining)

    def initUI(self):
        self.initMenus()
        self.initSizers()
        self.SetSize((400, 300))
        self.SetMinSize((400, 390))
        self.SetMaxSize((400, 390))
        self.SetTitle('Rover Status')
        self.Centre()
        self.Show(True)

    def initSizers(self):
        panel = wx.Panel(self)
        font = wx.SystemSettings_GetFont(wx.SYS_SYSTEM_FONT)

        vbox = wx.BoxSizer(wx.VERTICAL)
        hbox1 = wx.BoxSizer(wx.HORIZONTAL)

        # system runtime information
        systembox = wx.BoxSizer(wx.VERTICAL)

        sbox = wx.StaticBox(panel, -1, 'System Information', (10, 10), size=(380, 185))
        self.cpu_label = wx.StaticText(panel, label = "CPU Usage: 0%")
        self.cpu_gauge = wx.Gauge(panel, -1, 100, (-1, -1), (360, 10), wx.GA_HORIZONTAL | wx.GA_SMOOTH)

        self.ram_label = wx.StaticText(panel, label = "RAM Usage: 0%")
        self.ram_gauge = wx.Gauge(panel, -1, 100, (-1, -1), (360, 10), wx.GA_HORIZONTAL | wx.GA_SMOOTH)

        self.motor_battery_label = wx.StaticText(panel, label = "Motor Battery: 0 volts, 0 amperes")
        self.main_battery_label = wx.StaticText(panel, label = "Main Battery Remaining: 0%")

        systembox.Add(self.cpu_label, 1, wx.ALIGN_TOP | wx.ALIGN_LEFT | wx.BOTTOM, 5)
        systembox.Add(self.cpu_gauge, 1, wx.ALIGN_TOP | wx.BOTTOM, 5)
        systembox.Add(self.ram_label, 1, wx.ALIGN_LEFT | wx.BOTTOM, 5)
        systembox.Add(self.ram_gauge, 1, wx.BOTTOM, 5)
        systembox.Add(wx.StaticLine(panel, -1, (-1, -1), (360, 2)), 1, wx.TOP | wx.BOTTOM)
        systembox.Add(self.motor_battery_label, 1)
        systembox.Add(self.main_battery_label, 1)
        hbox1.Add(systembox, 1, wx.ALIGN_LEFT)

        vbox.Add((0, 35), 0)
        vbox.Add(hbox1, 0, wx.ALIGN_CENTRE)

        # rover-specific information
        sbox2 = wx.StaticBox(panel, -1, 'Robot Information', (10, 200), size=(380, 160))
        enabled_label = wx.StaticText(panel, label = "Robot state:")
        self.enable_toggle = wx.ToggleButton(panel, 1, 'Enable', (-1, -1))
        self.Bind(wx.EVT_TOGGLEBUTTON, self.onToggleEnable, id = 1)

        vbox2 = wx.BoxSizer(wx.VERTICAL)
        vbox2.Add(enabled_label, 1)
        vbox2.Add(self.enable_toggle, 1)
        vbox2.Add((0, 15), 0)

        self.encoder_speed_label = wx.StaticText(panel, label = "Encoder Velocities: [ 0.00 m/s, 0.00 m/s ]")
        self.encoder_count_label = wx.StaticText(panel, label = "Encoder Counts: [ 0, 0 ]")
        vbox2.Add(self.encoder_speed_label)
        vbox2.Add((0, 15), 0)
        vbox2.Add(self.encoder_count_label)
        vbox2.Add((0, 15), 0)

        hbox2 = wx.BoxSizer(wx.HORIZONTAL)
        hbox2.Add((20, 0), 0)
        hbox2.Add(vbox2, wx.LEFT | wx.RIGHT, 20)

        vbox.Add((0, 35), 0)
        vbox.Add(hbox2, wx.ALIGN_TOP)

        # set the sizer
        panel.SetSizer(vbox)

    def initMenus(self):
        menubar = wx.MenuBar()

        # file menu
        fileMenu = wx.Menu()
        closeItem = fileMenu.Append(wx.ID_EXIT, 'Quit', 'Quit application', kind = wx.ITEM_NORMAL)
        self.Bind(wx.EVT_MENU, self.onQuit, closeItem)
        menubar.Append(fileMenu, '&File')
        
        # about menu
        aboutMenu = wx.Menu()
        aboutItem = aboutMenu.Append(wx.ID_ABOUT, 'About', 'About this program', kind = wx.ITEM_NORMAL)
        self.Bind(wx.EVT_MENU, self.onAbout, aboutItem)
        menubar.Append(aboutMenu, '&Help')
        
        self.SetMenuBar(menubar)

    def onToggleEnable(self, e):
        msg = Enabled()
        msg.motorsEnabled = self.enable_toggle.GetValue()
        msg.settingsDumpEnabled = False
        self.enablepub.publish(msg)

    def onAbout(self, e):
        aboutText = "Rover status indicator by Robert Ying"
        dlg = wx.MessageDialog(self, aboutText, "", wx.OK)
        dlg.ShowModal()
        dlg.Destroy()

    def onQuit(self, e):
        self.Close()

if __name__ == '__main__':
    try:
        app = wx.App() # create a new app
        GUIWindow(None) # create a GUIWindow
        app.MainLoop() # run the main event loop
    except rospy.ROSInterruptException:
        pass
