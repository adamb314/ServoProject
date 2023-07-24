'''
Module with useful GUI functions
'''

import gi
gi.require_version("Gtk", "3.0")
from gi.repository import GLib, Gtk  # pylint: disable=wrong-import-position

def transferToTargetMessage(parent):
    dialog = Gtk.MessageDialog(
            transient_for=parent,
            flags=0,
            message_type=Gtk.MessageType.INFO,
            buttons=Gtk.ButtonsType.OK,
            text='Transfer to target needed!',
    )
    dialog.format_secondary_text(
        "Please transfer new configuration to target to apply changes"
    )
    dialog.run()
    dialog.destroy()
    parent.setFocusOnTranferButton()

def disconnectMotorFromGearboxMessage(parent):
    dialog = Gtk.MessageDialog(
            transient_for=parent,
            flags=0,
            message_type=Gtk.MessageType.INFO,
            buttons=Gtk.ButtonsType.OK,
            text='Disconnect motor from gearbox!',
    )
    dialog.format_secondary_text(
        "Please make sure the motor can run freely by disconnecting it from the gearbox before continuing"
    )
    dialog.run()
    dialog.destroy()

def startManuallyCalibrationMessage(parent, timeString):
    dialog = Gtk.MessageDialog(
            transient_for=parent,
            flags=0,
            message_type=Gtk.MessageType.INFO,
            buttons=Gtk.ButtonsType.OK,
            text='Start manual calibration',
    )
    dialog.format_secondary_text(
        "Please connect the motor to the gearbox and move the servo-output-shaft "
        "back and forth for " + timeString + " over the whole range of motion, "
        "with constant speed."
    )
    dialog.run()
    dialog.destroy()

def exceptionMessage(parent, e):
    def showErrorFunc(parent, e):
        dialog = Gtk.MessageDialog(
                transient_for=parent,
                flags=0,
                message_type=Gtk.MessageType.ERROR,
                buttons=Gtk.ButtonsType.OK,
                text='Exception',
        )
        dialog.format_secondary_text(f'{e!r}')
        dialog.run()
        dialog.destroy()
    GLib.idle_add(showErrorFunc, parent, e)

def nullFunEvent(widget):
    pass

def passOnScroll(widget, event):
    widget.stop_emission_by_name('scroll-event')

def createLabel(text):
    label = Gtk.Label(label=text)
    label.set_use_markup(True)
    label.set_margin_start(30)
    label.set_margin_end(50)
    label.set_margin_top(8)
    label.set_margin_bottom(10)
    label.set_xalign(0.0)

    return label

def addTopLabelTo(text, widget):
    boxHorizontal = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
    boxHorizontal.add(widget)

    label = Gtk.Label(label=text)
    label.set_use_markup(True)
    label.set_margin_start(30)
    label.set_margin_end(50)
    label.set_margin_top(8)
    label.set_margin_bottom(10)
    label.set_xalign(0.0)

    box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
    box.add(label)
    box.add(boxHorizontal)

    return box

def getActiveComboBoxItem(comboBox):
    items = comboBox.get_model()
    return items[comboBox.get_active()][0]

def setComboBoxItems(comboBox, currentItem, itemList):
    activeIndex = -1
    items = Gtk.ListStore(str)
    for i, name in enumerate(itemList):
        n = []
        n.append(name)
        items.append(n)

        if currentItem == name:
            activeIndex = i

    comboBox.set_model(items)
    comboBox.set_active(activeIndex)

def creatComboBox(currentItem, itemList, onChangeFun = nullFunEvent, getLowLev = False):
    comboBox = Gtk.ComboBox.new_with_model(Gtk.ListStore(str))
    comboBox.set_margin_start(40)
    comboBox.set_margin_end(10)
    comboBox.set_margin_top(10)
    comboBox.set_margin_bottom(8)
    rendererText = Gtk.CellRendererText()
    comboBox.pack_start(rendererText, True)
    comboBox.add_attribute(rendererText, "text", 0)

    setComboBoxItems(comboBox, currentItem, itemList)

    comboBox.connect("changed", onChangeFun)
    comboBox.connect("scroll-event", passOnScroll)

    eventBox = Gtk.EventBox()
    box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
    box.add(comboBox)
    eventBox.add(box)

    if getLowLev:
        return eventBox, comboBox

    return eventBox

def creatSpinButton(startValue, minValue, maxValue, stepSize, *, onChangeFun = nullFunEvent, getLowLev = False):
    spinButton = Gtk.SpinButton.new_with_range(min=minValue, max=maxValue, step=stepSize)
    spinButton.set_value(startValue)
    spinButton.set_margin_start(40)
    spinButton.set_margin_end(50)
    spinButton.set_margin_top(10)
    spinButton.set_margin_bottom(8)
    spinButton.connect("changed", onChangeFun)
    spinButton.connect("scroll-event", passOnScroll)

    box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
    box.add(spinButton)

    if getLowLev:
        return box, spinButton

    return box

def createButton(text, onClickFun = nullFunEvent, width = 500, getLowLev = False):
    button = Gtk.Button(label=text)
    button.connect("clicked", onClickFun)
    button.set_margin_start(40)
    button.set_margin_end(50)
    button.set_margin_top(8)
    button.set_margin_bottom(10)
    button.set_property("width-request", width)

    box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
    box.add(button)

    if getLowLev:
        return box, button

    return box

def createToggleButton(text, onClickFun = nullFunEvent, width = 500, getLowLev = False):
    button = Gtk.ToggleButton(label=text)
    button.connect("toggled", onClickFun)
    button.set_margin_start(40)
    button.set_margin_end(50)
    button.set_margin_top(8)
    button.set_margin_bottom(10)
    button.set_property("width-request", width)

    box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
    box.add(button)

    if getLowLev:
        return box, button

    return box

def createEntry(initText, onEdit = nullFunEvent, width = 500, getLowLev = False):
    entry = Gtk.Entry()
    entry.set_text(initText)
    entry.connect('changed', onEdit)
    entry.set_margin_start(40)
    entry.set_margin_end(50)
    entry.set_margin_top(8)
    entry.set_margin_bottom(10)
    entry.set_property("width-request", width)

    box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
    box.add(entry)

    if getLowLev:
        return box, entry

    return box

def createLabelBox(text, orientation=Gtk.Orientation.VERTICAL):

    label = Gtk.Label(label=text)
    label.set_use_markup(True)
    label.set_margin_start(10)
    label.set_margin_end(10)
    label.set_margin_top(8)
    label.set_margin_bottom(10)
    label.set_xalign(0.0)

    box = Gtk.Box(orientation=orientation)

    box.add(label)

    return box

def creatHScale(startValue, minValue, maxValue, stepSize, *,
                onChangeFun = nullFunEvent, width = 500, getLowLev = False):
    scale = Gtk.Scale.new_with_range(orientation=Gtk.Orientation.HORIZONTAL, min=minValue, max=maxValue, step=stepSize)
    scale.set_value(startValue)
    scale.set_margin_start(40)
    scale.set_margin_end(50)
    scale.set_margin_top(10)
    scale.set_margin_bottom(8)
    scale.set_property("width-request", width)
    scale.connect("value-changed", onChangeFun)
    scale.connect("scroll-event", passOnScroll)

    box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
    box.add(scale)

    if getLowLev:
        return box, scale

    return box

def creatProgressBar(label, width = 500, getLowLev = False):
    progressBar = Gtk.ProgressBar()
    if label != '':
        progressBar.set_text(label)
        progressBar.set_show_text(True)
    progressBar.set_fraction(0.0)
    progressBar.set_margin_start(40)
    progressBar.set_margin_end(50)
    progressBar.set_margin_top(10)
    progressBar.set_margin_bottom(8)
    progressBar.set_property("width-request", width)

    box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
    box.add(progressBar)

    if getLowLev:
        return box, progressBar

    return box

class ControlParameters:
    def __init__(self, mainSpeed, velControlSpeed, filterSpeed, inertiaMarg):
        self.mainSpeed = None
        self.velControlSpeedRatio = None
        self.filterSpeedRatio = None
        self.inertiaMarg = None
        self.setValues(mainSpeed, velControlSpeed, filterSpeed, inertiaMarg)

    def getValues(self):
        velControlSpeed = self.mainSpeed * self.velControlSpeedRatio
        filterSpeed = self.mainSpeed * self.velControlSpeedRatio * self.filterSpeedRatio
        return (self.mainSpeed, velControlSpeed, filterSpeed, self.inertiaMarg)

    def getRatios(self):
        return (self.velControlSpeedRatio, self.filterSpeedRatio)

    def getMainSpeed(self):
        return self.mainSpeed

    def setValues(self, mainSpeed, velControlSpeed, filterSpeed, inertiaMarg):
        self.mainSpeed = mainSpeed
        self.velControlSpeedRatio = velControlSpeed / mainSpeed
        self.filterSpeedRatio = filterSpeed / velControlSpeed
        self.inertiaMarg = inertiaMarg

    def setMainSpeed(self, mainSpeed):
        self.mainSpeed = mainSpeed

def openAdvancedParametersDialog(parent, currentParameters):
    # pylint: disable=too-many-locals, too-many-statements
    dialog = Gtk.MessageDialog(
            transient_for=parent,
            flags=0,
            message_type=Gtk.MessageType.OTHER,
            buttons=Gtk.ButtonsType.OK,
            text='Advanced parameters',
    )
    dialog.format_secondary_text(
        ''
    )
    box = dialog.get_message_area()

    velControlSpeedRatio, filterSpeedRatio = currentParameters.getRatios()
    controlSpeed, velControlSpeed, filterSpeed, inertiaMarg = currentParameters.getValues()

    controlSpeedScale = creatHScale(controlSpeed, 0, 100, 1, getLowLev=True)
    controlSpeedScale = addTopLabelTo('<b>Control speed</b>', controlSpeedScale[0]), controlSpeedScale[1]
    box.pack_start(controlSpeedScale[0], False, False, 0)

    velControlSpeedScale = creatHScale(velControlSpeed, 0, 100 * 4, 4, getLowLev=True)
    velControlSpeedScale = (addTopLabelTo('<b>Velocity control speed</b>', velControlSpeedScale[0]),
                            velControlSpeedScale[1])
    box.pack_start(velControlSpeedScale[0], False, False, 0)

    filterSpeedScale = creatHScale(filterSpeed, 0, 100 * 32, 32, getLowLev=True)
    filterSpeedScale = addTopLabelTo('<b>Filter speed</b>', filterSpeedScale[0]), filterSpeedScale[1]
    box.pack_start(filterSpeedScale[0], False, False, 0)

    inertiaMargScale = creatHScale(inertiaMarg, 1.0, 3.0, 0.1, getLowLev=True)
    inertiaMargScale = addTopLabelTo('<b>Inertia margin</b>',
            inertiaMargScale[0]), inertiaMargScale[1]
    box.pack_start(inertiaMargScale[0], False, False, 0)
    box.show_all()

    secondaryEvent = False

    def onControlSpeedScaleChange(widget):
        nonlocal secondaryEvent
        nonlocal controlSpeed
        controlSpeed = controlSpeedScale[1].get_value()

        secondaryEvent = True

        velControlSpeedScale[1].set_value(controlSpeed * velControlSpeedRatio)

        secondaryEvent = False

    def onVelControlSpeedScaleChange(widget):
        nonlocal secondaryEvent
        nonlocal velControlSpeed
        nonlocal velControlSpeedRatio
        velControlSpeed = velControlSpeedScale[1].get_value()
        velControlSpeed = int(round(velControlSpeed / 4)) * 4

        isDrivingWidget = not secondaryEvent
        if isDrivingWidget:
            velControlSpeedRatio = velControlSpeed / controlSpeed
            secondaryEvent = True

        filterSpeedScale[1].set_value(velControlSpeed * filterSpeedRatio)

        if velControlSpeed != velControlSpeedScale[1].get_value():
            velControlSpeedScale[1].set_value(velControlSpeed)

        if isDrivingWidget:
            secondaryEvent = False

    def onFilterSpeedScaleChange(widget):
        nonlocal secondaryEvent
        nonlocal filterSpeed
        nonlocal filterSpeedRatio
        filterSpeed = filterSpeedScale[1].get_value()
        filterSpeed = int(round(filterSpeed / 32)) * 32

        isDrivingWidget = not secondaryEvent
        if isDrivingWidget:
            filterSpeedRatio = filterSpeed / velControlSpeed
            secondaryEvent = True

        if filterSpeed != filterSpeedScale[1].get_value():
            filterSpeedScale[1].set_value(filterSpeed)

        if isDrivingWidget:
            secondaryEvent = False

    def onInertiaMargScaleChange(widget):
        nonlocal inertiaMarg
        inertiaMarg = inertiaMargScale[1].get_value()

    controlSpeedScale[1].connect('value-changed', onControlSpeedScaleChange)
    velControlSpeedScale[1].connect('value-changed', onVelControlSpeedScaleChange)
    filterSpeedScale[1].connect('value-changed', onFilterSpeedScaleChange)
    inertiaMargScale[1].connect('value-changed', onInertiaMargScaleChange)

    dialog.run()
    dialog.destroy()

    return ControlParameters(controlSpeed, velControlSpeed, filterSpeed, inertiaMarg)
