## @package SDFusion
# This is an exporter for Autodesk Fusion 360 models to SDFormat.
#
# This can be loaded as an addon in Fusion 360.
# It exports all rigid groups of the robot model as links
# to STL and creates nodes in the SDF file for them.
# It creates SDF nodes for all joints of the robot model.
# Supported joint types are: "fixed", "revolute", and "ball".

import adsk.core
import adsk.fusion
import traceback
import xml.etree.ElementTree as ET

from .exporter import SDFExporter
from .helpers import *

commandId = 'SDFusionExporter'
commandName = 'SDFusion'
commandDescription = 'cardsflow exporter'

# Global set of event handlers to keep them referenced for the duration of the command
handlers = []
rootComp = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct).rootComponent
allVP = []


# Event handler that reacts to any changes the user makes to any of the command inputs.
class SDFusionInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            command = args.firingEvent.sender
            inputs = command.commandInputs

            # if viapoints tab is inactive disable selection input
            inputs[-1].isEnabled = inputs.itemById("SDFusionExporter_tab_2").isActive
            inputs[-1].isVisble = inputs[-1].isEnabled

            eventArgs = adsk.core.InputChangedEventArgs.cast(args)
            inputs = eventArgs.inputs
            cmdInput = eventArgs.input

            if cmdInput.id == 'selection':# and cmdInput.id != 'number' :
                self.updateViaPoints(inputs)

        except:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

    def updateViaPoints(self, inputs):
        muscleInput = inputs.itemById('muscle')
        muscle = muscleInput.value
        selInput = inputs.itemById('selection')
        selection = selInput.selection(0)
        entity = selection.entity
        edge = adsk.fusion.BRepEdge.cast(entity)
        numberInput = inputs.itemById('number')
        number = numberInput.value
        # get link name
        linkInput = inputs.itemById('link').selectedItem
        # TODO check if link selected
        link = '?'
        if linkInput:
            link = linkInput.name

        # rootComp = adsk.fusion.Design.cast(self.app.activeProduct).rootComponent
        conPoints = rootComp.constructionPoints
        # Create construction point input
        pointInput = conPoints.createInput()
        # Create construction point by center
        pointInput.setByCenter(edge)
        point = conPoints.add(pointInput)
        point.name = "VP_motor" + muscle + "_" + link + "_" + number

        vp = ViaPoint(motor=muscle, link=link, number=number, edge=edge)

        global allVP
        allVP.append(vp)

        # automatically increase VP number by 1
        numberInput.value = str(int(number) + 1)

class SDFusionDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            #TODO fix the bug when viapoints get deleted on destroy
            self.preserveViaPoints()

            eventArgs = adsk.core.CommandEventArgs.cast(args)
            if (eventArgs.terminationReason == 1): # "Export to SDF" clicked

                # Get the values from the command inputs.
                inputs = eventArgs.command.commandInputs

                try:
                    exporter = SDFExporter()
                    exporter.updateFlags(inputs, commandId)
                    if exporter.askForExportDirectory():
                        exporter.createDiectoryStructure()

                        if exporter.runCleanUp:
                            exporter.removeSmallParts()

                        if exporter.exportViaPoints:
                            exporter.traverseViaPoints()

                        # build sdf root node
                        exporter.root = ET.Element("sdf", version="1.6")
                        exporter.model = ET.Element("model", name=exporter.modelName)
                        exporter.root.append(exporter.model)

                        allRigidGroups = exporter.getAllRigidGroups()

                        # exports all rigid groups to STL and SDF
                        names = []
                        progressDialog0 = exporter.app.userInterface.createProgressDialog()
                        progressDialog0.isBackgroundTranslucent = False
                        progressDialog0.show("SDFusion", 'Processing rigid groups: %v/%m', 0, len(allRigidGroups), 1)

                        for rig in allRigidGroups:
                            progressDialog0.progressValue += 1
                            if rig is not None and rig.name[:6] == "EXPORT":
                                name = rig.name[7:] # get rid of EXPORT_ tag
                                if name in names: # ignoring duplicate export
                                    exporter.logfile.write("WARNING: ignoring duplicate export of " + name + ", check your model for duplicate EXPORT Rigid Groups\n")
                                    continue
                                progressDialog0.message = "%v/%m " + name
                                exporter.getAllBodiesInRigidGroup(name,rig)
                                if not exporter.copyBodiesToNewComponentAndExport(name):
                                    return
                            if progressDialog0.wasCancelled:
                                progressDialog0.hide()
                                return

                        adsk.doEvents()

                        exporter.exportJointsToSDF()
                        if exporter.exportViaPoints:
                            exporter.exportViaPointsToSDF()

                            if exporter.exportCASPR: # exporting caspr only makes sense if we export viaPoints aswell
                                exporter.exportCASPRcables()
                                exporter.exportCASPRbodies()

                            if exporter.exportCardsflow:
                                exporter.exportToCardsflow()

                        if exporter.exportLighthouseSensors:
                            exporter.exportLighthouseSensorsToYAML()

                        progressDialog0.hide()
                        adsk.doEvents()

                        exporter.finish()

                except:
                    exporter.finish()
                    if exporter.ui:
                        exporter.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
            # adsk.terminate()
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

    def preserveViaPoints(self):
        global allVP
        for vp in allVP:
            muscle = vp.motor
            link = vp.link
            number = vp.number
            edge = vp.edge
            conPoints = rootComp.constructionPoints
            # Create construction point input
            pointInput = conPoints.createInput()
            # Create construction point by center
            pointInput.setByCenter(edge)
            point = conPoints.add(pointInput)
            point.name = "VP_motor" + muscle + "_" + link + "_" + number
            adsk.doEvents()

class SDFusionCreatedHandler(adsk.core.CommandCreatedEventHandler):
    ui = None
    app = None
    def __init__(self):
        super().__init__()
        self.app = adsk.core.Application.get()
        self.ui = self.app.userInterface
    def notify(self, args):
        try:

            cmd = args.command
            cmd.okButtonText = "Export to SDF"
             # Connect to the input changed event.

            onInputChanged = SDFusionInputChangedHandler()
            cmd.inputChanged.add(onInputChanged)
            handlers.append(onInputChanged)

            onDestroy = SDFusionDestroyHandler()
            cmd.destroy.add(onDestroy)

            # Keep the handler referenced beyond this function
            handlers.append(onDestroy)
            inputs = cmd.commandInputs
            global commandId

            self.createFlagsTab(inputs)
            self.createViaPointsTab(inputs)

        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

    def createFlagsTab(self, inputs):
        tabCmdInput1 = inputs.addTabCommandInput(commandId + '_tab_1', 'SDFusion')
        tab1ChildInputs = tabCmdInput1.children

        tab1ChildInputs.addStringValueInput(commandId + '_model_name', 'Model Name:', '')  # self.rootComp.name)
        tab1ChildInputs.addBoolValueInput(commandId + '_updateRigidGroups', 'updateRigidGroups', True, '', False)
        tab1ChildInputs.addBoolValueInput(commandId + '_meshes', 'exportMeshes', True, '', True)
        tab1ChildInputs.addBoolValueInput(commandId + '_sdf', 'sdf', True, '', True)
        tab1ChildInputs.addBoolValueInput(commandId + '_viapoints', 'viapoints', True, '', True)
        tab1ChildInputs.addBoolValueInput(commandId + '_caspr', 'caspr', True, '', False)
        tab1ChildInputs.addBoolValueInput(commandId + '_cardsflow', 'CARDSflow', True, '', True)
        tab1ChildInputs.addBoolValueInput(commandId + '_opensim', 'opensim', True, '', False)
        tab1ChildInputs.addBoolValueInput(commandId + '_darkroom', 'darkroom', True, '', False)
        tab1ChildInputs.addBoolValueInput(commandId + '_remove_small_parts', 'remove parts smaller 1g', True, '', False)
        tab1ChildInputs.addBoolValueInput(commandId + '_self_collide', 'self_collide', True, '', False)
        tab1ChildInputs.addBoolValueInput(commandId + '_dummy_inertia', 'dummy_inertia', True, '', False)
        # tab1ChildInputs.addBoolValueInput(commandId + '_cache', 'cache', True, '', True)

    def createViaPointsTab(self, inputs):
        tabCmdInput2 = inputs.addTabCommandInput(commandId + '_tab_2', 'ViaPoints')
        # Get the CommandInputs object associated with the parent command.
        cmdInputs = adsk.core.CommandInputs.cast(tabCmdInput2.children)
        muscleInput = cmdInputs.addStringValueInput('muscle', 'Myomuscle Number [Int]', '0')
        numberInput = cmdInputs.addStringValueInput('number', 'Via-Point Number [Int]', '0')
        linkInput = cmdInputs.addDropDownCommandInput('link', 'Link Name',
                                                      adsk.core.DropDownStyles.LabeledIconDropDownStyle)
        dropdownItems = linkInput.listItems
        # add a dropdown item for every link
        for lin in getRobotLinkNames(rootComp):
            dropdownItems.add(lin, False, '')

        selectionInput = cmdInputs.addSelectionInput('selection', 'Select', 'Select a circle for the via-point.')
        selectionInput.setSelectionLimits(1, 1)
        selectionInput.addSelectionFilter("CircularEdges")
        if (not tabCmdInput2.isActive):
            selectionInput.isEnabled = False


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        global commandId
        global commandName
        global commandDescription

        # Create command defintion
        cmdDef = ui.commandDefinitions.itemById(commandId)
        if not cmdDef:
            cmdDef = ui.commandDefinitions.addButtonDefinition(commandId, commandName, commandDescription)

        # Add command created event
        onCommandCreated = SDFusionCreatedHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        # Keep the handler referenced beyond this function

        handlers.append(onCommandCreated)

        # Execute command
        cmdDef.execute()

        # Prevent this module from being terminate when the script returns, because we are waiting for event handlers to fire
        adsk.autoTerminate(False)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))