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
#import os.path
import xml.etree.ElementTree as ET
import math
import xml.dom.minidom as DOM
import os, errno
from collections import defaultdict
from math import sqrt

commandId = 'SDFusionExporter'
commandName = 'SDFusion'
commandDescription = 'cardsflow exporter'

# Global set of event handlers to keep them referenced for the duration of the command
handlers = []

model_name = None
meshes = None
sdf = None
viapoints = None
opensim = None
caspr = None
darkroom = None
remove_small_parts = None
self_collide = None
dummy_inertia = None
cache = None

## global variable to keep track of how many via points are created
numberViaPoints = 0
## global variable to specify the links that can be choosen
links = []

allVP = []

rootComp = None

def getLinkNames():
    # Get all links of the robot (all rigid groups)
    # get active design
    global app
    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    # get root component in this design
    global rootComp
    rootComp = design.rootComponent
    # get all rigid groups of the root component
    links = []
    allRigidGroups = rootComp.allRigidGroups
    for rig in allRigidGroups:
        if rig is not None and rig.name[:6] == "EXPORT":
            links.append(rig.name)
    return links

class MyViaPoint():
    motor = ''
    number = ''
    link = ''
    edge =  None

# Event handler that reacts to any changes the user makes to any of the command inputs.
class MyCommandInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:      
            command = args.firingEvent.sender
            inputs = command.commandInputs
            global model_name
            global meshes
            global sdf
            global viapoints
            global opensim
            global caspr
            global darkroom
            global remove_small_parts
            global self_collide
            global dummy_inertia
            global cache

            # We need access to the inputs within a command during the execute.
            tabCmdInput1 = inputs.itemById(commandId + '_tab_1')
            tab1ChildInputs = tabCmdInput1.children
            model_name = tab1ChildInputs.itemById(commandId + '_model_name')
            meshes = tab1ChildInputs.itemById(commandId + '_meshes')
            sdf = tab1ChildInputs.itemById(commandId + '_sdf')
            viapoints = tab1ChildInputs.itemById(commandId + '_viapoints')
            opensim = tab1ChildInputs.itemById(commandId + '_opensim')
            caspr = tab1ChildInputs.itemById(commandId + '_caspr')
            darkroom = tab1ChildInputs.itemById(commandId + '_darkroom')
            remove_small_parts = tab1ChildInputs.itemById(commandId + '_remove_small_parts')            
            self_collide = tab1ChildInputs.itemById(commandId + '_self_collide')          
            dummy_inertia = tab1ChildInputs.itemById(commandId + '_dummy_inertia')   
            cache = tab1ChildInputs.itemById(commandId + '_cache')   
            
            eventArgs = adsk.core.InputChangedEventArgs.cast(args)
            inputs = eventArgs.inputs
            cmdInput = eventArgs.input
            if cmdInput.id == 'selection0' and cmdInput.id != 'number0' :
                muscleInput = inputs.itemById('muscle0')
                muscle = muscleInput.value
                selInput = inputs.itemById('selection0')
                selection  = selInput.selection(0)
                entity =  selection.entity
                edge = adsk.fusion.BRepEdge.cast(entity)
                numberInput = inputs.itemById('number0')
                number = numberInput.value
                # get link name
                linkInput = inputs.itemById('link0').selectedItem
                link = '?'
                if linkInput:
                    link = linkInput.name

                global rootComp
                conPoints = rootComp.constructionPoints
                # Create construction point input
                pointInput = conPoints.createInput()
                # Create construction point by center
                pointInput.setByCenter(edge)
                point = conPoints.add(pointInput)
                point.name = "VP_motor"+ muscle + "_" + link + "_" + number

                vp = MyViaPoint()
                vp.motor =  muscle
                vp.link = link
                vp.number = number
                vp.edge =  edge

                global allVP
                allVP.append(vp)

                # automatically increase VP number by 1
                numberInput.value = str(int(number) + 1)
                
        except:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
                        
class MyCommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            # When the command is done, terminate the script
            # This will release all globals which will remove all event handlers
            global allVP
            for vp in allVP:
                muscle = vp.motor
                link = vp.link
                number = vp.number
                edge =  vp.edge
                global rootComp
                conPoints = rootComp.constructionPoints
                # Create construction point input
                pointInput = conPoints.createInput()
                # Create construction point by center
                pointInput.setByCenter(edge)
                point = conPoints.add(pointInput)
                point.name = "VP_motor"+ muscle + "_" + link + "_" + number
                adsk.doEvents()
            # When the command is done, terminate the script
            # This will release all globals which will remove all event handlers   
            global model_name
            global sdf
            global meshes
            global viapoints
            global opensim
            global caspr
            global darkroom
            global remove_small_parts
            global self_collide
            global dummy_inertia
            global cache
            returnvalue = adsk.core.Application.get().userInterface.messageBox("for real?", "export?", 3)    
            if returnvalue == 2:
                try:
                    exporter = SDFExporter()
                    if remove_small_parts is not None:
                        exporter.runCleanUp = remove_small_parts.value
                    if meshes is not None:
                        exporter.exportMeshes = meshes.value
                    if viapoints is not None:
                        exporter.exportViaPoints = viapoints.value
                    if caspr is not None:
                        exporter.exportCASPR = caspr.value
                    if opensim is not None:
                        exporter.exportOpenSimMuscles = opensim.value
                    if darkroom is not None:
                        exporter.exportLighthouseSensors = darkroom.value
                    if model_name is not None:
                        exporter.modelName = model_name.value
                    if self_collide is not None:
                        exporter.self_collide = self_collide.value
                    if dummy_inertia is not None:
                        exporter.dummy_inertia = dummy_inertia.value
                    if cache is not None:
                        exporter.cache = cache.value
                    if exporter.askForExportDirectory():
                        exporter.createDiectoryStructure()
                
                        if exporter.runCleanUp:
                            allComponents = exporter.design.allComponents
                            progressDialog = exporter.app.userInterface.createProgressDialog()
                            progressDialog.isBackgroundTranslucent = False
                            progressDialog.show("Clean up", 'Looking for small components', 0, len(allComponents), 0)
                            for component in allComponents:
                                progressDialog.progressValue += 1
                                if component.physicalProperties.mass < 0.001:
                                    for o in component.occurrences:
                                        progressDialog.message = "Removing " + component.name
                                        o.deleteMe()
                
                            progressDialog.hide()
                
                        # build sdf root node
                        exporter.root = ET.Element("sdf", version="1.6")
                        exporter.model = ET.Element("model", name=exporter.modelName)
                        exporter.root.append(exporter.model)
                
                        allRigidGroups = exporter.getAllRigidGroups()
                        
                        allComponents = exporter.design.allComponents
                        progressDialog0 = exporter.app.userInterface.createProgressDialog()
                        progressDialog0.isBackgroundTranslucent = False
                        progressDialog0.isCancelButtonShown = True
                        progressDialog0.show("Export Rigid Groups", 'Rigid Groups', 0, len(allRigidGroups), 1)
                        # exports all rigid groups to STL and SDF
                        names = []
                        for rig in allRigidGroups:
                            progressDialog0.progressValue += 1
                            if rig is not None and rig.name[:6] == "EXPORT":
                                name = rig.name[7:] # get rid of EXPORT_ tag
                                if name in names: # ignoring duplicate export
                                    exporter.logfile.write("WARNING: ignroing duplicate export of " + name + ", check your model for duplicate EXPORT Rigid Groups\n")
                                    continue
                                progressDialog0.message = "%v/%m " + name
                                exporter.getAllBodiesInRigidGroup(name,rig)
                                if exporter.copyBodiesToNewComponentAndExport(name) == False:
                                    return
                            if progressDialog0.wasCancelled:
                                    progressDialog0.hide()
                                    return
                        progressDialog0.hide()                                    
                                    
                        progressDialog1 = exporter.app.userInterface.createProgressDialog()
                        progressDialog1.isBackgroundTranslucent = False
                        progressDialog1.isCancelButtonShown = True
                        progressDialog1.show("Export Robot Desciptions", 'Robot Desciptions', 0, len(allComponents), 0)
                        exporter.exportJointsToSDF()
                        if exporter.exportViaPoints:
                            progressDialog1.message = "SDF"
                            exporter.exportViaPointsToSDF()
                            if progressDialog1.wasCancelled:
                                    progressDialog1.hide()
                                    return
                            if exporter.exportCASPR: # exporting caspr only makes sense if we export viaPoints aswell
                                progressDialog1.message = "CASPR"
                                exporter.exportCASPRcables()
                                exporter.exportCASPRbodies()
                                if progressDialog1.wasCancelled:
                                    progressDialog1.hide()
                                    return
                        if exporter.exportLighthouseSensors:
                            progressDialog1.message = "darkroom"
                            exporter.exportLighthouseSensorsToYAML()
                            
                        if progressDialog1.wasCancelled:
                            progressDialog1.hide()
                            return    
                        progressDialog1.hide()   
                        progressDialog1.message = "finishing"
                        exporter.finish()
                        progressDialog0.hide()
                        progressDialog1.hide()
                except:
                    exporter.finish()
                    if exporter.ui:
                        exporter.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
            adsk.terminate()
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
                
                
class MyCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    ui = None
    app = None
    product = None
    design = None
    exportMgr = None
    rootOcc = None
    def __init__(self):
        super().__init__()
        self.app = adsk.core.Application.get()
        self.ui  = self.app.userInterface
        # get active design
        self.product = self.app.activeProduct
        self.design = adsk.fusion.Design.cast(self.product)
        self.exportMgr = self.design.exportManager
        # get root component in this design
        self.rootComp = self.design.rootComponent
        global rootComp
        rootComp = self.rootComp
        # get all occurrences within the root component
        self.rootOcc = self.rootComp.occurrences
    def notify(self, args):
        try:
            cmd = args.command
            
             # Connect to the input changed event.           
            onInputChanged = MyCommandInputChangedHandler()
            cmd.inputChanged.add(onInputChanged)
            handlers.append(onInputChanged)
            
            onDestroy = MyCommandDestroyHandler()
            cmd.destroy.add(onDestroy)
            # Keep the handler referenced beyond this function
            handlers.append(onDestroy)
            inputs = cmd.commandInputs
            global commandId
            
            tabCmdInput1 = inputs.addTabCommandInput(commandId + '_tab_1', 'SDFusion')
            tab1ChildInputs = tabCmdInput1.children
            
            tab1ChildInputs.addStringValueInput(commandId + '_model_name', 'Model Name:', self.rootComp.name)
            tab1ChildInputs.addBoolValueInput(commandId + '_meshes', 'meshes', True, '', True)
            tab1ChildInputs.addBoolValueInput(commandId + '_sdf', 'sdf', True, '', True)
            tab1ChildInputs.addBoolValueInput(commandId + '_viapoints', 'viapoints', True, '', True)
            tab1ChildInputs.addBoolValueInput(commandId + '_caspr', 'caspr', True, '', False)
            tab1ChildInputs.addBoolValueInput(commandId + '_opensim', 'opensim', True, '', False)
            tab1ChildInputs.addBoolValueInput(commandId + '_darkroom', 'darkroom', True, '', False)
            tab1ChildInputs.addBoolValueInput(commandId + '_remove_small_parts', 'remove parts smaller 1g', True, '', False)
            tab1ChildInputs.addBoolValueInput(commandId + '_self_collide', 'self_collide', True, '', False)
            tab1ChildInputs.addBoolValueInput(commandId + '_dummy_inertia', 'dummy_inertia', True, '', False)
            tab1ChildInputs.addBoolValueInput(commandId + '_cache', 'cache', True, '', True)
            
            tabCmdInput2 = inputs.addTabCommandInput(commandId + '_tab_2', 'ViaPoints')
            # Get the CommandInputs object associated with the parent command.
            cmdInputs = adsk.core.CommandInputs.cast(tabCmdInput2.children)
            # add input for myomuscle number
            muscleInput = cmdInputs.addStringValueInput('muscle{}'.format(numberViaPoints), 'Myomuscle Number [Int]', '0')
            # add input for via point number
            numberInput = cmdInputs.addStringValueInput('number{}'.format(numberViaPoints), 'Via-Point Number [Int]', '0')
            # add input for link name
            linkInput =  cmdInputs.addDropDownCommandInput('link{}'.format(numberViaPoints), 'Link Name', adsk.core.DropDownStyles.LabeledIconDropDownStyle);
            dropdownItems = linkInput.listItems
            # add a dropdown item for every link
            global links
            for lin in links:
                dropdownItems.add(lin, False, '')
            # Create a selection input.
            selectionInput = cmdInputs.addSelectionInput('selection{}'.format(numberViaPoints), 'Select', 'Select a circle for the via-point.')
            selectionInput.setSelectionLimits(1,1)
            selectionInput.addSelectionFilter("CircularEdges")
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
                
def run(context):
    ui = None
    try:
        global app
        app = adsk.core.Application.get()
        global ui
        ui = app.userInterface

        global commandId
        global commandName
        global commandDescription
        
        # Create command defintion
        cmdDef = ui.commandDefinitions.itemById(commandId)
        if not cmdDef:
            cmdDef = ui.commandDefinitions.addButtonDefinition(commandId, commandName, commandDescription)
            
        # Add command created event
        onCommandCreated = MyCommandCreatedHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        # Keep the handler referenced beyond this function
        handlers.append(onCommandCreated)
        
        # Get all links the robot has
        global links
        links = getLinkNames()

        # Execute command
        cmdDef.execute()            

        # Prevent this module from being terminate when the script returns, because we are waiting for event handlers to fire
        adsk.autoTerminate(False)
        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
            
## A class to hold information about all muscles.
class Plugin:
    myoMuscles = []

## A class to hold information about all viaPoints of a muscle.
class MyoMuscle:
    number = ""
    viaPoints = []
    def __init__(self,num):
        self.number = num
        self.viaPoints = []

## A class to hold information about a viaPoint.
class ViaPoint:
    coordinates = ""
    link = ""
    number = ""
    global_coordinates = []
    
class VisualMarker:
    coordinates = ""
    link = ""

class SDFExporter():
    ui = None
    app = None
    product = None
    design = None
    exportMgr = None
    rootOcc = None
    rootComp = None

    fileDir = "E:/roboy_models"
    modelName = "TestCube"

    root = None
    model = None

    osimroot = None

    logfile = "log.txt"

    numberOfRigidGroupsToExport = 0

    progressDialog = None

    numberOfBodies = defaultdict()
    bodies = defaultdict(list)
    joints = defaultdict(list)
    COM = defaultdict(adsk.core.Point3D)
    totalMass = defaultdict()
    inertias = defaultdict(list)
    number_of_coms = defaultdict()
    calculateCOMFlag = True

    transformMatrices = defaultdict(adsk.core.Matrix3D)

    runCleanUp = False
    exportMeshes = True
    exportViaPoints = False
    exportLighthouseSensors = False
    exportCASPR = False
    exportOpenSimMuscles = False
    self_collide = False
    dummy_inertia = False
    cache = False

    ## Global variable to specify the file name of the plugin loaded by the SDF.
    # Only necessary if **exportViaPoints** is **True**.
    pluginFileName = "libmusc_plugin.so"

    ## Global variable to specify the name of the plugin loaded by the SDF-
    # Only necessary if **exportViaPoints** id **True**.
    pluginName = "musc_plugin"
    pluginObj = Plugin()

    def __init__(self):
        self.app = adsk.core.Application.get()
        self.ui  = self.app.userInterface
        # get active design
        self.product = self.app.activeProduct
        self.design = adsk.fusion.Design.cast(self.product)
        self.exportMgr = self.design.exportManager
        # get root component in this design
        self.rootComp = self.design.rootComponent
        # get all occurrences within the root component
        self.rootOcc = self.rootComp.occurrences

    def askForExportMeshes(self):
        returnvalue = self.ui.messageBox("Do you want to export meshes to STL?", "export options", 3)
        if returnvalue == 2:
            self.exportMeshes = True
        elif returnvalue == 3:
            self.exportMeshes = False
        else:
            return False
        return True

    def askForExportViaPoints(self):
        returnvalue = self.ui.messageBox("Do you want me to export ViaPoints?", "export options", 3)
        if returnvalue == 2:
            self.exportViaPoints = True
        elif returnvalue == 3:
            self.exportViaPoints = False
        else:
            return False
        return True

    def askForExportOsimMuscles(self):
        returnvalue = self.ui.messageBox("Do you want me to export OpenSim muscles?", "export options", 3)
        if returnvalue == 2:
            self.exportOpenSimMuscles = True
        elif returnvalue == 3:
            self.exportOpenSimMuscles = False
        else:
            return False
        return True

    def askForExportLighthouseSensors(self):
        returnvalue = self.ui.messageBox("Do you want me to export DarkRoomSensors aswell?", "export options", 3)
        if returnvalue == 2: #yes
            self.exportLighthouseSensors = True
        elif returnvalue == 3:
            self.exportLighthouseSensors = False
        else:
            return False
        return True

    def askForExportCASPR(self):
        returnvalue = self.ui.messageBox("Do you want me to export CASPR aswell?", "export options", 3)
        if returnvalue == 2: #yes
            self.exportCASPR = True
        elif returnvalue == 3:
            self.exportCASPR = False
        else:
            return False
        return True

    def askForExportDirectory(self):
        fileDialog = self.ui.createFileDialog()
        fileDialog.isMultiSelectEnabled = False
        fileDialog.title = "Specify result directory"
        fileDialog.filter = 'directory (*/*)'
        fileDialog.filterIndex = 0
        fileDialog.initialDirectory = self.fileDir
        fileDialog.initialFilename = self.modelName
        dialogResult = fileDialog.showSave()
        if dialogResult == adsk.core.DialogResults.DialogOK:
            self.fileDir = fileDialog.filename
        else:
            return False
        return True
    def createDiectoryStructure(self):
        try:
            os.makedirs(self.fileDir)
            os.makedirs(self.fileDir+'/meshes')
            os.makedirs(self.fileDir+'/meshes/CAD')
            if self.exportLighthouseSensors:
                os.makedirs(self.fileDir+'/lighthouseSensors')
            if self.exportCASPR:
                os.makedirs(self.fileDir+'/caspr')
            self.logfile = open(self.fileDir+'/logfile.txt', 'w')
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise
    def getAllRigidGroups(self):
        allRigidGroups = self.rootComp.allRigidGroups
        self.numberOfRigidGroupsToExport = 0
        for rig in allRigidGroups:
            if rig is not None and rig.name[:6] == "EXPORT":
                self.numberOfRigidGroupsToExport = self.numberOfRigidGroupsToExport+1        
        return allRigidGroups

    def getAllBodiesInRigidGroup(self, name, rigidGroup):
        self.numberOfBodies[name] = 0
        self.COM[name] = adsk.core.Point3D.create(0,0,0)
        self.number_of_coms[name] = 0
        self.totalMass[name] = 0
        self.logfile.write("mass[kg] \t COM[cm] \t\t\t component name\n")
        self.getBodies(name,rigidGroup.occurrences,0)
#        self.getCOM(name,rigidGroup.occurrences)
#        if self.calculateCOMFlag == True:
#            # calculate COM -> dividing by totalMass
#            scaledCOM = self.COM[name].asVector()
#            scaledCOM.scaleBy(1/self.totalMass[name])
#            self.COM[name] = scaledCOM.asPoint()
#        self.logfile.write("----------------------------------------------------------------\n")
#        self.logfile.write("Mass of %s: %f\n" % (name, self.totalMass[name]))
#        self.logfile.write("COM of %s: %f %f %f\n" % (name, self.COM[name].x, self.COM[name].y, self.COM[name].z))
#        self.logfile.write("----------------------------------------------------------------\n")

    def getBodies(self, name, occurrences, currentLevel):
        for occurrence in occurrences:
            self.numberOfBodies[name] = self.numberOfBodies[name] + occurrence.component.bRepBodies.count
            for body in occurrence.component.bRepBodies:
                self.bodies[name].append(body)
            if occurrence.childOccurrences:
                self.getBodies(name,occurrence.childOccurrences,currentLevel+1)
    def getCOM(self, name, occurrences):
        # check if a COM point is defined
        self.logfile.write("COM: " + name + "\n")
        allComponents = self.design.allComponents
        self.calculateCOMFlag = True
        for com in allComponents:
            if com is not None:
                allConstructionPoints = com.constructionPoints
                for point in allConstructionPoints:
                    if point is not None:
                        if point.name[:3] == "COM" and point.name[4:] == name:
                            self.COM[name] = adsk.core.Point3D.create(point.geometry.x,point.geometry.y,point.geometry.z)
                            self.calculateCOMFlag = False
        # if not, we calculate it
        self.calculateCOM(name, occurrences, 0)
        return False

    def calculateCOM(self, name, occurrences, currentLevel):
        for occurrence in occurrences:
            physicalProperties = occurrence.component.getPhysicalProperties()
            self.totalMass[name] = self.totalMass[name] + physicalProperties.mass
            if self.calculateCOMFlag == True:
                self.number_of_coms[name] = self.number_of_coms[name] + 1
                centerOfMass = physicalProperties.centerOfMass
                #print(centerOfMass.asArray())
                centerOfMass.transformBy(occurrence.transform)
                #print(centerOfMass.asArray())
                centerOfMass = centerOfMass.asVector()
                centerOfMass.scaleBy(physicalProperties.mass)
                self.COM[name].translateBy(centerOfMass)

                self.logfile.write("%f \t %f %f %f \t\t %s\n" % (physicalProperties.mass, centerOfMass.x, centerOfMass.y, centerOfMass.z, occurrence.name))
            if occurrence.childOccurrences:
                self.calculateCOM(name,occurrence.childOccurrences,currentLevel+1)

    def copyBodiesToNewComponentAndExport(self, name):
        self.logfile.write("Body: " + name + "\n")
        transformMatrix = adsk.core.Matrix3D.create()

        new_occurence = None

        if self.cache:  
            self.logfile.write("looking for cached " + name + "\n")
            new_occurence = self.rootOcc.itemByName("EXPORT_" + name + ":1")      
        
        if new_occurence is None: # if not exported yet
            self.logfile.write(name + "not found, copying bodies to new component\n")
            # global new_component
            temp_occurence = self.rootOcc.addNewComponent(transformMatrix)
            temp_occurence.component.name = "TEMP_EXPORT_" + name
            group = [g for g in self.rootComp.allRigidGroups if g.name == "EXPORT_"+name][0]
            i = 0
            for occurrence in group.occurrences:
                for b in occurrence.bRepBodies:
                    new_body = b.copyToComponent(temp_occurence)
                    new_body.name = 'body'+str(i)
                    i = i+1                    
                    
            physicalProperties = temp_occurence.component.getPhysicalProperties()
            calculateCOM = True
            for occurrence in self.rootOcc:
                com = occurrence.component
                if com is not None:
                    allConstructionPoints = com.constructionPoints
                    for point in allConstructionPoints:
                        if point is not None:
                            if point.name[:3] == "COM" and point.name[4:] == name:
                                calculateCOM = False
                                self.COM[name] = adsk.core.Point3D.create(point.geometry.x,point.geometry.y,point.geometry.z)
                                self.COM[name].transformBy(occurrence.transform)
                                
            if calculateCOM == True:
                centerOfMass = physicalProperties.centerOfMass
                centerOfMass = centerOfMass.asVector()
                #centerOfMass.scaleBy(physicalProperties.mass)
                self.COM[name] = centerOfMass
                
            self.totalMass[name] = physicalProperties.mass
            transformMatrix = temp_occurence.transform
            transformMatrix.translation = adsk.core.Vector3D.create( self.COM[name].x, self.COM[name].y, self.COM[name].z)
            #temp_occurence.transform = transformMatrix
            self.transformMatrices[name] = transformMatrix
            adsk.doEvents() 
            new_occurence = self.rootOcc.addNewComponent(transformMatrix)
            new_occurence.component.name = "EXPORT_" + name
            group = [g for g in self.rootComp.allRigidGroups if g.name == "EXPORT_"+name][0]
            i = 0
            for occurrence in group.occurrences:
                for b in occurrence.bRepBodies:
                    new_body = b.copyToComponent(new_occurence)
                    new_body.name = 'body'+str(i)
                    i = i+1    

        if self.exportMeshes:
            self.logfile.write("exporting stl of " + name + "\n")
            self.exportToStl(new_occurence, name)

        link = self.linkSDF(new_occurence, name)
        self.model.append(link)
        
        temp_occurence.deleteMe()
        # delete the temporary new occurrence
        if not self.cache:
            new_occurence.deleteMe()
        # Call doEvents to give Fusion a chance to react.
        adsk.doEvents()

        return True

    def exportJointsToSDF(self):
        progressDialog = self.app.userInterface.createProgressDialog()
        progressDialog.isBackgroundTranslucent = False

        #get all joints of the design
        allComponents = self.design.allComponents
        allRigidGroups = self.rootComp.allRigidGroups
        for com in allComponents:
            if com is not None:
                allJoints = com.joints
                
                for joi in allJoints:
                    if joi is not None and joi.name[:6] == "EXPORT":
                        progressDialog.show("Joint", "Processing joints", 0, 30, 0)
                        progressDialog.progressValue += 1
                        progressDialog.message = joi.name
                        self.logfile.write("Joint: " + joi.name + "\n")
                        one = joi.occurrenceOne
                        two = joi.occurrenceTwo
                        if one is not None and two is not None:
                            name_parent = None
                            name_child = None
                            for rig in allRigidGroups:
                                if rig is not None and rig.name[:6] == "EXPORT":
                                    value_child = rig.occurrences.itemByName(one.name)
                                    value_parent = rig.occurrences.itemByName(two.name)
                                    if value_parent is not None:
                                        name_parent = rig.name[7:]
                                    if value_child is not None:
                                        name_child = rig.name[7:]
                            if name_parent is not None and name_child is not None:
                                self.logfile.write("\tparent: " + name_parent + "\n")
                                self.logfile.write("\tchild: " + name_child + "\n")
                                matrix = self.transformMatrices[name_child]
                                print(one.transform.translation.asArray())
                                self.joints[name_child] = (name_parent,joi)
        #                        transformMatrix.transformBy(one.transform)
                                #print(matrix.asArray())
                                joint = self.jointSDF(joi, name_parent, name_child, matrix)
                                self.model.append(joint)
                            else:
                                self.logfile.write("\tERROR writing joint " + joi.name + ", check your rigid EXPORT groups, the parent/child link must be part of a rigid Export group!\n")
                        progressDialog.hide()

    def exportViaPointsToSDF(self):
        allComponents = self.design.allComponents
        sketches = self.rootComp.sketches
        xyPlane = self.rootComp.xYConstructionPlane
        sketch = sketches.add(xyPlane)
        points = {}
        EEs = []
        VMs = []
        construction_point_names = []
        for occurrence in self.rootOcc:
            com = occurrence.component
            try:
                if com is not None:
                    allConstructionPoints = com.constructionPoints
                    for point in allConstructionPoints:
                        if point is not None:
                            viaPoint = ViaPoint()
                            construction_point_names.append(point.name[:2])
                            if point.name[:2] == "VP":
                                viaPointInfo = point.name.split("_")
                                vec = adsk.core.Point3D.create(point.geometry.x,point.geometry.y,point.geometry.z)
                                vec.transformBy(occurrence.transform)
                                viaPoint.global_coordinates = [vec.x,vec.y,vec.z]
                                linkname = '_'.join(viaPointInfo[3:-1])
                                origin = self.transformMatrices[linkname].translation
                                origin = origin.asPoint()
                                dist = origin.vectorTo(vec)
                                viaPoint.coordinates = str(dist.x*0.01) + " " + str(dist.y*0.01) + " " + str(dist.z*0.01)
                                viaPoint.link = linkname
                                viaPoint.number = viaPointInfo[-1:]
                                myoNumber = viaPointInfo[1][5:]
                                myoMuscleList = list(filter(lambda x: x.number == myoNumber, self.pluginObj.myoMuscles))
                                if not myoMuscleList:
                                    myoMuscle = MyoMuscle(myoNumber)
                                    myoMuscle.viaPoints.append(viaPoint)
                                    self.pluginObj.myoMuscles.append(myoMuscle)
                                if myoMuscleList:
                                    myoMuscleList[0].viaPoints.append(viaPoint)

                                if myoNumber not in points:
                                    points[myoNumber] = adsk.core.ObjectCollection.create() 
                                points[myoNumber].add(vec)
                            if point.name[:2] == "EE":
                                eeInfo = point.name.split("_")
                                vec = adsk.core.Point3D.create(point.geometry.x,point.geometry.y,point.geometry.z)
                                viaPoint.global_coordinates = [point.geometry.x,point.geometry.y,point.geometry.z]
                                linkname = '_'.join(eeInfo[1:])
                                origin = self.transformMatrices[linkname].translation
                                origin = origin.asPoint()
                                dist = origin.vectorTo(vec)
                                ee = VisualMarker()
                                ee.coordinates = str(dist.x*0.01) + " " + str(dist.y*0.01) + " " + str(dist.z*0.01)
                                ee.link = linkname
                                EEs.append(ee)
                            if point.name[:2] == "VM":
                                vmInfo = point.name.split("_")
                                vec = adsk.core.Point3D.create(point.geometry.x,point.geometry.y,point.geometry.z)
                                viaPoint.global_coordinates = [point.geometry.x,point.geometry.y,point.geometry.z]
                                linkname = '_'.join(vmInfo[1:])
                                origin = self.transformMatrices[linkname].translation
                                origin = origin.asPoint()
                                dist = origin.vectorTo(vec)
                                vm = VisualMarker()
                                vm.coordinates = str(dist.x*0.01) + " " + str(dist.y*0.01) + " " + str(dist.z*0.01)
                                vm.link = linkname
                                VMs.append(vm)
                                    
            except:
                   self.ui.messageBox("Exception in " + point.name + '\n' +traceback.format_exc())
                   pass
        for com in allComponents:
                try:
                    if com is not None:
                        allConstructionPoints = com.constructionPoints
                        for point in allConstructionPoints:
                            if point is not None:
                                if point.name[:2] not in construction_point_names:
                                    viaPoint = ViaPoint()
                                    if point.name[:2] == "VP":
                                        viaPointInfo = point.name.split("_")
                                        vec = adsk.core.Point3D.create(point.geometry.x,point.geometry.y,point.geometry.z)
                                        viaPoint.global_coordinates = [point.geometry.x,point.geometry.y,point.geometry.z]
                                        linkname = '_'.join(viaPointInfo[3:-1])
                                        origin = self.transformMatrices[linkname].translation
                                        origin = origin.asPoint()
                                        dist = origin.vectorTo(vec)
                                        viaPoint.coordinates = str(dist.x*0.01) + " " + str(dist.y*0.01) + " " + str(dist.z*0.01)
                                        viaPoint.link = linkname
                                        viaPoint.number = viaPointInfo[-1:]
                                        myoNumber = viaPointInfo[1][5:]
                                        myoMuscleList = list(filter(lambda x: x.number == myoNumber, self.pluginObj.myoMuscles))
                                        if not myoMuscleList:
                                            myoMuscle = MyoMuscle(myoNumber)
                                            myoMuscle.viaPoints.append(viaPoint)
                                            self.pluginObj.myoMuscles.append(myoMuscle)
                                        if myoMuscleList:
                                            myoMuscleList[0].viaPoints.append(viaPoint)
        
                                        if myoNumber not in points:
                                            points[myoNumber] = adsk.core.ObjectCollection.create() 
                                        points[myoNumber].add(vec)
                                    if point.name[:2] == "EE":
                                        eeInfo = point.name.split("_")
                                        vec = adsk.core.Point3D.create(point.geometry.x,point.geometry.y,point.geometry.z)
                                        viaPoint.global_coordinates = [point.geometry.x,point.geometry.y,point.geometry.z]
                                        linkname = '_'.join(eeInfo[1:])
                                        origin = self.transformMatrices[linkname].translation
                                        origin = origin.asPoint()
                                        dist = origin.vectorTo(vec)
                                        ee = VisualMarker()
                                        ee.coordinates = str(dist.x*0.01) + " " + str(dist.y*0.01) + " " + str(dist.z*0.01)
                                        ee.link = linkname
                                        EEs.append(ee)
                                    if point.name[:2] == "VM":
                                        vmInfo = point.name.split("_")
                                        vec = adsk.core.Point3D.create(point.geometry.x,point.geometry.y,point.geometry.z)
                                        viaPoint.global_coordinates = [point.geometry.x,point.geometry.y,point.geometry.z]
                                        linkname = '_'.join(vmInfo[1:])
                                        origin = self.transformMatrices[linkname].translation
                                        origin = origin.asPoint()
                                        dist = origin.vectorTo(vec)
                                        vm = VisualMarker()
                                        vm.coordinates = str(dist.x*0.01) + " " + str(dist.y*0.01) + " " + str(dist.z*0.01)
                                        vm.link = linkname
                                        VMs.append(vm)
                                    
                except:
                       self.ui.messageBox("Exception in " + point.name + '\n' +traceback.format_exc())
                       pass

        for pointSet in points.values():
            sketch.sketchCurves.sketchFittedSplines.add(pointSet)

        plugin = ET.Element("plugin", filename=self.pluginFileName, name=self.pluginName)
        if (not self.exportOpenSimMuscles):
            self.model.append(plugin)
        allMyoMuscles = self.pluginObj.myoMuscles
        #allMyoMuscles.sort(key=lambda x: x.number)
        # create myoMuscle nodes
        for myo in allMyoMuscles:
            myoMuscle = ET.Element("myoMuscle", name="motor"+myo.number)
            plugin.append(myoMuscle)
            allViaPoints = myo.viaPoints
            allViaPoints.sort(key=lambda x: x.number)
            link = ET.Element("link", name="default")
            # create viaPoint nodes as children of links
            for via in allViaPoints:
                if link.get("name") != via.link:
                    link = ET.Element("link", name=via.link)
                    myoMuscle.append(link)
                # TODO: export more types of viaPoints
                viaPoint = ET.Element("viaPoint", type="FIXPOINT")
                # TODO: rotate global coordinates into link frame coordinates
                viaPoint.text=via.coordinates
                link.append(viaPoint)
        i = 0
        for ee in EEs:
            endeffector = ET.Element("endEffector", name="endeffector"+str(i), link=ee.link)
            i = i+1
            endeffector.text = ee.coordinates
            plugin.append(endeffector)
        for vm in VMs:
            marker = ET.Element("marker", link=vm.link)
            marker.text = vm.coordinates
            plugin.append(marker)
        if (self.exportOpenSimMuscles):
            osimPlugin = ET.Element("plugin", filename="libgazebo_ros_muscle_interface.so", name="muscle_interface_plugin")
            self.model.append(osimPlugin)
            osimMuscles = ET.Element("muscles")
            osimMuscles.text = "model://"+self.model.get("name")+"/muscles.osim"
            self.model.append(osimMuscles)
            self.osimroot = ET.Element("OpenSimDocument", Version="30000")
            model = ET.Element("Model")
            forceSet = ET.Element("ForceSet")
            objects = ET.Element("objects")
            forceSet.append(objects)
            model.append(forceSet)
            self.osimroot.append(model)

            for myo in allMyoMuscles:
                # TODO add bodies
                muscle = ET.Element("Thelen2003Muscle", name="muscle" + myo.number)
                objects.append(muscle)
                gPath = ET.Element("GeometryPath")
                ppSet = ET.Element("PathPointSet")
                ppSetObjects = ET.Element("objects")
                ppSet.append(ppSetObjects)
                muscle.append(gPath)
                gPath.append(ppSet)
                pwSet = ET.Element("PathWrapSet")
                pwObjects = ET.Element("objects")
                pwSet.append(pwObjects)
                # pWrap = ET.Element("PathWrap",name="PathWrap_"+myo.number)
                # wrap_object = ET.Element("wrap_object") # TODO fix objects in pWrap
                # wrap_object.text = "WrapJoint"
                # pWrap.append(wrap_object)
                # method = ET.Element("method")
                # method.text = "midpoint"
                # pWrap.append(method)
                # pwObjects.append(pWrap)
                gPath.append(pwSet)
                max_isometric_force = ET.Element("max_isometric_force")
                max_isometric_force.text=str(1000.0)
                optimal_fiber_length = ET.Element("optimal_fiber_length")
                optimal_fiber_length.text = str(0.14108090847730637)  # TODO calculate optiomal fiber length
                tendon_slack_length = ET.Element("tendon_slack_length")

                stiffness = ET.Element("stiffness")
                stiffness.text = str(100000)
                dissipation = ET.Element("dissipation")
                dissipation.text = str(1)
                muscle.append(max_isometric_force)
                muscle.append(optimal_fiber_length)
                muscle.append(tendon_slack_length)
                muscle.append(stiffness)
                muscle.append(dissipation)

                allViaPoints = myo.viaPoints
                allViaPoints.sort(key=lambda x: x.number)

                dist = 0
                for i in range(len(allViaPoints)-1):
                    squared_dist = (allViaPoints[i].global_coordinates[0] - allViaPoints[i+1].global_coordinates[0])**2 + (allViaPoints[i].global_coordinates[1] - allViaPoints[i+1].global_coordinates[1])**2 + (allViaPoints[i].global_coordinates[2] - allViaPoints[i+1].global_coordinates[2])**2
                    # np.sum(allViaPoints[i].global_coordinates**2 + allViaPoints[i+1].global_coordinates**2, axis=0)
                    dist += sqrt(squared_dist)
                tendon_slack_length.text = str(dist)

                for point in allViaPoints:
                    pathPoint = ET.Element("PathPoint", name=muscle.get("name")+"_node"+point.number[0])
                    location = ET.Element("location")
                    location.text = point.coordinates
                    body = ET.Element("body")
                    body.text = point.link
                    pathPoint.append(location)
                    pathPoint.append(body)
                    ppSetObjects.append(pathPoint)
            bodySet = ET.Element("BodySet", name="")
            bodySetObjects = ET.Element("objects")
            bodySet.append(bodySetObjects)
            model.append(bodySet)


    def traverseViaPoints(self):
        allComponents = self.design.allComponents
        for com in allComponents:
            if com is not None:
                allConstructionPoints = com.constructionPoints
                for point in allConstructionPoints:
                    progressDialog = self.app.userInterface.createProgressDialog()
                    progressDialog.isBackgroundTranslucent = False
                    progressDialog.show("Traversing viaPoints", 'Checking', 0, len(allConstructionPoints), 0)
                    if point is not None:
                        if point.name[:2] == "VP":
                            progressDialog.message = point.name
                            progressDialog.progressValue += 1
                            viaPointInfo = point.name.split("_")
                            viaPoint = ViaPoint()
                            vec = adsk.core.Point3D.create(point.geometry.x,point.geometry.y,point.geometry.z)
                            viaPoint.global_coordinates = [point.geometry.x,point.geometry.y,point.geometry.z]

    def exportLighthouseSensorsToYAML(self):
        #get all joints of the design
        allComponents = self.design.allComponents
        DarkRoomSensors = defaultdict(list)
        for com in allComponents:
            if com is not None:
                allConstructionPoints = com.constructionPoints
                for point in allConstructionPoints:
                    if point.name[:2] == "LS":
                        names = point.name.split('_')
                        linkname = "_".join(names[1:-1])
                        sensor_position = adsk.core.Point3D.create(point.geometry.x,point.geometry.y,point.geometry.z)
                        # the lighthouse sensors shall be relative to COM, such that the pose estimation returns a pose for the COM
                        vec = self.COM[linkname].vectorTo(sensor_position)
                        DarkRoomSensors[linkname].append(vec)
        objectID = 0
        for name,sensors in DarkRoomSensors.items():
            file = open(self.fileDir+'/lighthouseSensors/' + name+'.yaml', 'w')
            file.write('name: ' + name + '\n');
            file.write('ObjectID: ' + str(objectID) + '\n');
            file.write('mesh: ' + "../meshes/CAD/" + name + '.stl\n');
            file.write('sensor_relative_locations:\n');
            i = 0
            for point in sensors:
                line = '- [' + str (i) + ', ' + str(point.x*0.01) + ', ' + str(point.y*0.01) + ', ' + str(point.z*0.01) + ']\n'
                file.write(line);
                i = i+1
            file.close()
            objectID = objectID + 1

    def finish(self):
        # write sdf
        filename = self.fileDir + "/model.sdf"
        domxml = DOM.parseString(ET.tostring(self.root))
        pretty = domxml.toprettyxml()
        file = open(filename, "w")
        file.write(pretty)
        file.close()
        # write config
        file = open(self.fileDir + '/model.config', 'w')
        file.write('<?xml version="1.0" ?>\n<model>\n<name>'+self.modelName+'</name>\n<version>1.0</version>\n')
        file.write('<sdf version="1.6">model.sdf</sdf>\n<author>\n<name></name>\n<email></email>\n</author>\n')
        file.write('<description>awesome</description>\n<changes>exported from fusion 360</changes>\n</model>\n')
        file.close();

        if (self.osimroot != None):
            file = open(self.fileDir + '/muscles.osim', 'w')
            domxml = DOM.parseString(ET.tostring(self.osimroot))
            pretty = domxml.toprettyxml()
            file.write(pretty)
            file.close()

        self.logfile.close()
        self.ui.messageBox("SDF file of model " + self.modelName + " written to '" + self.fileDir + "'.")
    ## Converts three double values to string.
    #
    # This function converts three double values to a string separated by spaces.
    #
    # @param x the first double value
    # @param y the second double value
    # @param z the third double value
    # @return the string of these values
    def vectorToString(self, x, y, z):
        string = str(x) + " " + str(y) + " " + str(z)
        return string

    ## Builds SDF pose node from vector.
    #
    # This function builds the SDF pose node for every joint.
    #
    # @param vector the vector pointing to the origin of the joint.
    # @return the SDF pose node
    def sdfPoseVector(self, vector):
        pose = ET.Element("pose", frame="")
        # convert from cm (Fusion 360) to m (SI)
        x = 0.01 * vector.x
        y = 0.01 * vector.y
        z = 0.01 * vector.z
        pos = self.vectorToString(x, y, z)
        rot = self.vectorToString(0, 0, 0)
        pose.text = pos + " " + rot
        return pose

    ## Builds SDF pose node from matrix.
    #
    # This function builds the SDF pose node for every link.
    #
    # @param matrix the transformation matrix of the link
    # @return the SDF pose node
    def sdfPoseMatrix(self, matrix):
        pose = ET.Element("pose", frame="")
        # convert from cm (Fusion 360) to m (SI)
        trans = matrix.translation
        x = 0.01 * trans.x
        y = 0.01 * trans.y
        z = 0.01 * trans.z
        pos = self.vectorToString(x, y, z)
        # calculate roll pitch yaw from transformation matrix
        r11 = matrix.getCell(0, 0)
        r21 = matrix.getCell(1, 0)
        r31 = matrix.getCell(2, 0)
        r32 = matrix.getCell(2, 1)
        r33 = matrix.getCell(2, 2)
        pitch = math.atan2(-r31, math.sqrt(math.pow(r11, 2) + math.pow(r21, 2)))
        cp = math.cos(pitch)
        yaw = math.atan2(r21 / cp, r11 / cp)
        roll = math.atan2(r32 / cp, r33 / cp)
        rot = self.vectorToString(roll, pitch, yaw)
        pose.text = pos + " " + rot
        return pose

    ## Builds SDF inertial node from physical properties.
    #
    # This function builds the SDF inertial node for every link.
    #
    # @param physics the physical properties of a link
    # @return the SDF inertial node
    def sdfInertial(self, physics, name):
        inertial = ET.Element("inertial")
        # the link frame is located at the COM, therefor we use zero vector for pose
        zeroVector = adsk.core.Point3D.create(0,0,0)
        pose = self.sdfPoseVector(zeroVector)
        inertial.append(pose)
        # build mass node
        mass = ET.Element("mass")
        if self.dummy_inertia:
            mass.text = "0.1"
        else:
            mass.text = str(physics.mass)
        inertial.append(mass)
        # build inertia node
        inertia = self.sdfInertia(physics,name)
        inertial.append(inertia)
        return inertial

    ## Builds SDF node for one moment of inertia.
    #
    # This helper function builds the SDF node for one moment of inertia.
    #
    # @param tag the tag of the XML node
    # @param value the text of the XML node
    # @return the SDF moment of inertia node
    def sdfMom(self, tag, value):
        node = ET.Element(tag)
        # convert from kg/cm^2 (Fusion 360) to kg/m^2 (SI)
        node.text = str(0.0001 * value)
        return node

    ## Builds SDF inertia node from physical properties.
    #
    # This function builds the SDF inertia node for every link.
    #
    # @param physics the physical properties of a link
    # @return the SDF inertia node
    def sdfInertia(self, physics, name):
        inertia = ET.Element("inertia")
        xx = 1000
        yy = 1000
        zz = 1000
        xy = 0
        xz = 0
        yz = 0
        if not self.dummy_inertia:
            (returnValue, xx, yy, zz, xy, yz, xz) = physics.getXYZMomentsOfInertia()
            
        inertia.append(self.sdfMom("ixx", xx))
        inertia.append(self.sdfMom("ixy", xy))
        inertia.append(self.sdfMom("ixz", xz))
        inertia.append(self.sdfMom("iyy", yy))
        inertia.append(self.sdfMom("iyz", yz))
        inertia.append(self.sdfMom("izz", zz))

        self.inertias[name].append(xx)
        self.inertias[name].append(yy)
        self.inertias[name].append(zz)
        self.inertias[name].append(xy)
        self.inertias[name].append(yz)
        self.inertias[name].append(xz)
        return inertia

    ## Builds SDF link node.
    #
    # This function builds the SDF link node for every link.
    #
    # @param lin the link to be exported
    # @param name of the link to be exported
    # @return the SDF link node
    def linkSDF(self, lin, name):
        # linkName = lin.component.name
        link = ET.Element("link", name=name)
        self_collide = ET.Element("self_collide")
        if self.self_collide:
            self_collide.text = "true"
        else:
            self_collide.text = "false"
        link.append(self_collide)
        
        # build pose node
        #matrix = gazeboMatrix()
        pose = self.sdfPoseMatrix(lin.transform)
        link.append(pose)
        # get physical properties of occurrence
        physics = lin.physicalProperties
        # build inertial node
        inertial = self.sdfInertial(physics, name)
        link.append(inertial)
        # build collision node
        collision = ET.Element("collision", name = name + "_collision")
        if (not self.exportOpenSimMuscles):
            link.append(collision)
        # build geometry node
        geometry = ET.Element("geometry")
        collision.append(geometry)
        # build mesh node
        mesh = ET.Element("mesh")
        geometry.append(mesh)
        # build uri node
        uri = ET.Element("uri")
        global modelName
        uri.text = "model://" + self.modelName + "/meshes/CAD/" + name + ".stl"
        mesh.append(uri)
        # scale the mesh from mm to m
        scale = ET.Element("scale")
        scale.text = "0.001 0.001 0.001"
        mesh.append(scale)
        # build visual node (equal to collision node)
        visual = ET.Element("visual", name = name + "_visual")
        visual.append(geometry)
        link.append(visual)
        return link

    ## Builds SDF joint node.
    #
    # This function builds the SDF joint node for every joint type.
    #
    # @param joi the joint
    # @param name_parent the name of the parent link
    # @param name_child the name of the child link
    # @return the SDF joint node
    def jointSDF(self, joi, name_parent, name_child, transformMatrix):
        jointInfo = []
        jointType = ""
        jType = joi.jointMotion.jointType
        if jType == 0: # fixed
            jointType = "fixed"
        elif jType == 1: # revolute joint
            # build axis node
            axis = ET.Element("axis")
            xyz = ET.Element("xyz")
            vector = joi.jointMotion.rotationAxisVector
            xyz.text = self.vectorToString(vector.x, vector.y, vector.z)
            axis.append(xyz)
            # build limit node
            mini = joi.jointMotion.rotationLimits.minimumValue
            maxi = joi.jointMotion.rotationLimits.maximumValue
            limit = ET.Element("limit")
            axis.append(limit)
            lower = ET.Element("lower")
            lower.text = str(mini)
            limit.append(lower)
            upper = ET.Element("upper")
            upper.text = str(maxi)
            limit.append(upper)
            # build frame node
            frame = ET.Element("use_parent_model_frame")
            frame.text = "0"
            axis.append(frame)
            jointInfo.append(axis)
            jointType = "revolute"
        elif jType == 2: # slider
            # build axis node
            axis = ET.Element("axis")
            xyz = ET.Element("xyz")
            vector = joi.jointMotion.slideDirectionVector
            xyz.text = self.vectorToString(vector.x, vector.y, vector.z)
            axis.append(xyz)
            # build limit node, convert from cm to meter
            mini = joi.jointMotion.slideLimits.minimumValue/100.0 
            maxi = joi.jointMotion.slideLimits.maximumValue/100.0
            limit = ET.Element("limit")
            axis.append(limit)
            lower = ET.Element("lower")
            lower.text = str(mini)
            limit.append(lower)
            upper = ET.Element("upper")
            upper.text = str(maxi)
            limit.append(upper)
            # build frame node
            frame = ET.Element("use_parent_model_frame")
            frame.text = "0"
            axis.append(frame)
            jointInfo.append(axis)
            jointType = "prismatic"
        elif jType == 3: # cylindrical
            # not implemented
            jointType = ""
        elif jType == 4: # pin slot
            # not implemented
            jointType = ""
        elif jType == 5: # planar
            # not implemented
            jointType = ""
        elif jType == 6: 
            # SDFormat does not implement ball joint limits
            jointType = "ball"
        name = joi.name[7:]
        joint = ET.Element("joint", name=name, type=jointType)
        # build parent node
        parent = ET.Element("parent")
        parent.text = name_parent
        joint.append(parent)
        # build child node
        child = ET.Element("child")
        child.text = name_child
        joint.append(child)
        # build pose node
        vec = joi.geometryOrOriginTwo.origin
        print(vec.asArray())
        origin = transformMatrix.translation
        origin = origin.asPoint()
        dist = origin.vectorTo(vec)
        #print(transformMatrix.asArray())
        pose = self.sdfPoseVector(dist)
        self.logfile.write("\tpos: "+ str(dist.x) + "\t" + str(dist.y) + "\t" + str(dist.z) + "\n")        
        
        joint.append(pose)
        joint.extend(jointInfo)
        return joint

    ## Plain STL export.
    ##
    # @param occ the occurrence to be exported
    # @param linkName the name of the created STL file
    def exportToStep(self, occ, linkname):
        # Create an STEPExportOptions object and do the export.
        stepOptions = self.exportMgr.createSTEPExportOptions(self.fileDir+ '/meshes/CAD/' + linkname + '.step', occ.component)
        return self.exportMgr.execute(stepOptions)

    def exportToStl(self, occ, linkname):
        # Create an STEPExportOptions object and do the export.
        stlExportOptions = self.exportMgr.createSTLExportOptions(occ, self.fileDir+ '/meshes/CAD/' + linkname + '.stl')
        stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
        return self.exportMgr.execute(stlExportOptions)

    ## Clear filenames of unwanted characters
    #
    # This function replaces all ':' with underscores and deletes spaces in filenames.
    # to one in the Gazebo cooridnate system.
    #
    # @param name a filename
    # @return the filename without ':' and spaces
    def clearName(self, name):
        name = name.replace(":", "_")
        name = name.replace(" ", "")
        return name

    def prettify(self, elem):
        """Return a pretty-printed XML string for the Element.
        """
        rough_string = ET.tostring(elem, 'utf-8')
        reparsed = DOM.parseString(rough_string)
        return reparsed.toprettyxml(indent="\t")

    def exportCASPRcables(self):
        file = open(self.fileDir + '/caspr/'+ self.modelName+'_cables.xml', 'w')
        file.write('<?xml version="1.0" encoding="utf-8"?>\n')
        file.write('<!DOCTYPE cables SYSTEM "../../../templates/cables.dtd">\n')
        cables = ET.Element('cables')
        cables.set('default_cable_set', 'WORKING')
        cable_set = ET.SubElement(cables, 'cable_set')
        cable_set.set('id','WORKING')

        allMyoMuscles = self.pluginObj.myoMuscles
        # create myoMuscle nodes
        i = 0
        for myo in allMyoMuscles:
            cable_ideal = ET.SubElement(cable_set, 'cable_ideal')
            cable_ideal.set('name', 'cable ' + str(i))
            i = i+1
            cable_ideal.set('attachment_reference', 'com')
            properties = ET.SubElement(cable_ideal, 'properties')
            force_min = ET.SubElement(properties, 'force_min')
            force_min.text = '10'
            force_max = ET.SubElement(properties, 'force_max')
            force_max.text = '80'

            attachments = ET.SubElement(cable_ideal, 'attachments')
            allViaPoints = myo.viaPoints
            allViaPoints.sort(key=lambda x: x.number)
            for via in allViaPoints:
                attachment = ET.SubElement(attachments, 'attachment')
                link = ET.SubElement(attachment, 'link')
                link.text = via.link
                location = ET.SubElement(attachment, 'location')
                location.text=via.coordinates
        file.write(self.prettify(cables))
        file.close()

    def exportCASPRbodies(self):
        file = open(self.fileDir + '/caspr/'+ self.modelName+'_bodies.xml', 'w')
        file.write('<?xml version="1.0" encoding="utf-8"?>\n')
        file.write('<!DOCTYPE bodies_system SYSTEM "../../../templates/bodies.dtd">\n')
        bodies_system = ET.Element('bodies_system')
        links = ET.SubElement(bodies_system, 'links')
        links.set('display_range','-0.3 0.3 0.0 1.0 -0.3 0.3')
        links.set('view_angle','-37 32')
        for (parent_name,(child_name, joint)) in self.joints.items():
            link_rigid = ET.SubElement(links, 'link_rigid')
            link_rigid.set('num','1')
            link_rigid.set('name',parent_name)
            joi = ET.SubElement(link_rigid, 'joint')
            joi.set('type','R_xyx')
            vector = joint.jointMotion.rotationAxisVector
            joi.set('axis',self.vectorToString(vector.x, vector.y, vector.z))
            joi.set('q_min', str(joint.jointMotion.rotationLimits.minimumValue))
            joi.set('q_max', str(joint.jointMotion.rotationLimits.maximumValue))
            physical = ET.SubElement(link_rigid, 'physical')
            mass = ET.SubElement(physical, 'mass')
            mass.text = str(self.totalMass[parent_name])

            joint_origin = joint.geometryOrOriginOne.origin

            com_origin = self.COM[parent_name]
            com_location = ET.SubElement(physical, 'com_location')
            com_location.text = str((com_origin.x-joint_origin.x)/100.0) + ' ' + str((com_origin.y-joint_origin.y)/100.0) + ' ' + str((com_origin.z-joint_origin.z)/100.0)

            end_location = ET.SubElement(physical, 'end_location')
            end_location.text = '0 0 0'
            inertia = ET.SubElement(physical, 'inertia')
            inertia.set('ref','com')
            Ixx = ET.SubElement(inertia, 'Ixx')
            Ixx.text = str(self.inertias[parent_name][0])
            Iyy = ET.SubElement(inertia, 'Iyy')
            Iyy.text = str(self.inertias[parent_name][1])
            Izz = ET.SubElement(inertia, 'Izz')
            Izz.text = str(self.inertias[parent_name][2])
            Ixy = ET.SubElement(inertia, 'Ixy')
            Ixy.text = str(self.inertias[parent_name][3])
            Ixz = ET.SubElement(inertia, 'Ixz')
            Ixz.text = str(self.inertias[parent_name][5])
            Iyz = ET.SubElement(inertia, 'Iyz')
            Iyz.text = str(self.inertias[parent_name][4])

            print(parent_name + ' ' + str(joint.geometryOrOriginOne.origin.asArray()))
            print(child_name + ' ' + str(joint.geometryOrOriginTwo.origin.asArray()))
#                transform_one = self.transformMatrices[name]
#                transform_one.invert()
#                #one.transform.transformBy(transform_one)
#                print(one.transform.asArray())
#                print(transform_one.asArray())

            parent = ET.SubElement(link_rigid, 'parent')
            num = ET.SubElement(parent, 'num')
            num.text = child_name
            location = ET.SubElement(parent, 'location')
            location.text = str(joint_origin.x/100.0) + ' ' + str(joint_origin.y/100.0) + ' ' + str(joint_origin.z/100.0)

        operational_spaces = ET.SubElement(bodies_system, 'operational_spaces')
        operational_spaces.set('default_operational_set', 'test')
        operational_set = ET.SubElement(operational_spaces, 'operational_set')
        operational_set.set('id','test')
        position = ET.SubElement(operational_set, 'position')
        position.set('marker_id','1')
        position.set('name','test1')
        link = ET.SubElement(position, 'link')
        link.text = '2'
        offset = ET.SubElement(position, 'offset')
        offset.text = '0.0 0.0 0.0'
        axes = ET.SubElement(position, 'axes')
        axes.set('active_axes','x')

        file.write(self.prettify(bodies_system))
        file.close()
