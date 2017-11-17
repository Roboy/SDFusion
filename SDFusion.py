import adsk.core
import adsk.fusion
import traceback
#import os.path
import xml.etree.ElementTree as ET
import math
import xml.dom.minidom as DOM

## @package SDFusion
# This is an exporter for Autodesk Fusion 360 models to SDFormat.
#
# This can be loaded as an addon in Fusion 360.
# It exports all rigid groups of the robot model as links
# to STL and creates nodes in the SDF file for them.
# It creates SDF nodes for all joints of the robot model.
# Supported joint types are: "fixed", "revolute", and "ball".

## Global variable to make the Fusion 360 design object accessible
# for every function.
design = None

## Global variable to make the output file directory accessible for
# every function.
fileDir = "C:/Users/techtalents/Documents/roboy/roboy2.0"
logfile = open(fileDir+'/logfile.txt', 'w')
ui = None
progressDialog = None
max_level = 0
new_component = None
exportMgr = None
bodies = 0
rootOcc = None
rootComp = None
ui = None
from collections import defaultdict
densities = defaultdict(list)
features = None
inputString = ''

## Global variable to make the robot model name accessible for
# every function.
modelName = "roboy2.0"

## Global variable to make the root occurrence accessible for
# every function.
rootOcc = None
rootComp = None

## Global variable to specify if the exporter should export viaPoints.
exportViaPoints = True

## Global variable to specify the file name of the plugin loaded by the SDF.
# Only necessary if **exportViaPoints** is **True**.
pluginFileName = "libmyomuscle_plugin.so"

## Global variable to specify the name of the plugin loaded by the SDF-
# Only necessary if **exportViaPoints** id **True**.
pluginName = "myomuscle_plugin"

## Global viaPoint

## Transforms a matrix from Fusion 360 to Gazebo.
#
# This transforms a matrix given in the Fusion 360 coordinate system
# to one in the Gazebo cooridnate system.
#
# @param self a matrix given wrt the Fusion 360 coordinate system
# @return the matrix wrt the Gazebo coordinate system
def gazeboMatrix(self):
    matrix = adsk.core.Matrix3D.create()
    matrix.setCell(1, 1, 0)
    matrix.setCell(1, 2, -1)
    matrix.setCell(2, 1, 1)
    matrix.setCell(2, 2, 0)
    self.transformBy(matrix)
    return self

## Converts three double values to string.
#
# This function converts three double values to a string separated by spaces.
#
# @param x the first double value
# @param y the second double value
# @param z the third double value
# @return the string of these values
def vectorToString(x, y, z):
    string = str(x) + " " + str(y) + " " + str(z)
    return string

## Builds SDF pose node from vector.
#
# This function builds the SDF pose node for every joint.
#
# @param vector the vector pointing to the origin of the joint.
# @return the SDF pose node
def sdfPoseVector(vector):
    pose = ET.Element("pose", frame="")
    # convert from cm (Fusion 360) to m (SI)
    x = 0.01 * vector.x
    y = 0.01 * vector.y
    z = 0.01 * vector.z
    pos = vectorToString(x, y, z)
    rot = vectorToString(0, 0, 0)
    pose.text = pos + " " + rot
    return pose
    
## Builds SDF pose node from matrix.
#
# This function builds the SDF pose node for every link.
#
# @param matrix the transformation matrix of the link
# @return the SDF pose node
def sdfPoseMatrix(matrix):
    pose = ET.Element("pose", frame="")
    # convert from cm (Fusion 360) to m (SI)
    trans = matrix.translation
    x = 0.01 * trans.x
    y = 0.01 * trans.y
    z = 0.01 * trans.z
    pos = vectorToString(x, y, z)
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
    rot = vectorToString(roll, pitch, yaw)
    pose.text = pos + " " + rot
    return pose

## Builds SDF inertial node from physical properties.
#
# This function builds the SDF inertial node for every link.
#
# @param physics the physical properties of a link
# @return the SDF inertial node
def sdfInertial(physics):
    inertial = ET.Element("inertial")
    # build pose node of COM
    com = physics.centerOfMass
    pose = sdfPoseVector(com)
    inertial.append(pose)
    # build mass node
    mass = ET.Element("mass")
    mass.text = str(physics.mass)
    inertial.append(mass)
    # build inertia node
    inertia = sdfInertia(physics)
    inertial.append(inertia)
    return inertial

## Builds SDF node for one moment of inertia.
#
# This helper function builds the SDF node for one moment of inertia.
#
# @param tag the tag of the XML node
# @param value the text of the XML node
# @return the SDF moment of inertia node
def sdfMom(tag, value):
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
def sdfInertia(physics):
    inertia = ET.Element("inertia")
    (returnValue, xx, yy, zz, xy, yz, xz) = physics.getXYZMomentsOfInertia()
    inertia.append(sdfMom("ixx", xx))
    inertia.append(sdfMom("ixy", xy))
    inertia.append(sdfMom("ixz", xz))
    inertia.append(sdfMom("iyy", yy))
    inertia.append(sdfMom("iyz", yz))
    inertia.append(sdfMom("izz", zz))
    return inertia
    
## Builds SDF link node.
#
# This function builds the SDF link node for every link.
#
# @param lin the link to be exported
# @return the SDF link node
def linkSDF(lin):
    linkName = lin.component.name
    link = ET.Element("link", name=linkName)
    # build pose node
    matrix = gazeboMatrix(lin.transform)
    pose = sdfPoseMatrix(matrix)
    link.append(pose)
    # get physical properties of occurrence
    physics = lin.physicalProperties
    # build inertial node
    inertial = sdfInertial(physics)
    link.append(inertial)
    # build collision node
    collision = ET.Element("collision", name = linkName + "_collision")
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
    uri.text = "model://" + modelName + "/meshes/" + linkName + ".stl"
    mesh.append(uri)
    # scale the mesh from mm to m
    scale = ET.Element("scale")
    scale.text = "0.001 0.001 0.001"
    mesh.append(scale)
    # build visual node (equal to collision node)
    visual = ET.Element("visual", name = linkName + "_visual")
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
def jointSDF(joi, name_parent, name_child):
    jointInfo = []
    jointType = ""
    jType = joi.jointMotion.jointType
    if jType == 0:
        jointType = "fixed"
    elif jType == 1:
        jointInfo = revoluteJoint(joi)
        jointType = "revolute"
    elif jType == 2:
        # not implemented
        jointType = ""
    elif jType == 3:
        # not implemented
        jointType = ""
    elif jType == 4:
        # not implemented
        jointType = ""
    elif jType == 5:
        # not implemented
        jointType = ""
    elif jType == 6:
        # SDFormat does not implement ball joint limits
        jointType = "ball"
    name = joi.name
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
    pose = sdfPoseVector(joi.geometryOrOriginOne.origin)
    joint.append(pose)
    joint.extend(jointInfo)
    return joint

## Builds SDF axis node for revolute joints.
#
# This function builds the SDF axis node for revolute joint.
#
# @param joi one revolute joint object
# @return a list of information nodes (here one axis node)
# for the revolute joint
def revoluteJoint(joi):
    info = []
    # build axis node
    axis = ET.Element("axis")
    xyz = ET.Element("xyz")    
    vector = joi.jointMotion.rotationAxisVector
    xyz.text = vectorToString(vector.x, vector.y, vector.z)
    axis.append(xyz)
    # build limit node
    mini = joi.jointMotion.rotationLimits.minimumValue
    maxi = joi.jointMotion.rotationLimits.maximumValue
    limit = ET.Element("limit")
    axis.append(limit)
    # Lower and upper limit have to be switched and inverted,
    # because Fusion 360 moves the parent link wrt to the
    # child link and Gazebo moves the child link wrt to the
    # parent link.
    lower = ET.Element("lower")
    lower.text = str(-maxi)
    limit.append(lower)
    upper = ET.Element("upper")
    upper.text = str(-mini)
    limit.append(upper)
    # build frame node
    frame = ET.Element("use_parent_model_frame")
    frame.text = "0"
    axis.append(frame)
    info.append(axis)
    return info
    
## Plain STL export.
##
# @param occ the occurrence to be exported
# @param linkName the name of the created STL file
def exportToStep(occ, linkname):
    global fileDir
    global logfile
    global ui
    # Get the ExportManager from the active design.
    exportMgr = design.exportManager
    # Create an STEPExportOptions object and do the export.
    stepOptions = exportMgr.createSTEPExportOptions(fileDir+ '/meshes/' + linkname + '.step', occ.component)
    res = exportMgr.execute(stepOptions)
    
def exportToStl(occ, linkname):
    global fileDir
    global logfile
    global ui
    # Get the ExportManager from the active design.
    exportMgr = design.exportManager
    # Create an STEPExportOptions object and do the export.
    stlExportOptions = exportMgr.createSTLExportOptions(occ, fileDir+ '/meshes/' + linkname + '.stl')
    stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
    res = exportMgr.execute(stlExportOptions)

def bodiesInOccurrences(occurrences,currentLevel):
    global bodies
    global densities
    global inputString
    for occurrence in occurrences:
#        if occurrence is not None:
        bodies = bodies + occurrence.component.bRepBodies.count
        inputString += str(currentLevel) + '\tComponent: ' + occurrence.name + '\n'
        for body in occurrence.component.bRepBodies:
            densities[body.physicalProperties.density].append(body)
            inputString += '\t\t\t\tBody: ' + body.name + '\n'
        if occurrence.childOccurrences:
            bodiesInOccurrences(occurrence.childOccurrences,currentLevel+1)

def mergeBodiesOfEqualMaterial(densities):
    global features
    global progressDialog
    global new_component
    for density in densities:
        i = 0
        for body in densities[density]:
            new_body = body.copyToComponent(new_component)
            new_body.name = 'body'+str(i)
            progressDialog.progressValue = progressDialog.progressValue + 1
            i = i+1
            if progressDialog.wasCancelled:
                return    
#        try:
#            ToolBodies = adsk.core.ObjectCollection.create()
#            for body in densities[density]:
#                if i is 0:
#                    TargetBody = body
#                else:
#                    ToolBodies.add(body)
#                i = i+1
#                progressDialog.progressValue = progressDialog.progressValue + 1
#                if progressDialog.wasCancelled:
#                    progressDialog.hide()
#                    return     
#            CombineInput = features.combineFeatures.createInput(TargetBody, ToolBodies )
#             
#            CombineFeats = features.combineFeatures
#            CombineInput = CombineFeats.createInput(TargetBody, ToolBodies)
#            CombineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
#            CombineFeats.add(CombineInput)
#            TargetBody.copyToComponent(new_component)
#        except:
#            print('could not combine bodies')
    progressDialog.hide()

## Exports an single occurrence to STL.
#
# This exports a single Fusion occurence as an STL file.
#
# @param occ the occurrence that needs to be exported.
# @return a new occurrence which is used to export the
# relevant information to SDFormat
def occurrenceToSTL(occ):
    global rootOcc
    linkName = clearName(occ.name)
    # create new occurrence
    linkOcc = rootOcc.addNewComponent(adsk.core.Matrix3D.create())
    linkOcc.component.name = linkName
    # copy all bodies of the occurrence to the new occurrence
    allBod = occ.bRepBodies
    for bod in allBod:
        bod.copyToComponent(linkOcc)
    # export new occurrence to STL
    exportToStl(linkOcc, linkName)
    return linkOcc

## Clear filenames of unwanted characters
#
# This function replaces all ':' with underscores and deletes spaces in filenames.
# to one in the Gazebo cooridnate system.
#
# @param name a filename
# @return the filename without ':' and spaces
def clearName(name):
    name = name.replace(":", "_")
    name = name.replace(" ", "")
    return name

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

## Exports a robot model from Fusion 360 to SDFormat.
def run(context):        
    
    try:        
        app = adsk.core.Application.get()
        ui  = app.userInterface
        # get active design        
        product = app.activeProduct
        global design
        design = adsk.fusion.Design.cast(product)
        global exportMgr
        exportMgr = design.exportManager
        global rootComp
        # get root component in this design
        rootComp = design.rootComponent
        # get all occurrences within the root component
        global rootOcc
        rootOcc = rootComp.occurrences
        # build sdf root node
        root = ET.Element("sdf", version="1.6")
        model = ET.Element("model", name=modelName)
        root.append(model)
        # get all rigid groups of the root component
        allRigidGroups = rootComp.allRigidGroups
        numberOfRigidGroupsToExport = 0
        for rig in allRigidGroups:
            if rig is not None and rig.name[:6] == "EXPORT":
                numberOfRigidGroupsToExport = numberOfRigidGroupsToExport+1
        global progressDialog
        # exports all rigid groups to STL and SDF
        for rig in allRigidGroups:
            if rig is not None and rig.name[:6] == "EXPORT":
                global features
                features = rootComp.features                        
                global inputString
                bodiesInOccurrences(rig.occurrences,0)
                logfile.write(inputString)
                global progressDialog
                progressDialog = app.userInterface.createProgressDialog()
                progressDialog.isBackgroundTranslucent = False
                progressDialog.show(rig.name, 'Copy Bodies to new component: %v/%m', 0, bodies, 1)
                global new_component                
                new_component = rootOcc.addNewComponent(adsk.core.Matrix3D.create())
                
                global densities
                mergeBodiesOfEqualMaterial(densities)
                
                if progressDialog.wasCancelled:
                    progressDialog.hide()
                
                exportToStl(new_component, rig.name[7:])
                link = linkSDF(new_component)
                model.append(link)
                # delete the temporary new occurrence
                new_component.deleteMe()
                # Call doEvents to give Fusion a chance to react.
                adsk.doEvents()
                
                if progressDialog.wasCancelled:
                    progressDialog.hide()
                    logfile.close()
                    return
#            # If progress dialog is cancelled, stop drawing.
#            if progressDialog.wasCancelled:
#                break
#            i = i + 1
#            # Update progress value of progress dialog
#            progressDialog.progressValue = i
        # Hide the progress dialog at the end.
        progressDialog.hide()
        pluginObj = Plugin()
        global exportViaPoints
        #get all joints of the design
        allComponents = design.allComponents
        for com in allComponents:
            if com is not None:
                allJoints = com.joints
                for joi in allJoints:
                    if joi is not None and joi.name[:6] == "EXPORT":
                        one = joi.occurrenceOne
                        two = joi.occurrenceTwo
                        name_parent = clearName(one.name)
                        name_child = clearName(two.name)
                        missing_link = True
                        for rig in allRigidGroups:
                            value_parent = rig.occurrences.itemByName(one.name)
                            value_child = rig.occurrences.itemByName(two.name)
                            if value_parent is not None:
                                name_parent = rig.name
                            if value_child is not None:
                                name_child = rig.name
                                missing_link = False
                        joint = jointSDF(joi, name_parent, name_child)
                        model.append(joint)
                        # export missing links to SDF
                        if missing_link:
                            linkOcc = occurrenceToSTL(two)
                            link = linkSDF(linkOcc)
                            model.append(link)
                            # delete the temporary new occurrence
                            linkOcc.deleteMe()
                            # Call doEvents to give Fusion a chance to react.
                            adsk.doEvents()
                if(exportViaPoints):
                    # get all construction points that serve as viaPoints
                    allConstructionPoints = com.constructionPoints
                    for point in allConstructionPoints:
                        if point is not None:
                            if point.name[:2] == "VP":
                                viaPointInfo = point.name.split("_")
                                viaPoint = ViaPoint()
                                p = point.geometry
                                viaPoint.coordinates = str(p.x*0.01) + " " + str(p.y*0.01) + " " + str(p.z*0.01)
                                viaPoint.link = viaPointInfo[2]
                                viaPoint.number = viaPointInfo[3]
                                myoNumber = viaPointInfo[1][5:]
                                myoMuscleList = list(filter(lambda x: x.number == myoNumber, pluginObj.myoMuscles))
                                if not myoMuscleList:
                                    myoMuscle = MyoMuscle(myoNumber)
                                    myoMuscle.viaPoints.append(viaPoint)
                                    pluginObj.myoMuscles.append(myoMuscle)
                                if myoMuscleList:
                                    myoMuscleList[0].viaPoints.append(viaPoint) 
        if(exportViaPoints):
            # create plugin node
            global pluginFileName
            global pluginName
            plugin = ET.Element("plugin", filename=pluginFileName, name=pluginName)
            model.append(plugin)
            allMyoMuscles = pluginObj.myoMuscles
            allMyoMuscles.sort(key=lambda x: x.number)
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

        filename = fileDir + "/model.sdf"
        domxml = DOM.parseString(ET.tostring(root))
        pretty = domxml.toprettyxml()
        file = open(filename, "w")
        file.write(pretty)
        file.close()
        ui.messageBox("SDF file of model " + modelName + " written to '" + fileDir + "'.")
        logfile.close()
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))