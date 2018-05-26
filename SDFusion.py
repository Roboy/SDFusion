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
    COM = defaultdict(adsk.core.Point3D)
    totalMass = defaultdict()
    number_of_coms = defaultdict()
    calculateCOMFlag = True

    transformMatrices = defaultdict(adsk.core.Matrix3D)

    exportViaPoints = True
    exportLighthouseSensors = True
    exportOpenSimMuscles = False

    ## Global variable to specify the file name of the plugin loaded by the SDF.
    # Only necessary if **exportViaPoints** is **True**.
    pluginFileName = "libmyomuscle_plugin.so"

    ## Global variable to specify the name of the plugin loaded by the SDF-
    # Only necessary if **exportViaPoints** id **True**.
    pluginName = "myomuscle_plugin"
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
            return
    def asKForExportDirectory(self):
        fileDialog = self.ui.createFileDialog()
        fileDialog.isMultiSelectEnabled = False
        fileDialog.title = "Specify result directory"
        fileDialog.filter = 'directory (*/*)'
        fileDialog.filterIndex = 0
        fileDialog.initialDirectory = self.fileDir
        dialogResult = fileDialog.showSave()
        if dialogResult == adsk.core.DialogResults.DialogOK:
            self.fileDir = fileDialog.filename
            tree = self.fileDir.split('/')
            self.modelName = tree[-1]
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
        self.getCOM(name,rigidGroup.occurrences)
        if self.calculateCOMFlag == True:
            # calculate COM -> dividing by totalMass
            scaledCOM = self.COM[name].asVector()
            scaledCOM.scaleBy(1/self.totalMass[name])
            self.COM[name] = scaledCOM.asPoint()
        self.logfile.write("----------------------------------------------------------------\n")
        self.logfile.write("Mass of %s: %f\n" % (name, self.totalMass[name]))
        self.logfile.write("COM of %s: %f %f %f\n" % (name, self.COM[name].x, self.COM[name].y, self.COM[name].z))
        self.logfile.write("----------------------------------------------------------------\n")

    def getBodies(self, name, occurrences, currentLevel):
        for occurrence in occurrences:
            self.numberOfBodies[name] = self.numberOfBodies[name] + occurrence.component.bRepBodies.count
            for body in occurrence.component.bRepBodies:
                self.bodies[name].append(body)
            if occurrence.childOccurrences:
                self.getBodies(name,occurrence.childOccurrences,currentLevel+1)
    def getCOM(self, name, occurrences):
        # check if a COM point is defined
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
        progressDialog = self.app.userInterface.createProgressDialog()
        progressDialog.isBackgroundTranslucent = False
        progressDialog.show(name, 'Copy Bodies to new component: %v/%m', 0, self.numberOfBodies[name], 1)

        transformMatrix = adsk.core.Matrix3D.create()
        transformMatrix.translation = self.COM[name].asVector()
        self.transformMatrices[name] = transformMatrix
        print(self.transformMatrices[name].asArray())

        # global new_component
        new_component = self.rootOcc.addNewComponent(transformMatrix)
        i = 0
        for body in self.bodies[name]:
            new_body = body.copyToComponent(new_component)
            new_body.name = 'body'+str(i)
            progressDialog.progressValue = progressDialog.progressValue + 1
            i = i+1
            if progressDialog.wasCancelled:
                progressDialog.hide()
                return False

        self.exportToStl(new_component, name)

        link = self.linkSDF(new_component, name)
        self.model.append(link)

        # delete the temporary new occurrence
        new_component.deleteMe()
        # Call doEvents to give Fusion a chance to react.
        adsk.doEvents()

        progressDialog.hide()

        return True

    def exportJointsToSDF(self):
        #get all joints of the design
        allComponents = self.design.allComponents
        allRigidGroups = self.rootComp.allRigidGroups
        for com in allComponents:
            if com is not None:
                allJoints = com.joints
                for joi in allJoints:
                    if joi is not None and joi.name[:6] == "EXPORT":
                        one = joi.occurrenceOne
                        two = joi.occurrenceTwo
                        if one is not None and two is not None:
                            name_parent = self.clearName(one.name[7:])
                            name_child = self.clearName(two.name[7:])
                            missing_link = True
                            for rig in allRigidGroups:
                                if rig is not None and rig.name[:6] == "EXPORT":
                                    value_parent = rig.occurrences.itemByName(one.name)
                                    value_child = rig.occurrences.itemByName(two.name)
                                    if value_parent is not None:
                                        name_parent = rig.name[7:]
                                    if value_child is not None:
                                        name_child = rig.name[7:]
                                        missing_link = False
                            if missing_link==False:
                                matrix = self.transformMatrices[name_parent]
                                print(one.transform.translation.asArray())
        #                        transformMatrix.transformBy(one.transform)
                                #print(matrix.asArray())
                                joint = self.jointSDF(joi, name_parent, name_child, matrix)
                                self.model.append(joint)
    def exportViaPointsToSDF(self):
        #get all joints of the design
        allComponents = self.design.allComponents
        for com in allComponents:
            if com is not None:
                allConstructionPoints = com.constructionPoints
                for point in allConstructionPoints:
                    if point is not None:
                        if point.name[:2] == "VP":
                            viaPointInfo = point.name.split("_")
                            viaPoint = ViaPoint()
                            vec = adsk.core.Point3D.create(point.geometry.x,point.geometry.y,point.geometry.z)
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
        plugin = ET.Element("plugin", filename=self.pluginFileName, name=self.pluginName)
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
                pWrap = ET.Element("PathWrap",name="PathWrap_"+myo.number)
                wrap_object = ET.Element("wrap_object") # TODO fix objects in pWrap
                wrap_object.text = "WrapJoint"
                pWrap.append(wrap_object)
                method = ET.Element("method")
                method.text = "midpoint"
                pWrap.append(method)
                pwObjects.append(pWrap)
                gPath.append(pwSet)
                max_isometric_force = ET.Element("max_isometric_force")
                max_isometric_force.text=str(1000.0)
                optimal_fiber_length = ET.Element("optimal_fiber_length")
                optimal_fiber_length.text = str(0.14108090847730637)  # TODO calculate optiomal fiber length
                tendon_slack_length = ET.Element("tendon_slack_length")
                tendon_slack_length.text = str(0.015675656497478485) # TODO slack tendon_slack_length
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
                for point in allViaPoints:
                    pathPoint = ET.Element("PathPoint", name=muscle.get("name")+"_node"+point.number[0])
                    location = ET.Element("location")
                    location.text = point.coordinates
                    body = ET.Element("body")
                    body.text = point.link
                    pathPoint.append(location)
                    pathPoint.append(body)
                    ppSetObjects.append(pathPoint)


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
    def sdfInertial(self, physics):
        inertial = ET.Element("inertial")
        # the link frame is located at the COM, therefor we use zero vector for pose
        zeroVector = adsk.core.Point3D.create(0,0,0)
        pose = self.sdfPoseVector(zeroVector)
        inertial.append(pose)
        # build mass node
        mass = ET.Element("mass")
        mass.text = str(physics.mass)
        inertial.append(mass)
        # build inertia node
        inertia = self.sdfInertia(physics)
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
    def sdfInertia(self, physics):
        inertia = ET.Element("inertia")
        (returnValue, xx, yy, zz, xy, yz, xz) = physics.getXYZMomentsOfInertia()
        inertia.append(self.sdfMom("ixx", xx))
        inertia.append(self.sdfMom("ixy", xy))
        inertia.append(self.sdfMom("ixz", xz))
        inertia.append(self.sdfMom("iyy", yy))
        inertia.append(self.sdfMom("iyz", yz))
        inertia.append(self.sdfMom("izz", zz))
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
        # build pose node
        #matrix = gazeboMatrix()
        pose = self.sdfPoseMatrix(lin.transform)
        link.append(pose)
        # get physical properties of occurrence
        physics = lin.physicalProperties
        # build inertial node
        inertial = self.sdfInertial(physics)
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
        if jType == 0:
            jointType = "fixed"
        elif jType == 1:
            jointInfo = self.revoluteJoint(joi)
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
        name = joi.name[7:]
        joint = ET.Element("joint", name=name, type=jointType)
        # build parent node
        parent = ET.Element("parent")
        parent.text = name_child
        joint.append(parent)
        # build child node
        child = ET.Element("child")
        child.text = name_parent
        joint.append(child)
        # build pose node
        vec = joi.geometryOrOriginTwo.origin
        print(vec.asArray())
        origin = transformMatrix.translation
        origin = origin.asPoint()
        dist = origin.vectorTo(vec)
        #print(transformMatrix.asArray())
        pose = self.sdfPoseVector(dist)
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
    def revoluteJoint(self, joi):
        info = []
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


## Exports a robot model from Fusion 360 to SDFormat.
def run(context):

    try:
        exporter = SDFExporter()
        exporter.askForExportViaPoints()
        if (exporter.exportViaPoints):
            exporter.askForExportOsimMuscles()
        exporter.askForExportLighthouseSensors()
        exporter.asKForExportDirectory()

        exporter.createDiectoryStructure()

        # build sdf root node
        exporter.root = ET.Element("sdf", version="1.6")
        exporter.model = ET.Element("model", name=exporter.modelName)
        exporter.root.append(exporter.model)

        allRigidGroups = exporter.getAllRigidGroups()
        # exports all rigid groups to STL and SDF
        for rig in allRigidGroups:
            if rig is not None and rig.name[:6] == "EXPORT":
                name = rig.name[7:] # get rid of EXPORT_ tag
                exporter.getAllBodiesInRigidGroup(name,rig)
                if exporter.copyBodiesToNewComponentAndExport(name) == False:
                    return

        exporter.exportJointsToSDF()
        exporter.exportViaPointsToSDF()
        exporter.exportLighthouseSensorsToYAML()
        exporter.finish()
    except:
        if exporter.ui:
            exporter.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
