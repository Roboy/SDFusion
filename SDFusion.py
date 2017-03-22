import adsk.core, adsk.fusion, traceback
import os.path, sys
import xml.etree.ElementTree as ET
import math

# transform fusion transformation matrix to gazebo frame
def gazeboMatrix(m):
    matrix = adsk.core.Matrix3D.create()
    matrix.setCell(1,1,0)
    matrix.setCell(1,2,-1)
    matrix.setCell(2,1,1)
    matrix.setCell(2,2,0)
    m.transformBy(matrix)

    return m

# build sdf pose node from vector
def sdfPoseVector(vector):
    pose = ET.Element("pose", frame="")
    pose.text = str(0.01*vector.x) + " " + str(0.01*vector.y) + " " + str(0.01*vector.z) + " 0 0 0"
    
    return pose
    
# build sdf pose node from matrix
def sdfPoseMatrix(matrix, frame):
    pose = ET.Element("pose", frame=frame)
    
    # convert from cm (fusion) to m (SI)
    trans = matrix.translation
    x = 0.01*trans.x
    y = 0.01*trans.y
    z = 0.01*trans.z
    
    # calculate roll pitch yaw from transformation matrix
    r11 = matrix.getCell(0,0)
    r21 = matrix.getCell(1,0)
    r31 = matrix.getCell(2,0)
    r32 = matrix.getCell(2,1)
    r33 = matrix.getCell(2,2)
    pitch = math.atan2(-r31, math.sqrt(math.pow(r11,2)+math.pow(r21,2)))
    cp = math.cos(pitch)
    yaw = math.atan2(r21/cp, r11/cp)
    roll = math.atan2(r32/cp, r33/cp)
    
    pose.text = str(x) + " " + str(y) + " " + str(z) + " " + str(roll) + " " + str(pitch) + " " + str(yaw)
    
    return pose
    
# build sdf inertia node from physical properties
def sdfInertia(physics):
    inertia = ET.Element("inertia")
    (returnValue, xx, yy, zz, xy, yz, xz) = physics.getXYZMomentsOfInertia()
    
    ixx = ET.Element("ixx")
    ixy = ET.Element("ixy")
    ixz = ET.Element("ixz")
    iyy = ET.Element("iyy")
    iyz = ET.Element("iyz")
    izz = ET.Element("izz")
    
    # convert from kg/cm^2 (fusion) to kg/m^2 (SI)
    ixx.text = str(0.0001*xx)
    ixy.text = str(0.0001*xy)
    ixz.text = str(0.0001*xz)
    iyy.text = str(0.0001*yy)
    iyz.text = str(0.0001*yz)
    izz.text = str(0.0001*zz)
    
    inertia.append(ixx)
    inertia.append(ixy)
    inertia.append(ixz)
    inertia.append(iyy)
    inertia.append(iyz)
    inertia.append(izz)
    return inertia
    
# build sdf link node
def linkSDF(modelName,newOcc):
    linkName = newOcc.component.name
    
    # build link node
    link = ET.Element("link", name=linkName)
            
    # build pose node
    matrix = gazeboMatrix(newOcc.transform)
    pose = sdfPoseMatrix(matrix,"")
    link.append(pose)
            
    # build inertial node
    inertial = ET.Element("inertial")
    link.append(inertial)
    
    #get physical properties of occurrence
    physics = newOcc.physicalProperties
    
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
    
    # build collision node
    collision = ET.Element("collision",name=linkName+"_collision")
    link.append(collision)
    
    # build visual node
    visual = ET.Element("visual", name=linkName+"_visual")
    link.append(visual)
    
    # build geometry node
    geometry = ET.Element("geometry")
    collision.append(geometry)
    visual.append(geometry)
    
    # build mesh node
    mesh = ET.Element("mesh")
    geometry.append(mesh)
    
    # build uri node
    uri = ET.Element("uri")
    uri.text = "model://" + modelName + "/meshes/" + linkName + ".stl"
    mesh.append(uri)
    
    # scale the mesh from mm to m
    scale = ET.Element("scale")
    scale.text = "0.001 0.001 0.001"
    mesh.append(scale)

    return link
    
# build sdf joint node
def jointSDF(joi,name_parent,name_child):

    jointInfo = []
    jointType = ""
    jType = joi.jointMotion.jointType
    if jType == 0:
        jointInfo = rigidJoint(joi)
        jointType = "fixed"
    elif jType == 1:
        jointInfo = revoluteJoint(joi)
        jointType = "revolute"
    elif jType == 2:
        jointInfo = sliderJoint(joi)
    elif jType == 3:
        jointInfo = cylindricalJoint(joi)
    elif jType == 4:
        jointInfo = pinSlotJoint(joi)
    elif jType == 5:
        jointInfo = planarJoint(joi)
    elif jType == 6:
        jointInfo = ballJoint(joi)
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


# export fixed joint
def rigidJoint(joi):
    return []

# export revolute joint
def revoluteJoint(joi):
    info = []

    # build axis node
    axis = ET.Element("axis")
    xyz = ET.Element("xyz")    
    vector = joi.jointMotion.rotationAxisVector
    xyz.text = str(vector.x) + " " + str(vector.y) + " " + str(vector.z)
    axis.append(xyz)

    # build limit node
    mini = joi.jointMotion.rotationLimits.minimumValue
    maxi = joi.jointMotion.rotationLimits.maximumValue
    limit = ET.Element("limit")
    axis.append(limit)
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

def sliderJoint(joi):
    return []

def cylindricalJoint(joi):
    return []

def pinSlotJoint(joi):
    return []

def planarJoint(joi):
    return []

# export ball joint
def ballJoint(joi):
    return []

    
# export a rigid group to STL
def rigidGroupToSTL(design,rig,fileDir):
    linkName = rig.name
    
    # create new occurrence
    linkOcc = design.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    linkOcc.component.name = linkName
    
    # copy all bodies of the rigid group to the new occurrence
    allOcc = rig.occurrences
    for occ in allOcc:
        allBod = occ.bRepBodies
        for bod in allBod:
            bod.copyToComponent(linkOcc)
            
    # export new occurrence to STL
    fileName = fileDir + "/meshes/" + linkName
    stlExportOptions = design.exportManager.createSTLExportOptions(linkOcc,fileName)
    design.exportManager.execute(stlExportOptions)
    
    return linkOcc
    
# export an occurrence to STL
def occurrenceToSTL(design,occ,fileDir):
    linkName = occ.name.replace(":","_")
    
    # create new occurrence
    linkOcc = design.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    linkOcc.component.name = linkName
    
    # copy all bodies of the occurrence to the new occurrence
    allBod = occ.bRepBodies
    for bod in allBod:
        bod.copyToComponent(linkOcc)
        
    # export new occurrence to STL
    fileName = fileDir + "/meshes/" + linkName
    stlExportOptions = design.exportManager.createSTLExportOptions(linkOcc, fileName)
    design.exportManager.execute(stlExportOptions)
    
    return linkOcc
        
def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        
        # get active design        
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        
        # get root component in this design
        rootComp = design.rootComponent
        
        # get the script location
        #scriptDir = os.path.dirname(os.path.realpath(__file__))
        
        # specify output location
        fileDir = "C:/Users/techtalentsVR1/Documents/roboy/fusion/SDFusion"
        
        # specify model name
        modelName = "legs_anna"
        
        # build sdf root node
        root = ET.Element("sdf", version="1.6")
        model = ET.Element("model", name=modelName)
        root.append(model)
        
        # get all rigid groups of the root component
        allRigidGroups = rootComp.allRigidGroups
            
        # exports all rigid groups to STL and SDF
        for rig in allRigidGroups:
            if rig is not None:
                linkOcc = rigidGroupToSTL(design,rig,fileDir)
                link = linkSDF(modelName,linkOcc)
                model.append(link)
                # delete the temporary new occurrence
                linkOcc.deleteMe()
        
                # Call doEvents to give Fusion a chance to react.
                adsk.doEvents()
        

        #get all joints of the design
        allComponents = design.allComponents
        for com in allComponents:
            if com is not None:
                allJoints = com.joints
                for joi in allJoints:
                    if joi is not None:
                        name_parent = joi.occurrenceOne.name.replace(":","_")
                        name_child = joi.occurrenceTwo.name.replace(":","_")
                        missing_link = True
                        for rig in allRigidGroups:
                            value_parent = rig.occurrences.itemByName(joi.occurrenceOne.name)
                            value_child = rig.occurrences.itemByName(joi.occurrenceTwo.name)
                            if value_parent is not None:
                                name_parent = rig.name
                            if value_child is not None:
                                name_child = rig.name
                                missing_link = False

                        joint = jointSDF(joi,name_parent,name_child)
                        model.append(joint)
                        
                        # export missing links to SDF
                        if missing_link:
                            linkOcc = occurrenceToSTL(design,joi.occurrenceTwo,fileDir)
                            link = linkSDF(modelName,linkOcc)
                            model.append(link)
                            # delete the temporary new occurrence
                            linkOcc.deleteMe()

                            # Call doEvents to give Fusion a chance to react.
                            adsk.doEvents()
        
        # build XML tree for SDF file
        tree = ET.ElementTree(root)
        filename = fileDir + "/model.sdf"
        
        # write XML tree to SDF file
        tree.write(filename)
        
        ui.messageBox('SDF file written to "' + filename + '"')

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))