import xml.etree.ElementTree as ET
import xml.dom.minidom as DOM

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
    motor = ''
    edge = None
    def __init__(self, coordinates='', motor='', link='', number='', edge='', global_coordinates=[]):
        self.motor = motor
        self.link = link
        self.number = number
        self.edge = edge
        self.coordinates = coordinates
        self.global_coordinates = global_coordinates

class VisualMarker:
    coordinates = ""
    link = ""

def getRobotLinkNames(rootComp):
    # Get all links of the robot (all rigid groups)
    # get active design
    # get all rigid groups of the root component
    links = []
    allRigidGroups = rootComp.allRigidGroups
    for rig in allRigidGroups:
        if rig is not None and rig.name[:6] == "EXPORT":
            links.append(rig.name)
    return links

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

def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = DOM.parseString(rough_string)
    return reparsed.toprettyxml(indent="\t")

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