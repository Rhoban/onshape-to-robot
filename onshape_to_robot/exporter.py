import os
from .message import success
import xml.dom.minidom
from .robot import Robot


class Exporter:
    def __init__(self):
        self.xml: str = ""
        self.ext: str = "xml"

    def build(self):
        raise Exception("This exporter should implement build() method")

    def get_xml(self, robot: Robot) -> str:
        self.build(robot)
        return self.xml

    def write_xml(self, robot: Robot, filename: str) -> str:
        with open(filename, "w") as file:
            self.build(robot)
            dom = xml.dom.minidom.parseString(self.xml)
            xml_output = dom.toprettyxml(indent="    ")
            file.write(xml_output)
            print(success(f"* Writing {os.path.basename(filename)}"))
