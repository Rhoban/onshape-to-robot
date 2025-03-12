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
    
    def remove_empty_text_nodes(self, node: xml.dom.Node):
        to_delete = []

        for child_node in node.childNodes:
            if isinstance(child_node, xml.dom.minidom.Text):
                child_node.data = child_node.data.strip()
                if child_node.data == "":
                    to_delete.append(child_node)
            else:
                self.remove_empty_text_nodes(child_node)

        for child_node in to_delete:
            node.childNodes.remove(child_node)

    def write_xml(self, robot: Robot, filename: str) -> str:
        with open(filename, "w") as file:
            self.build(robot)
            dom = xml.dom.minidom.parseString(self.xml)
            self.remove_empty_text_nodes(dom)
            xml_output = dom.toprettyxml(indent="  ")
            file.write(xml_output)
            print(success(f"* Writing {os.path.basename(filename)}"))
