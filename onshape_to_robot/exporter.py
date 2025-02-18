from .message import success
import xml.dom.minidom


class Exporter:
    def __init__(self):
        self.xml: str = ""

    def build(self):
        raise Exception("This exporter should implement build() method")

    def get_xml(self) -> str:
        self.build()
        return self.xml

    def write_xml(self, filename: str) -> str:
        with open(filename, "w") as file:
            self.build()
            dom = xml.dom.minidom.parseString(self.xml)
            xml_output = dom.toprettyxml(indent="    ")
            file.write(xml_output)
            print(success(f"* Writing {filename}"))
