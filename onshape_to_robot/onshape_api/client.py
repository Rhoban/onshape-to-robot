"""
client
======

Convenience functions for working with the Onshape API
"""

from .onshape import Onshape
from .cache import cache_response


def escape(s):
    return s.replace("/", "%2f").replace("+", "%2b")


class Client:
    """
    Defines methods for testing the Onshape API. Comes with several methods:

    - Create a document
    - Delete a document
    - Get a list of documents

    Attributes:
        - stack (str, default='https://cad.onshape.com'): Base URL
        - logging (bool, default=True): Turn logging on or off
    """

    def __init__(
        self, stack="https://cad.onshape.com", logging=True, creds="./config.json"
    ):
        """
        Instantiates a new Onshape client.

        Args:
            - stack (str, default='https://cad.onshape.com'): Base URL
            - logging (bool, default=True): Turn logging on or off
        """

        self._metadata_cache = {}
        self._massproperties_cache = {}
        self._stack = stack
        self._api = Onshape(stack=stack, logging=logging, creds=creds)

    def request(self, url, **kwargs):
        return self._api.request("get", url, **kwargs).json()

    def request_binary(self, url, **kwargs):
        return self._api.request("get", url, **kwargs).content

    @cache_response
    def get_document(self, did):
        """
        Get details for a specified document.

        Args:
            - did (str): Document ID

        Returns:
            - requests.Response: Onshape response data
        """
        return self.request(f"/api/documents/{escape(did)}")

    @cache_response
    def list_elements(self, did, wid, wmv="w"):
        """
        Get the list of elements in a given document
        """

        return self.request(
            f"/api/documents/d/{escape(did)}/{escape(wmv)}/{escape(wid)}/elements"
        )

    @cache_response
    def get_assembly(self, did, wmvid, eid, wmv="w", configuration="default"):
        """
        Retrieve the assembly structure for a specified document / workspace / element.
        """
        return self.request(
            f"/api/assemblies/d/{escape(did)}/{escape(wmv)}/{escape(wmvid)}/e/{escape(eid)}",
            query={
                "includeMateFeatures": "true",
                "includeMateConnectors": "true",
                "includeNonSolids": "true",
                "configuration": configuration,
            },
        )

    @cache_response
    def get_features(self, did, wvid, eid, wmv="w", configuration="default"):
        """
        Gets the feature list for specified document / workspace / part studio.

        Args:
            - did (str): Document ID
            - mid (str): Microversion
            - eid (str): Element ID

        Returns:
            - requests.Response: Onshape response data
        """

        return self.request(
            f"/api/assemblies/d/{escape(did)}/{escape(wmv)}/{escape(wvid)}/e/{escape(eid)}/features",
            query={"configuration": configuration},
        )

    @cache_response
    def get_sketches(self, did, mid, eid, configuration):
        """
        Get sketches for a given document / microversion / element.
        """
        return self.request(
            f"/api/partstudios/d/{escape(did)}/m/{escape(mid)}/e/{escape(eid)}/sketches",
            query={"includeGeometry": "true", "configuration": configuration},
        )

    @cache_response
    def get_parts(self, did, mid, eid, configuration):
        """
        Get parts for a given document / microversion / element.
        """
        return self.request(
            f"/api/parts/d/{escape(did)}/m/{escape(mid)}/e/{escape(eid)}",
            query={"configuration": configuration},
        )

    def find_new_partid(
        self, did, mid, eid, partid, configuration_before, configuration
    ):
        before = self.get_parts(did, mid, eid, configuration_before)
        name = None
        for entry in before:
            if entry["partId"] == partid:
                name = entry["name"]

        if name is not None:
            after = self.get_parts(did, mid, eid, configuration)
            for entry in after:
                if entry["name"] == name:
                    return entry["partId"]
        else:
            print("Onshape ERROR: Can't find new partid for " + str(partid))

        return partid

    @cache_response
    def part_studio_stl_m(
        self,
        did,
        wmvid,
        eid,
        partid="",
        wmv="m",
        configuration="default",
        linked_document_id=None,
    ):
        req_headers = {"Accept": "*/*"}
        query = {
            "mode": "binary",
            "units": "meter",
            "configuration": configuration,
        }
        if linked_document_id is not None:
            query["linkDocumentId"] = linked_document_id
        return self.request_binary(
            f"/api/parts/d/{escape(did)}/{escape(wmv)}/{escape(wmvid)}/e/{escape(eid)}/partid/{escape(partid)}/stl",
            query=query,
            headers=req_headers,
        )

    @cache_response
    def matevalues(self, did, wmvid, eid, wmv="w", configuration="default"):
        return self.request(
            f"/api/assemblies/d/{escape(did)}/{wmv}/{escape(wmvid)}/e/{escape(eid)}/matevalues",
            query={"configuration": configuration},
        )

    @cache_response
    def part_get_metadata(
        self,
        did,
        wmvid,
        eid,
        partid,
        wmv="m",
        configuration="default",
        linked_document_id=None,
    ):
        query = {"configuration": configuration}
        if linked_document_id is not None:
            query["linkDocumentId"] = linked_document_id
        return self.request(
            f"/api/metadata/d/{escape(did)}/{escape(wmv)}/{escape(wmvid)}/e/{escape(eid)}/p/{escape(partid)}",
            query=query,
        )

    @cache_response
    def part_mass_properties(
        self,
        did,
        wmvid,
        eid,
        partid,
        wmv="m",
        configuration="default",
        linked_document_id=None,
    ):
        query = {
            "configuration": configuration,
            "useMassPropertyOverrides": True,
        }
        if linked_document_id is not None:
            query["linkDocumentId"] = linked_document_id
        return self.request(
            f"/api/parts/d/{escape(did)}/{escape(wmv)}/{escape(wmvid)}/e/{escape(eid)}/partid/{escape(partid)}/massproperties",
            query=query,
        )

    @cache_response
    def standard_cont_mass_properties(
        self, did, vid, eid, partid, linked_document_id, configuration
    ):
        return self.request(
            f"/api/parts/d/{escape(did)}/v/{escape(vid)}/e/{escape(eid)}/partid/{escape(partid)}/massproperties",
            query={
                "configuration": configuration,
                "useMassPropertyOverrides": True,
                "linkDocumentId": linked_document_id,
                "inferMetadataOwner": True,
            },
        )

    @cache_response
    def elements_configuration(
        self, did, wmvid, eid, wmv, linked_document_id=None, configuration=None
    ):
        query = {}
        if linked_document_id is not None:
            query["linkDocumentId"] = linked_document_id
        return self.request(
            f"/api/elements/d/{escape(did)}/{escape(wmv)}/{escape(wmvid)}/e/{escape(eid)}/configuration",
            query=query,
        )

    @cache_response
    def get_variables(self, did, wvid, eid, wmv, configuration):
        return self.request(
            f"/api/variables/d/{escape(did)}/{escape(wmv)}/{escape(wvid)}/e/{escape(eid)}/variables",
            query={
                "configuration": configuration,
                "includeValuesAndReferencedVariables": True,
            },
        )
