'''
client
======

Convenience functions for working with the Onshape API
'''

from .onshape import Onshape

import mimetypes
import random
import string
import os
import json
import hashlib

def escape_slash(s):
    return s.replace('/', '%252f')

class Client():
    '''
    Defines methods for testing the Onshape API. Comes with several methods:

    - Create a document
    - Delete a document
    - Get a list of documents

    Attributes:
        - stack (str, default='https://cad.onshape.com'): Base URL
        - logging (bool, default=True): Turn logging on or off
    '''

    def __init__(self, stack='https://cad.onshape.com', logging=True, creds='./config.json'):
        '''
        Instantiates a new Onshape client.

        Args:
            - stack (str, default='https://cad.onshape.com'): Base URL
            - logging (bool, default=True): Turn logging on or off
        '''

        self._metadata_cache = {}
        self._massproperties_cache = {}
        self._stack = stack
        self._api = Onshape(stack=stack, logging=logging, creds=creds)
        self.useCollisionsConfigurations = True

    def new_document(self, name='Test Document', owner_type=0, public=False):
        '''
        Create a new document.

        Args:
            - name (str, default='Test Document'): The doc name
            - owner_type (int, default=0): 0 for user, 1 for company, 2 for team
            - public (bool, default=False): Whether or not to make doc public

        Returns:
            - requests.Response: Onshape response data
        '''

        payload = {
            'name': name,
            'ownerType': owner_type,
            'isPublic': public
        }

        return self._api.request('post', '/api/documents', body=payload)

    def rename_document(self, did, name):
        '''
        Renames the specified document.

        Args:
            - did (str): Document ID
            - name (str): New document name

        Returns:
            - requests.Response: Onshape response data
        '''

        payload = {
            'name': name
        }

        return self._api.request('post', '/api/documents/' + did, body=payload)

    def del_document(self, did):
        '''
        Delete the specified document.

        Args:
            - did (str): Document ID

        Returns:
            - requests.Response: Onshape response data
        '''

        return self._api.request('delete', '/api/documents/' + did)

    def get_document(self, did):
        '''
        Get details for a specified document.

        Args:
            - did (str): Document ID

        Returns:
            - requests.Response: Onshape response data
        '''
        return self._api.request('get', '/api/documents/' + did)

    def cache_get(self, method, key, callback, isString = False):
        if type(key) == tuple:
            key = '_'.join(list(key))
        fileName = method+'__'+key
        dirName = os.path.dirname(os.path.abspath(__file__))+'/cache'
        if not os.path.exists(dirName):
            os.mkdir(dirName)
        fileName = dirName+'/'+fileName
        if os.path.exists(fileName):
            f = open(fileName, 'rb')
            result = f.read()
            f.close()
        else:
            result = callback().content
            f = open(fileName, 'wb')
            f.write(result)
            f.close()
        if isString and type(result) == bytes:
            result = result.decode('utf-8')
        return result
            

    def list_documents(self):
        '''
        Get list of documents for current user.

        Returns:
            - requests.Response: Onshape response data
        '''

        return self._api.request('get', '/api/documents')

    def list_elements(self, did, wid, type='w'):
        '''
        Get the list of elements in a given document
        '''

        return self._api.request('get', '/api/documents/d/'+did+'/'+type+'/'+wid+'/elements')

    def create_assembly(self, did, wid, name='My Assembly'):
        '''
        Creates a new assembly element in the specified document / workspace.

        Args:
            - did (str): Document ID
            - wid (str): Workspace ID
            - name (str, default='My Assembly')

        Returns:
            - requests.Response: Onshape response data
        '''

        payload = {
            'name': name
        }

        return self._api.request('post', '/api/assemblies/d/' + did + '/w/' + wid, body=payload)

    def get_assembly(self, did, wid, eid, type='w'):
        return self._api.request('get', '/api/assemblies/d/'+did+'/'+type+'/'+wid+'/e/'+eid, query={'includeMateFeatures': 'true', 'includeMateConnectors': 'true', 'includeNonSolids': 'true', 'configuration': 'default'}).json()

    def get_features(self, did, wid, eid, type='w'):
        '''
        Gets the feature list for specified document / workspace / part studio.

        Args:
            - did (str): Document ID
            - wid (str): Workspace ID
            - eid (str): Element ID

        Returns:
            - requests.Response: Onshape response data
        '''

        return self._api.request('get', '/api/assemblies/d/' + did + '/'+type+'/' + wid + '/e/' + eid + '/features').json()

    def get_assembly_features(self, did, wid, eid):
        '''
        Gets the feature list for specified document / workspace / part studio.

        Args:
            - did (str): Document ID
            - wid (str): Workspace ID
            - eid (str): Element ID

        Returns:
            - requests.Response: Onshape response data
        '''

        return self._api.request('get', '/api/assemblies/d/' + did + '/w/' + wid + '/e/' + eid + '/features')

    def get_partstudio_tessellatededges(self, did, wid, eid):
        '''
        Gets the tessellation of the edges of all parts in a part studio.

        Args:
            - did (str): Document ID
            - wid (str): Workspace ID
            - eid (str): Element ID

        Returns:
            - requests.Response: Onshape response data
        '''

        return self._api.request('get', '/api/partstudios/d/' + did + '/w/' + wid + '/e/' + eid + '/tessellatededges')

    def upload_blob(self, did, wid, filepath='./blob.json'):
        '''
        Uploads a file to a new blob element in the specified doc.

        Args:
            - did (str): Document ID
            - wid (str): Workspace ID
            - filepath (str, default='./blob.json'): Blob element location

        Returns:
            - requests.Response: Onshape response data
        '''

        chars = string.ascii_letters + string.digits
        boundary_key = ''.join(random.choice(chars) for i in range(8))

        mimetype = mimetypes.guess_type(filepath)[0]
        encoded_filename = os.path.basename(filepath)
        file_content_length = str(os.path.getsize(filepath))
        blob = open(filepath)

        req_headers = {
            'Content-Type': 'multipart/form-data; boundary="%s"' % boundary_key
        }

        # build request body
        payload = '--' + boundary_key + '\r\nContent-Disposition: form-data; name="encodedFilename"\r\n\r\n' + encoded_filename + '\r\n'
        payload += '--' + boundary_key + '\r\nContent-Disposition: form-data; name="fileContentLength"\r\n\r\n' + file_content_length + '\r\n'
        payload += '--' + boundary_key + '\r\nContent-Disposition: form-data; name="file"; filename="' + encoded_filename + '"\r\n'
        payload += 'Content-Type: ' + mimetype + '\r\n\r\n'
        payload += blob.read()
        payload += '\r\n--' + boundary_key + '--'

        return self._api.request('post', '/api/blobelements/d/' + did + '/w/' + wid, headers=req_headers, body=payload)

    def part_studio_stl(self, did, wid, eid):
        '''
        Exports STL export from a part studio

        Args:
            - did (str): Document ID
            - wid (str): Workspace ID
            - eid (str): Element ID

        Returns:
            - requests.Response: Onshape response data
        '''

        req_headers = {
            'Accept': 'application/vnd.onshape.v1+octet-stream'
        }
        return self._api.request('get', '/api/partstudios/d/' + did + '/w/' + wid + '/e/' + eid + '/stl', headers=req_headers)

    def hash_partid(self, data):
        m = hashlib.sha1()
        m.update(data.encode('utf-8'))
        return m.hexdigest()

    def get_sketches(self, did, mid, eid, configuration):
        def invoke():
            return self._api.request('get', '/api/partstudios/d/' + did + '/m/' + mid + '/e/' + eid + '/sketches', 
            query={'includeGeometry': 'true', 'configuration': configuration})

        return json.loads(self.cache_get('sketches', (did, mid, eid, configuration), invoke))

    def get_parts(self, did, mid, eid, configuration):
        def invoke():
            return self._api.request('get', '/api/parts/d/' + did + '/m/' + mid + '/e/' + eid, query=
                {'configuration': configuration})

        return json.loads(self.cache_get('parts_list', (did, mid, eid, configuration), invoke))

    def find_new_partid(self, did, mid, eid, partid, configuration_before, configuration):
        before = self.get_parts(did, mid, eid, configuration_before)
        name = None
        for entry in before:
            if entry['partId'] == partid:
                name = entry['name']
        
        if name is not None:
            after = self.get_parts(did, mid, eid, configuration)
            for entry in after:
                if entry['name'] == name:
                    return entry['partId']
        else:
            print("OnShape ERROR: Can't find new partid for "+str(partid))

        return partid

    def part_studio_stl_m(self, did, mid, eid, partid, configuration = 'default'):
        if self.useCollisionsConfigurations:
            configuration_before = configuration
            parts = configuration.split(';')
            partIdChanged = False
            result = ''
            for k, part in enumerate(parts):
                kv = part.split('=')
                if len(kv) == 2:
                    if kv[0] == 'collisions':
                        kv[1] = 'true'
                        partIdChanged = True
                parts[k] = '='.join(kv)
            configuration = ';'.join(parts)

            if partIdChanged:
                partid = self.find_new_partid(did, mid, eid, partid, configuration_before, configuration)
    
        def invoke():
            req_headers = {
                'Accept': 'application/vnd.onshape.v1+octet-stream'
            }
            return self._api.request('get', '/api/parts/d/' + did + '/m/' + mid + '/e/' + eid + '/partid/'+escape_slash(partid)+'/stl', query={'mode': 'binary', 'units': 'meter', 'configuration': configuration}, headers=req_headers)

        return self.cache_get('part_stl', (did, mid, eid, self.hash_partid(partid), configuration), invoke)

    def part_get_metadata(self, did, mid, eid, partid, configuration = 'default'):
        def invoke():
            return self._api.request('get', '/api/parts/d/' + did + '/m/' + mid + '/e/' + eid + '/partid/'+escape_slash(partid)+'/metadata', query={'configuration': configuration})

        return json.loads(self.cache_get('metadata', (did, mid, eid, self.hash_partid(partid), configuration), invoke, True))

    def part_mass_properties(self, did, mid, eid, partid, configuration = 'default'):
        def invoke():
            return self._api.request('get', '/api/parts/d/' + did + '/m/' + mid + '/e/' + eid + '/partid/'+escape_slash(partid)+'/massproperties', query={'configuration': configuration})

        return json.loads(self.cache_get('massproperties', (did, mid, eid, self.hash_partid(partid), configuration), invoke, True))
