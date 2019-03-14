'''
exportstl
===

Demos 307 redirects with the Onshape API
'''

from apikey.client import Client

# stacks to choose from
stacks = {
    'cad': 'https://cad.onshape.com'
}

# create instance of the onshape client; change key to test on another stack
c = Client(stack=stacks['cad'], logging=True)

# get features for doc
did = raw_input('Enter document ID: ')
wid = raw_input('Enter workspace ID: ')
eid = raw_input('Enter element ID: ')

# get the STL export
stl = c.part_studio_stl(did, wid, eid)

# print to the console
print stl.text
