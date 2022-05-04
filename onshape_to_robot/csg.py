import json
import re
import os
import numpy as np

"""
These functions are responsible for parsing CSG files (constructive solid geometry), which are files
produced by OpenSCAD, containing no loop, variables etc.
"""


def multmatrix_parse(parameters):
    matrix = np.matrix(json.loads(parameters), dtype=float)
    matrix[0, 3] /= 1000.0
    matrix[1, 3] /= 1000.0
    matrix[2, 3] /= 1000.0
    return matrix


def cube_parse(parameters, dilatation):
    results = re.findall(r'^size = (.+), center = (.+)$', parameters)
    if len(results) != 1:
        print("! Can't parse CSG cube parameters: "+parameters)
        exit()
    extra = np.array([dilatation]*3)
    return (extra + np.array(json.loads(results[0][0]), dtype=float)/1000.0), results[0][1] == 'true'


def cylinder_parse(parameters, dilatation):
    results = re.findall(
        r'h = (.+), r1 = (.+), r2 = (.+), center = (.+)', parameters)
    if len(results) != 1:
        print("! Can't parse CSG cylinder parameters: "+parameters)
        exit()
    result = results[0]
    extra = np.array([dilatation/2, dilatation])
    return (extra + np.array([result[0], result[1]], dtype=float)/1000.0), result[3] == 'true'


def sphere_parse(parameters, dilatation):
    results = re.findall(r'r = (.+)$', parameters)
    if len(results) != 1:
        print("! Can't parse CSG sphere parameters: "+parameters)
        exit()
    return dilatation + float(results[0])/1000.0


def extract_node_parameters(line):
    line = line.strip()
    parts = line.split('(', 1)
    node = parts[0]
    parameters = parts[1]
    if parameters[-1] == ';':
        parameters = parameters[:-2]
    if parameters[-1] == '{':
        parameters = parameters[:-3]
    return node, parameters


def T(x, y, z):
    m = np.matrix(np.eye(4))
    m.T[3, :3] = [x, y, z]

    return m


def parse_csg(data, dilatation):
    shapes = []
    lines = data.split("\n")
    matrices = []
    for line in lines:
        line = line.strip()
        if line != '':
            if line[-1] == '{':
                node, parameters = extract_node_parameters(line)
                if node == 'multmatrix':
                    matrix = multmatrix_parse(parameters)
                else:
                    matrix = np.matrix(np.identity(4))
                matrices.append(matrix)
            elif line[-1] == '}':
                matrices.pop()
            else:
                node, parameters = extract_node_parameters(line)
                transform = np.matrix(np.identity(4))
                for entry in matrices:
                    transform = transform*entry
                if node == 'cube':
                    size, center = cube_parse(parameters, dilatation)
                    if not center:
                        transform = transform * \
                            T(size[0]/2.0, size[1]/2.0, size[2]/2.0)
                    shapes.append({
                        'type': 'cube',
                        'parameters': size,
                        'transform': transform
                    })
                if node == 'cylinder':
                    size, center = cylinder_parse(parameters, dilatation)
                    if not center:
                        transform = transform * T(0, 0, size[0]/2.0)
                    shapes.append({
                        'type': 'cylinder',
                        'parameters': size,
                        'transform': transform
                    })
                if node == 'sphere':
                    shapes.append({
                        'type': 'sphere',
                        'parameters': sphere_parse(parameters, dilatation),
                        'transform': transform
                    })
    return shapes


def process(filename, dilatation):
    tmp_data = os.getcwd()+'/_tmp_data.csg'
    os.system('openscad '+filename+' -o '+tmp_data)
    with open(tmp_data, "r", encoding="utf-8") as stream:
        data = stream.read()
    os.system('rm '+tmp_data)

    return parse_csg(data, dilatation)
