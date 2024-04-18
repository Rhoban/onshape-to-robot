import math
from colorama import Fore, Back, Style

joint_features = {}
configuration_parameters = {}


def init(client, config, root, workspaceId, assemblyId):
    global configuration_parameters, joint_features

    # Load joint features to get limits later
    if config['versionId'] == '':
        joint_features = client.get_features(
            config['documentId'], workspaceId, assemblyId)
    else:
        joint_features = client.get_features(
            config['documentId'], config['versionId'], assemblyId, type='v')

    # Retrieving root configuration parameters
    configuration_parameters = {}
    parts = root['fullConfiguration'].split(';')
    for part in parts:
        kv = part.split('=')
        if len(kv) == 2:
            configuration_parameters[kv[0]] = kv[1].replace('+', ' ')


def readExpression(expression):
    # Expression can itself be a variable from configuration
    # XXX: This doesn't handle all expression, only values and variables
    if expression[0] == '#':
        expression = configuration_parameters[expression[1:]]
    if expression[0:2] == '-#':
        expression = '-'+configuration_parameters[expression[2:]]

    parts = expression.split(' ')

    # Checking the unit, returning only radians and meters
    if parts[1] == 'deg':
        return math.radians(float(parts[0]))
    elif parts[1] in ['radian', 'rad']:
        # looking for PI
        if isinstance(parts[0], str):
            if parts[0] == '(PI)':
                value = math.pi
            else:
                raise ValueError(f"{parts[0]} variable isn't supported")
        else:
            value = parts[0]
        return float(value)
    elif parts[1] == 'mm':
        return float(parts[0])/1000.0
    elif parts[1] == 'cm':
        return float(parts[0])/100.0
    elif parts[1] == 'm':
        return float(parts[0])
    elif parts[1] == 'in':
        return float(parts[0])*0.0254
    else:
        print(Fore.RED + 'Unknown unit: '+parts[1] + Style.RESET_ALL)
        exit()


def readParameterValue(parameter, name):
    # This is an expression
    if parameter['typeName'] == 'BTMParameterNullableQuantity':
        return readExpression(parameter['message']['expression'])
    if parameter['typeName'] == 'BTMParameterConfigured':
        message = parameter['message']
        parameterValue = configuration_parameters[message['configurationParameterId']]

        for value in message['values']:
            if value['typeName'] == 'BTMConfiguredValueByBoolean':
                booleanValue = (parameterValue == 'true')
                if value['message']['booleanValue'] == booleanValue:
                    return readExpression(value['message']['value']['message']['expression'])
            elif value['typeName'] == 'BTMConfiguredValueByEnum':
                if value['message']['enumValue'] == parameterValue:
                    return readExpression(value['message']['value']['message']['expression'])
            else:
                print(Fore.RED+"Can't read value of parameter "+name+" configured with "+value['typeName']+Style.RESET_ALL)
                exit()

        print(Fore.RED+"Could not find the value for "+name+Style.RESET_ALL)
    else:
        print(Fore.RED+'Unknown feature type for '+name+': ' +
              parameter['typeName']+Style.RESET_ALL)
        exit()

# Gets the limits of a given joint


def getLimits(jointType, name):
    enabled = False
    minimum, maximum = 0, 0
    for feature in joint_features['features']:
        # Find coresponding joint
        if name == feature['message']['name']:
            # Find min and max values
            for parameter in feature['message']['parameters']:
                if parameter['message']['parameterId'] == "limitsEnabled":
                    enabled = parameter['message']['value']

                if jointType == 'revolute':
                    if parameter['message']['parameterId'] == 'limitAxialZMin':
                        minimum = readParameterValue(parameter, name)
                    if parameter['message']['parameterId'] == 'limitAxialZMax':
                        maximum = readParameterValue(parameter, name)
                elif jointType == 'prismatic':
                    if parameter['message']['parameterId'] == 'limitZMin':
                        minimum = readParameterValue(parameter, name)
                    if parameter['message']['parameterId'] == 'limitZMax':
                        maximum = readParameterValue(parameter, name)
    if enabled:
        return (minimum, maximum)
    else:
        if jointType != 'continuous':
            print(Fore.YELLOW + 'WARNING: joint ' + name + ' of type ' +
                jointType + ' has no limits ' + Style.RESET_ALL)
        return None
