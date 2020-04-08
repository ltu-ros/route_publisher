import json
from route_publisher.msg import Route, RouteCommand

def loadRoute(filename):
    with open(filename, 'r') as f:
        return jsonToRoute(json.load(f))

def jsonToRoute(data):
    commands = []
    for c in data['commands']:
        commands.append(jsonToRouteCommand(c))

    return Route(commands=commands)

def jsonToRouteCommand(data):
    return RouteCommand(
            command=data['command'],
            x=data.get('x', 0),
            y=data.get('y', 0),
            seconds=data.get('seconds', 0))
