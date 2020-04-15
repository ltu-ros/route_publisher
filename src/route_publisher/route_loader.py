import yaml
from route_publisher.msg import Route, RouteCommand

def loadRoute(filename):
    with open(filename, 'r') as f:
        return jsonToRoute(yaml.safe_load(f))

def jsonToRoute(data):
    print(data)
    return Route(commands=[
        jsonToRouteCommand(c) for c in data['commands']
    ])


def jsonToRouteCommand(data):
    command = data['command']
    return RouteCommand(
            command=data['command'],
            name=data.get('name', command),
            x=data.get('x', 0),
            y=data.get('y', 0),
            seconds=data.get('seconds', 0))
