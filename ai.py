import struct
import xml.etree.ElementTree as ET
from Queue import *
from math import sqrt
import math

# bounds of the window, in lat/long
LEFTLON = -78.9132
RIGHTLON = -78.8872
TOPLAT = 43.8953
BOTLAT = 43.8673
WIDTH = RIGHTLON-LEFTLON
HEIGHT = TOPLAT-BOTLAT
# ratio of one degree of longitude to one degree of latitude
LONRATIO = math.cos(TOPLAT*3.1415/180)
EPIX = 1201
MPERLAT = 111000
MPERLON = MPERLAT*LONRATIO


def node_dist(n1, n2):
    dx = (n2.pos[0]-n1.pos[0])*MPERLON
    dy = (n2.pos[1]-n1.pos[1])*MPERLAT
    return math.sqrt(dx*dx+dy*dy)


class Way():
    def __init__(self, n, t):
        self.name = n
        self.type = t
        self.nodes = []


class Edge():
    def __init__(self, w, src, d):
        self.way = w
        self.dest = d
        self.cost = node_dist(src, d)


class Node():
    def __init__(self, id, p, e=0):
        self.id = id
        self.pos = p
        self.ways = []
        self.elev = e
        self.waystr = None

    def get_waystr(self):
        if self.waystr is None:
            self.waystr = ""
            self.wayset = set()
            for w in self.ways:
                self.wayset.add(w.way.name)
            for w in self.wayset:
                self.waystr += w.encode("utf-8") + " "
        return self.waystr


def build_elevs(efilename):
    efile = open(efilename)
    estr = efile.read()
    elevs = []
    for spot in range(0, len(estr), 2):
        elevs.append(struct.unpack('>h', estr[spot:spot+2])[0])
    return elevs


def build_graph(elevs):
    tree = ET.parse('map1.osm')
    root = tree.getroot()
    nodes = dict()
    ways = dict()
    coastnodes = []
    xml_ids = []
    for item in root:
        if item.tag == 'node':
            coords = ((float)(item.get('lat')), (float)(item.get('lon')))
            erow = (int)((43 - coords[0]) * EPIX)
            ecol = (int)((coords[1]-18) * EPIX)
            try:
                el = elevs[erow*EPIX+ecol]
            except IndexError:
                el = 0
            xml_ids.append((long)(item.get('id')))
            nodes[(long)(item.get('id'))] = Node(
                (long)(item.get('id')), coords, el)
        elif item.tag == 'way':
            useme = False
            oneway = False
            myname = 'unknown road'
            for thing in item:
                if thing.tag == 'tag' and thing.get('k') == 'highway':
                    useme = True
                    mytype = thing.get('v')
                if thing.tag == 'tag' and thing.get('k') == 'name':
                    myname = thing.get('v')
                if thing.tag == 'tag' and thing.get('k') == 'oneway':
                    if thing.get('v') == 'yes':
                        oneway = True
            if useme:
                wayid = (long)(item.get('id'))
                ways[wayid] = Way(myname, mytype)
                nlist = []
                for thing in item:
                    if thing.tag == 'nd':
                        nlist.append((long)(thing.get('ref')))
                thisn = nlist[0]
                for n in range(len(nlist)-1):
                    nextn = nlist[n+1]
                    nodes[thisn].ways.append(
                        Edge(ways[wayid], nodes[thisn], nodes[nextn]))
                    thisn = nextn
                if not oneway:
                    thisn = nlist[-1]
                    for n in range(len(nlist)-2, -1, -1):
                        nextn = nlist[n]
                        nodes[thisn].ways.append(
                            Edge(ways[wayid], nodes[thisn], nodes[nextn]))
                        thisn = nextn
                ways[wayid].nodes = nlist
    return nodes, ways, coastnodes, xml_ids


class Planner():
    __slots__ = ('nodes', 'ways')

    def __init__(self, n, w):
        self.nodes = n
        self.ways = w

    def heur(self, node, gnode):
        # Heuristic function is simply a slope between two nodes.
        a = node_dist(node, gnode)
        b = gnode.elev - node.elev
        return math.sqrt(a*a + b*b)

    def plan(self, s, g):
        '''
        Standard A* search
        '''
        parents = {}
        costs = {}
        q = PriorityQueue()
        q.put((self.heur(s, g), s))
        parents[s] = None
        costs[s] = 0
        while not q.empty():
            cf, cnode = q.get()
            if cnode == g:
                print "Path found, time will be", costs[g]*60/5000, " minutes"
                return self.make_path(parents, g)

            for edge in cnode.ways:
                newcost = costs[cnode] + edge.cost
                if edge.dest not in parents or newcost < costs[edge.dest]:
                    parents[edge.dest] = (cnode, edge.way)
                    costs[edge.dest] = newcost
                    q.put((self.heur(edge.dest, g)+newcost, edge.dest))

    def make_path(self, par, g):
        nodes = []
        ways = []
        curr = g
        nodes.append(curr)
        while par[curr] is not None:
            prev, way = par[curr]
            ways.append(way.name)
            nodes.append(prev)
            curr = prev
        nodes.reverse()
        ways.reverse()
        return nodes, ways


def get_nodes(nodes, xml_ids, s):
    for i in range(0, len(nodes)):
        if s in str(nodes.get(xml_ids[i]).pos):
            return nodes.get(xml_ids[i])


nodes, ways, coastnodes, xml_ids = build_graph(
    build_elevs("n43_w079_1arc_v2.bil"))


planner = Planner(nodes, ways)

startNode = (get_nodes(nodes, xml_ids, "(43.8792082, -78.903459)"))
endNode = (get_nodes(nodes, xml_ids, "(43.8855935, -78.9040196)"))
n, w = planner.plan(startNode, endNode)
dup = ""
print "Visit the following roads..."
for item in w:
    if item != dup:
        print item
        dup = item
