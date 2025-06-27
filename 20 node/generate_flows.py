import xml.etree.ElementTree as ET
import random

netfile = r"C:\Users\Admin\Downloads\sumo test\New folder\20 node\20e.net.xml"
routefile = r"C:\Users\Admin\Downloads\sumo test\New folder\20 node\flows.rou.xml"

# Parameters
normal_rate = 300
congested_rate = 30
normal_duration = 1800
congested_duration = 1800
congested_start = 1800

# Parse network to get edge IDs
tree = ET.parse(netfile)
root = tree.getroot()

edges = []
for edge in root.findall('edge'):
    if edge.get('function') is None:
        edges.append(edge.get('id'))

with open(routefile, "w") as f:
    f.write('<routes>\n')
    f.write('  <vType id="car" accel="1.0" decel="4.5" sigma="0.5" length="5" maxSpeed="13.89" color="1,0,0"/>\n')

    # Normal flows
    for i, edge_id in enumerate(edges):
        # pick a random destination edge for each flow that is not the same as the source
        to_edge = edge_id
        while to_edge == edge_id:
            to_edge = random.choice(edges)
        f.write(f'  <flow id="normal_{edge_id}" type="car" from="{edge_id}" to="{to_edge}" begin="0" end="{normal_duration}" period="{normal_rate}" />\n')

    # Congested flows
    for i, edge_id in enumerate(edges):
        to_edge = edge_id
        while to_edge == edge_id:
            to_edge = random.choice(edges)
        f.write(f'  <flow id="congested_{edge_id}" type="car" from="{edge_id}" to="{to_edge}" begin="{congested_start}" end="{congested_start+congested_duration}" period="{congested_rate}" />\n')

    f.write('</routes>\n')
print(f"Generated {routefile} with flows for {len(edges)} edges.")