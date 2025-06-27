import xml.etree.ElementTree as ET

# Input/Output Files
netfile = r"C:\Users\Admin\Downloads\sumo test\New folder\20 node\20e.net.xml"
tllfile = r"C:\Users\Admin\Downloads\sumo test\New folder\20 node\20e.tll.xml"

# Parse the network file
tree = ET.parse(netfile)
root = tree.getroot()

# Find all junctions of type traffic_light
junctions = []
for node in root.findall('junction'):
    if node.get('type') == 'traffic_light':
        junctions.append(node.get('id'))

# Write the TLL file
with open(tllfile, "w") as f:
    f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
    f.write('<tlLogics>\n')
    for jid in junctions:
        f.write(f'    <tlLogic id="{jid}" type="static" programID="0" offset="0">\n')
        # You may want to adjust the state length (here: 4 signals, common for 4-arm intersections)
        f.write('        <phase duration="30" state="GrGr"/>\n')
        f.write('        <phase duration="5" state="yryr"/>\n')
        f.write('        <phase duration="30" state="rGrG"/>\n')
        f.write('        <phase duration="5" state="ryry"/>\n')
        f.write('    </tlLogic>\n')
    f.write('</tlLogics>\n')

print(f"Generated {tllfile} with {len(junctions)} traffic lights.")