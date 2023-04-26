import numpy as np
from bs4 import BeautifulSoup

with open('data/Competition_track.graphml', 'r') as f:
    data = f.read()

soup = BeautifulSoup(data, "xml")

edges = soup.find_all('edge')
nodes = soup.find_all('node')

angles = {}

for edge in edges:
    source = edge['source']
    target = edge['target']

    source_x = 69.420
    source_y = 69.420
    target_x = 69.420
    target_y = 69.420

    for node in nodes:
        coords = node.find_all('data')
        if node['id'] == source:
            source_x = float(coords[0].text)
            source_y = float(coords[1].text)
        elif node['id'] == target:
            target_x = float(coords[0].text)
            target_y = float(coords[1].text)

        if source_x != 69.420 and target_y != 69.420:
            break

    angles[source] = np.round(np.rad2deg(
        np.arctan2(target_y - source_y, target_x - source_x)))
    # angles[source] = np.round(angles[source]/90)*90

print(angles)
