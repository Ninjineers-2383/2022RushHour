import csv
import pprint

filename = input("Enter path of telemetry file: ")

data = []

with open(filename) as f:
    reader = csv.reader(f)
    for row in reader:
        data.append(row)

areas = []
yaw = []
pitch = []


for i in data:
    if i[1].startswith("NT:/photonvision/limelight/targetArea"):
        areas.append(i)
    if i[1].startswith("NT:/photonvision/limelight/targetYaw"):
        yaw.append(i)
    if i[1].startswith("NT:/photonvision/limelight/targetPitch"):
        pitch.append(i)

pprint.pprint(sorted(zip(areas, yaw, pitch), key=lambda x: float(x[2][2]),
                     reverse=True)[:100])
