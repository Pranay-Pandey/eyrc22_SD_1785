import csv
import os
import numpy as np

csv_path = r'/home/zhedac/images/data.csv'
visited = []

def get_file():
    global csv_path
    if not os.path.exists(csv_path):
        return
    csv_file = open(csv_path, "r")
    reader = csv.reader(csv_file)
    for line in reader:
        print(str(line))
        plot_from_point(np.float32(line[1]) , np.float32(line[2]))
        visited.append(line)
            
            
def plot_from_point(lon, lat):
    canvas = iface.mapCanvas()


    pnt = QgsPointXY(lon, lat)

    m = QgsVertexMarker(canvas)
    m.setCenter(pnt)
    m.setColor(QColor('Black'))
    m.setIconType(QgsVertexMarker.ICON_CIRCLE)
    m.setIconSize(12)
    m.setPenWidth(1)
    m.setFillColor(QColor(0, 200, 0))
    
get_file()
