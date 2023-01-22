import rospy
from sentinel_drone.msg import Geolocation

class PlotFunc():
    def __init__(self):
        rospy.init_node('PlotIT')
        canvas = iface.mapCanvas()

        #Subsriber
        rospy.Subscriber('/geolocation', Geolocation, self.Plotit)

    def Plotit(self, lon, lat):
        pnt = QgsPointXY(lon, lat)

        m = QgsVertexMarker(canvas)
        m.setCenter(pnt)
        m.setColor(QColor('Black'))
        m.setIconType(QgsVertexMarker.ICON_CIRCLE)
        m.setIconSize(12)
        m.setPenWidth(1)
        m.setFillColor(QColor(0, 200, 0))

#run line below to remove:
#canvas.scene().removeItem(m)

#if __name__ == '__main__':
e_drone = PlotFunc()
r = rospy.Rate(16) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
while not rospy.is_shutdown():
	e_drone
	r.sleep()