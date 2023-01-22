import subprocess




##Changing CO-ordinate reference system 
from_SRS = "EPSG:4326"
to_SRS = "EPSG:4326"
src=r'/home/pranay/Downloads/task2d.tif'
dest= r'/home/pranay/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/updated.tif'
cmd_list = ["gdalwarp","-r", "bilinear", "-s_srs", from_SRS, "-t_srs", to_SRS, "-overwrite", src, dest]

subprocess.run(cmd_list)