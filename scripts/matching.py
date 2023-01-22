import cv2
import subprocess

img1 = cv2.imread('/home/zhedac/images/aniketaagya.png')
img2 = cv2.imread('/home/zhedac/images/task2d.tif')

img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

sift = cv2.xfeatures2d.SIFT_create()

kp1, descriptors_1 = sift.detectAndCompute(img1,None)
kp2, descriptors_2 = sift.detectAndCompute(img2,None)

bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)

matches = bf.match(descriptors_1,descriptors_2)

matches = sorted(matches, key = lambda x:x.distance)

l1 = []
l2 = []


matched_img = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], img2, flags=2)
matched_img = cv2.resize(matched_img,(0,0),fx=.2,fy=.2)
cv2.imshow('image', matched_img)

cv2.waitKey(0)
cv2.destroyAllWindows()

#print(len(matches))
for mat in matches[:20]:

    img1_idx = mat.queryIdx
    img2_idx = mat.trainIdx

    #x - columns
    #y - rows
    #Get the coordinates
    (x1, y1) = kp1[img1_idx].pt
    (x2, y2) = kp2[img2_idx].pt
    l1.append({x1,y1})
    l2.append({x2,y2})
    
print(l1,l2)

from_SRS = "EPSG:4326"

to_SRS = "EPSG:4326"

src='/home/zhedac/images/task2d.tif'

dest= '/home/zhedac/images/updated_task2d.tif'

cmd_list = ["gdalwarp","-r", "near", "-order", "1", "-s_srs", from_SRS, "-t_srs", to_SRS, "-overwrite", src, dest]

subprocess.run(cmd_list)

src="/home/zhedac/images/aniketaagya.png"
dest="/home/zhedac/images/updated_task2d.tif"
cmd_list = ["gdal_translate"]
for i in range(len(l1)):
    
    cmd_list.append("-gcp")
    cmd_list.append(str(l1[0]))
    cmd_list.append(str(l1[1])) 
    cmd_list.append(str(l2[0]))
    cmd_list.append(str(l2[1]))

cmd_list.append("-of")
cmd_list.append("GTiff")
cmd_list.append(src)
cmd_list.append(dest)
subprocess.run(cmd_list)

    
