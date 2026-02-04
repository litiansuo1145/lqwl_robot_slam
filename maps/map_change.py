# 简单的Python脚本清理地图
import cv2
import numpy as np

# 加载地图
img = cv2.imread('my_map_03.pgm', cv2.IMREAD_GRAYSCALE)

# 去除小斑点
kernel = np.ones((3,3), np.uint8)
img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)  # 开运算去除小障碍

# 保存
cv2.imwrite('my_map_cleaned.pgm', img)