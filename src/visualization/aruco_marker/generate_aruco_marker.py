import cv2
import numpy as np
from cv2 import aruco

# サイズとオフセット値
size = 150
offset = 10
x_offset = y_offset = offset // 2

# マーカーの個数
num_markers = 6
grid_size = int(np.ceil(np.sqrt(num_markers)))

# 辞書を取得して画像を生成
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# 白い画像を作成（すべてのマーカーを収めるために適切なサイズにする）
img_size = grid_size * (size + offset)
img = np.zeros((img_size, img_size), dtype=np.uint8)
img += 255

# マーカーを生成して画像に重ねる
for marker_id in range(num_markers):
    ar_img = aruco.generateImageMarker(dictionary, marker_id, size)
    row = marker_id // grid_size
    col = marker_id % grid_size
    y_start = row * (size + offset) + y_offset
    x_start = col * (size + offset) + x_offset
    img[y_start:y_start + ar_img.shape[0], x_start:x_start + ar_img.shape[1]] = ar_img

cv2.imwrite("markers_0_to_5.png", img)

