'''import cv2
print(cv2.__version__)
if hasattr(cv2, 'VideoWriter_fourcc'):
    print("cv2.VideoWriter_fourcc is available")
else:
    print("cv2.VideoWriter_fourcc is not available")

fourcc = cv2.VideoWriter_fourcc(*'MP4V')
print(fourcc)'''
import torch
m1 = torch. FloatTensor ([[1, 2]])
m2 = torch. FloatTensor ([[3], [4]])
print (m1 + m2)
