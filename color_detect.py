import cv2
import numpy as np

# imageの中からcolor_l～color_hの範囲の色で物体の輪郭を取得、その輪郭の隣接矩形を返す
# 隣接矩形rect[]は　始点x,始点y,幅,高さ　で出てくる
def find_rect_of_target_color(image,color_l,color_h):
  hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
  h = hsv[:, :, 0]
  s = hsv[:, :, 1]
  mask = np.zeros(h.shape, dtype=np.uint8)
  color_l = color_l*(255/360)
  color_h = color_h*(255/360)
  mask[((h < color_h) & (h > color_l)) & (s > 170)] = 255
  contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  rects = []

  for contour in contours:
    approx = cv2.convexHull(contour)
    area = cv2.contourArea(approx)
    if area < 1e2 or 1e5 < area:
      continue
    rect = cv2.boundingRect(approx)
    rects.append(np.array(rect))
  return rects


def move(_x,_y):
  capture = cv2.VideoCapture(1)
  _, frame = capture.read()
  rects = find_rect_of_target_color(frame,180,230)
  if len(rects) > 0:
    rect = max(rects, key=(lambda x: x[2] * x[3]))
    cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
    cx = rect[0]+(rect[2] // 2)
    cy = rect[1]+(rect[3] // 2)
    width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
    widthl = int(width) // 2
    heightl = int(height) // 2
    dx = cx-widthl
    dy = cy-heightl
    cv2.circle(frame, (cx,cy), 5, (0, 255, 0), thickness=-1)
    d_centor = "dev/x="+str(dx)+",dev/y="+str(dy)
    p_centor = "x="+str(cx)+",y="+str(cy)
    
    cv2.putText(frame, p_centor, (rect[0], rect[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
    cv2.putText(frame, d_centor, (rect[0], rect[1]-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
  cv2.imshow('position', frame)
  if dx > 0 and dy > 0:
    rx = _x + 5
    ry = _y + 5
    return  rx, ry
  elif dx < 0 and dy > 0:
    rx = _x - 5
    ry = _y + 5
    return rx, ry
  elif dx > 0 and dy < 0:
    rx = _x + 5
    ry = _y - 5
    return rx, ry
  elif dx < 0 and dy < 0:
    rx = _x - 5
    ry = _y - 5
    return rx, ry
  else:
    return _x,_y


# メイン処理
if __name__ == "__main__":
  capture = cv2.VideoCapture(1)
  #capture.set(3, 1080)  # Width
  #capture.set(4, 960)  # Heigh
  print(type(capture))
  width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
  height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
  print(width,height)
  while cv2.waitKey(30) < 0:
    _, frame = capture.read()
    rects = find_rect_of_target_color(frame,13,55)
    if len(rects) > 0:
      rect = max(rects, key=(lambda x: x[2] * x[3]))
      cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
      cx = rect[0]+(rect[2] // 2)
      cy = rect[1]+(rect[3] // 2)
      cv2.circle(frame, (cx,cy), 5, (0, 255, 0), thickness=-1)
      widthl = int(width) // 2
      heightl = int(height) // 2
      p_centor = "x="+str(cx)+",y="+str(cy)
      d_centor = "dev/x="+str(cx-widthl)+",dev/y="+str(heightl-cy)
      cv2.putText(frame, p_centor, (rect[0], rect[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
      cv2.putText(frame, d_centor, (rect[0], rect[1]-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), thickness=1)
    cv2.imshow('position', frame)
  capture.release()
  cv2.destroyAllWindows()