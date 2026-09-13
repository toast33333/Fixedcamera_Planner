# 以下代码改自https://github.com/rockchip-linux/rknn-toolkit2/tree/master/examples/onnx/yolov5
#!/usr/bin/env python3
import cv2
import numpy as np
# 改为导入ROS1相关模块
import rospy
from detection_msgs.msg import Detection  # 消息类型保持不变（需在ROS1中定义）

OBJ_THRESH, NMS_THRESH, IMG_SIZE = 0.25, 0.45, 640


CLASSES = ("person", "bicycle", "car", "motorbike ", "aeroplane ", "bus ", "train", "truck ", "boat", "traffic light",
            "fire hydrant", "stop sign ", "parking meter", "bench", "bird", "cat", "dog ", "horse ", "sheep", "cow", "elephant",
            "bear", "zebra ", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
            "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife ",
            "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza ", "donut", "cake", "chair", "sofa",
            "pottedplant", "bed", "diningtable", "toilet ", "tvmonitor", "laptop	", "mouse	", "remote ", "keyboard ", "cell phone", "microwave ",
            "oven ", "toaster", "sink", "refrigerator ", "book", "clock", "vase", "scissors ", "teddy bear ", "hair drier", "toothbrush ") 
#CLASSES = ("uav","drone","FPV")

# 原有函数保持不变（filter_boxes, nms_boxes, dfl, box_process, yolov8_post_process, draw, letterbox）
def filter_boxes(boxes, box_confidences, box_class_probs):
    
    box_confidences = box_confidences.reshape(-1)
    candidate, class_num = box_class_probs.shape

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    _class_pos = np.where(class_max_score * box_confidences >= OBJ_THRESH)
    scores = (class_max_score * box_confidences)[_class_pos]

    boxes = boxes[_class_pos]
    classes = classes[_class_pos]

    return boxes, classes, scores

def nms_boxes(boxes, scores):
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep

def dfl(position):
    n, c, h, w = position.shape
    p_num = 4
    mc = c // p_num
    y = position.reshape(n, p_num, mc, h, w)
    
    e_y = np.exp(y - np.max(y, axis=2, keepdims=True))
    y = e_y / np.sum(e_y, axis=2, keepdims=True)
    
    acc_metrix = np.arange(mc).reshape(1, 1, mc, 1, 1)
    y = (y * acc_metrix).sum(2)
    return y

def box_process(position):
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    stride = np.array([IMG_SIZE // grid_h, IMG_SIZE // grid_w]).reshape(1, 2, 1, 1)

    position = dfl(position)
    box_xy = grid + 0.5 - position[:, 0:2, :, :]
    box_xy2 = grid + 0.5 + position[:, 2:4, :, :]
    xyxy = np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)

    return xyxy

def yolov8_post_process(input_data):
   
    

        boxes, scores, classes_conf = [], [], []
        defualt_branch = 3
        pair_per_branch = len(input_data) // defualt_branch
        for i in range(defualt_branch):
          boxes.append(box_process(input_data[pair_per_branch * i]))
          classes_conf.append(input_data[pair_per_branch * i + 1])
          scores.append(np.ones_like(input_data[pair_per_branch * i + 1][:, :1, :, :], dtype=np.float32))
   

        def sp_flatten(_in):
            ch = _in.shape[1]
            _in = _in.transpose(0, 2, 3, 1)
            return _in.reshape(-1, ch)

        boxes = [sp_flatten(_v) for _v in boxes]
        classes_conf = [sp_flatten(_v) for _v in classes_conf]
        scores = [sp_flatten(_v) for _v in scores]

        boxes = np.concatenate(boxes)
        classes_conf = np.concatenate(classes_conf)
        scores = np.concatenate(scores)

        boxes, classes, scores = filter_boxes(boxes, scores, classes_conf)

        nboxes, nclasses, nscores = [], [], []
        for c in set(classes):
            inds = np.where(classes == c)
            b = boxes[inds]
            c = classes[inds]
            s = scores[inds]
            keep = nms_boxes(b, s)

            if len(keep) != 0:
                nboxes.append(b[keep])
                nclasses.append(c[keep])
                nscores.append(s[keep])

        if not nclasses and not nscores:
            return None, None, None

        boxes = np.concatenate(nboxes)
        classes = np.concatenate(nclasses)
        scores = np.concatenate(nscores)

        return boxes, classes, scores
  

def draw(image, boxes, scores, classes, ratio, padding):
    for box, score, cl in zip(boxes, scores, classes):
        top, left, right, bottom = box
        
        # 转换坐标到原始图像尺寸
        top = (top - padding[0]) / ratio[0]
        left = (left - padding[1]) / ratio[1]
        right = (right - padding[0]) / ratio[0]
        bottom = (bottom - padding[1]) / ratio[1]
       
        # 确保坐标在图像范围内
        top = max(0, int(top))
        left = max(0, int(left))
        right = min(image.shape[1], int(right))
        bottom = min(image.shape[0], int(bottom))
        
        # 绘制边界框和标签
        cv2.rectangle(image, (top, left), (int(right), int(bottom)), (255, 0, 0), 2)
        cv2.putText(image, f'{CLASSES[cl]} {score:.2f}',
                    (top, left - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2)

def letterbox(im, new_shape=(640, 640), color=(0, 0, 0)):
    shape = im.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    r = max(r,1e-6)

    ratio = r, r
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]

    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right,
                            cv2.BORDER_CONSTANT, value=color)
    return im, ratio, (left, top)

# 修改：ROS1发布逻辑
def myFunc(rknn_lite, IMG, publisher):  # 接收ROS1发布者作为参数
    IMG2 = cv2.cvtColor(IMG, cv2.COLOR_BGR2RGB)
    IMG2, ratio, padding = letterbox(IMG2)
    IMG2 = np.expand_dims(IMG2, 0)
    
    outputs = rknn_lite.inference(inputs=[IMG2], data_format=['nhwc'])
    boxes, classes, scores = yolov8_post_process(outputs)

    if boxes is not None:
        # 绘制检测结果
        draw(IMG, boxes, scores, classes, ratio, padding)

        # 新增：获取图像尺寸并检查有效性
        img_height, img_width = IMG.shape[:2]  # OpenCV格式：(高, 宽)
        if img_width <= 0 or img_height <= 0:
            rospy.logwarn("图像尺寸无效，无法计算中心坐标")
            return IMG

       
        center_x = img_width / 2.0 
        center_y = img_height / 2.0

        # 新增：检查缩放比例有效性（避免除零）
        if ratio[0] <= 1e-9 or ratio[1] <= 1e-9:
            rospy.logwarn("缩放比例异常，跳过坐标转换")
            return IMG


        # 发布每个目标的检测数据到ROS1话题
        for box, cls_idx, score in zip(boxes, classes, scores):
            # 转换坐标到原始图像尺寸
            x1, y1, x2, y2 = box
            x1 = (x1 - padding[0]) / ratio[0]
            y1 = (y1 - padding[1]) / ratio[1]
            x2 = (x2 - padding[0]) / ratio[0]
            y2 = (y2 - padding[1]) / ratio[1]
            
            # 确保坐标有效
          
            x1 = float(max(0, min(x1, img_width)))
            y1 = float(max(0, min(y1, img_height)))
            x2 = float(max(0, min(x2, img_width)))
            y2 = float(max(0, min(y2, img_height)))

            # 新增：检查坐标有效性（避免NaN/无穷大）
            if not (np.isfinite(x1) and np.isfinite(y1) and
                    np.isfinite(x2) and np.isfinite(y2)):
                rospy.logwarn("检测到无效坐标，跳过该目标")
                continue



            x1_rel = x1 - center_x
            y1_rel = center_y - y1 
            x2_rel = x2 - center_x 
            y2_rel = center_y - y2

            # 获取类别名称
            class_name = CLASSES[int(cls_idx)] if int(cls_idx) < len(CLASSES) else "unknown"
            
            # 发布ROS1消息
            msg = Detection()
            msg.x1 = x1_rel
            msg.y1 = y1_rel
            msg.x2 = x2_rel
            msg.y2 = y2_rel
            msg.score = float(score)
            msg.class_name = class_name
            publisher.publish(msg)  # 直接使用ROS1发布者发布消息

    return IMG
