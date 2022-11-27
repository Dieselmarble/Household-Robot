#!/usr/bin/env python3
# !coding=utf-8

import cv2
import  numpy as np
import os
#from Objection import Objection,Rcet

class YOLO_model:
    def __init__(self,confThreshold=0.5, nmsThreshold=0.5, objThreshold=0.6):
        modelpath = os.environ['HELLO_FLEET_PATH'] + '/deep_perception_models/' + '/yolov5/best.onnx'

        classpath = os.environ['HELLO_FLEET_PATH'] + '/deep_perception_models/' + '/yolov5/class.names'

        with open(classpath, 'rt') as f:
            self.classes = f.read().rstrip('\n').split('\n')
        self.num_classes = len(self.classes)

        self.inpHeight, self.inpWidth = 640, 640
        anchors = [[10, 13, 16, 30, 33, 23], [30, 61, 62, 45, 59, 119], [116, 90, 156, 198, 373, 326]]
        self.stride = np.array([8., 16., 32.])
        self.nl = len(anchors)
        self.na = len(anchors[0]) // 2
        self.grid = [np.zeros(1)] * self.nl
        self.anchor_grid = np.asarray(anchors, dtype=np.float32).reshape(self.nl, -1, 2)
        self.net = cv2.dnn.readNet(modelpath)
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.objThreshold = objThreshold
        self._inputNames = ''


    def resize_image(self, srcimg, keep_ratio=True, dynamic=False):
        top, left, newh, neww = 0, 0, self.inpWidth, self.inpHeight
        if keep_ratio and srcimg.shape[0] != srcimg.shape[1]:
            hw_scale = srcimg.shape[0] / srcimg.shape[1]
            if hw_scale > 1:
                newh, neww = self.inpHeight, int(self.inpWidth / hw_scale)
                img = cv2.resize(srcimg, (neww, newh), interpolation=cv2.INTER_AREA)
                if not dynamic:
                    left = int((self.inpWidth - neww) * 0.5)
                    img = cv2.copyMakeBorder(img, 0, 0, left, self.inpWidth - neww - left, cv2.BORDER_CONSTANT,
                                             value=(114, 114, 114))  # add border
            else:
                newh, neww = int(self.inpHeight * hw_scale), self.inpWidth
                img = cv2.resize(srcimg, (neww, newh), interpolation=cv2.INTER_AREA)
                if not dynamic:
                    top = int((self.inpHeight - newh) * 0.5)
                    img = cv2.copyMakeBorder(img, top, self.inpHeight - newh - top, 0, 0, cv2.BORDER_CONSTANT,
                                             value=(114, 114, 114))
        else:
            img = cv2.resize(srcimg, (self.inpWidth, self.inpHeight), interpolation=cv2.INTER_AREA)
        return img, newh, neww, top, left

    def _make_grid(self, nx=20, ny=20):
        xv, yv = np.meshgrid(np.arange(ny), np.arange(nx))
        return np.stack((xv, yv), 2).reshape((-1, 2)).astype(np.float32)

    def preprocess(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        return img

    def postprocess(self, frame,scr_img, outs, flag,padsize=None):
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]
        newh, neww, padh, padw = padsize
        ratioh, ratiow = frameHeight / newh, frameWidth / neww
        # Scan through all the bounding boxes output from the network and keep only the
        # ones with high confidence scores. Assign the box's class label as the class with the highest score.

        confidences = []
        boxes = []
        classIds = []

        for detection in outs:
            if detection[4] > self.objThreshold:
                scores = detection[5:]
                classId = np.argmax(scores)
                confidence = scores[classId] * detection[4]
                # print("confidence:", confidence)
                if confidence > self.confThreshold:
                    center_x = int((detection[0] - padw) * ratiow)
                    center_y = int((detection[1] - padh) * ratioh)
                    width = int(detection[2] * ratiow)
                    height = int(detection[3] * ratioh)
                    left = int(center_x - width * 0.5)
                    top = int(center_y - height * 0.5)

                    confidences.append(float(confidence))
                    boxes.append([left, top, width, height])
                    classIds.append(classId)
        # Perform non maximum suppression to eliminate redundant overlapping boxes with
        # lower confidences.
        if len(boxes)==0:
            return frame,[]
        # print("the before NMS boxes is ",boxes)
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confThreshold, self.nmsThreshold).flatten()
        new_boxes=[]
        if len(indices) == 0:
            print("Can't find any thing!!!")
            return frame, new_boxes
        else:
            for i in indices:
                if self.classes[classIds[i]] !=flag:
                    continue
                box = boxes[i]
                # left = box[0]
                # top = box[1]
                # width = box[2]
                # height = box[3]
                left = box[1]
                top = box[0]
                width = box[3]
                height = box[2]
                new_boxes.append([left+width/2,top+height/2,width,height])
                #frame = self.drawPred(scr_img, classIds[i], confidences[i], left, top, left + width, top + height)
        if len(new_boxes) == 0:
            print("Can't find the Object thing")
        if len(new_boxes) >= 2:
            new_boxes.sort(key = lambda x:abs(x[1]-360))
            return frame, new_boxes[0]
        return frame,new_boxes

    def drawPred(self, frame, classId, conf, left, top, right, bottom):
        # Draw a bounding box.
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), thickness=4)

        label = '%.2f' % conf
        label = '%s:%s' % (self.classes[classId], label)

        # Display the label at the top of the bounding box
        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, labelSize[1])
        # cv2.rectangle(frame, (left, top - round(1.5 * labelSize[1])), (left + round(1.5 * labelSize[0]), top + baseLine), (255,255,255), cv2.FILLED)
        # cv2.putText(frame, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), thickness=2)
        return frame

    def rotation_90(self, img_src):

        height, width = img_src.shape[:2]
        print("img width:%d height:%d" % (width, height))

        # 2.创建X,Y map
        map_x = np.zeros([width, height], np.float32)
        map_y = np.zeros([width, height], np.float32)

        # 3.执行重映射 调整 X Y map位置
        for i in range(width):
            for j in range(height):
                map_x.itemset((i, j), i)
                map_y.itemset((i, j), j)

        # 4.执行重映射处理
        img_dst = cv2.remap(img_src, map_x, map_y, cv2.INTER_LINEAR)

        # # 5.显示结果
        # cv2.imshow("img_src", img_src)
        # cv2.imshow("img_dst", img_dst)
        # #
        # cv2.waitKey()
        # cv2.destroyAllWindows()

        return img_dst

    def Detection(self, srcimg,flag):
        rot_img = self.rotation_90(srcimg)
        img, newh, neww, padh, padw = self.resize_image(rot_img)
        blob = cv2.dnn.blobFromImage(img, scalefactor=1 / 255.0, swapRB=True)
        # blob = cv2.dnn.blobFromImage(self.preprocess(img))
        # Sets the input to the network
        self.net.setInput(blob, self._inputNames)

        # Runs the forward pass to get output of the output layers
        outs = self.net.forward(self.net.getUnconnectedOutLayersNames())[0].squeeze(axis=0)

        # inference output
        # row_ind = 0
        # for i in range(self.nl):
        #     h, w = int(self.inpHeight / self.stride[i]), int(self.inpWidth / self.stride[i])
        #     length = int(self.na * h * w)
        #     if self.grid[i].shape[2:4] != (h, w):
        #         self.grid[i] = self._make_grid(w, h)
        #
        #     outs[row_ind:row_ind + length, 0:2] = (outs[row_ind:row_ind + length, 0:2] * 2. - 0.5 + np.tile(
        #         self.grid[i], (self.na, 1))) * int(self.stride[i])
        #     outs[row_ind:row_ind + length, 2:4] = (outs[row_ind:row_ind + length, 2:4] * 2) ** 2 * np.repeat(
        #         self.anchor_grid[i], h * w, axis=0)
        #     row_ind += length
        ret_img,boxes = self.postprocess(rot_img,srcimg, outs, flag,padsize=(newh, neww, padh, padw))

        return ret_img,boxes



