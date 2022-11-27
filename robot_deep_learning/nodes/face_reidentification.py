#!/usr/bin/env python3
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import deep_models_shared_python3 as dm
import os
import glob

class FaceReidentificator:
    def __init__(self, models_directory, faces_directory,  confidence_threshold=0.2, landmarks_to_detect=None, use_neural_compute_stick=False):
        # Load the models
        models_dir = models_directory
        self.faces_dir = faces_directory
        # ========================= face identification ============================= # 
        faceID_model_dir = models_directory + '/open_model_zoo/face-reidentification-retail-0095/FP32/'
        faceID_weights_filename = faceID_model_dir + 'face-reidentification-retail-0095.bin'
        faceID_config_filename = faceID_model_dir + 'face-reidentification-retail-0095.xml'
        
        self.faceID_model = cv2.dnn.readNet(faceID_weights_filename, faceID_config_filename)
        if use_neural_compute_stick:
            print('Not attempting to use a Intel Neural Compute Stick 2 for facial landmarks due to potential errors.')

        dm.print_model_info(self.faceID_model, 'faceId_net_model')
        self.cosine_similarity_ths = 0.4 # Compare similarity threshold

    ''' 
        INputs
        Name: "data" , shape: [1x3x128x128] - An input image in the format [BxCxHxW], where:
            B - batch size
            C - number of channels
            H - image height
            W - image width

        Expected color order is BGR.

        Ouputs:
        The net outputs a blob with the shape [1, 256, 1, 1], containing a row-vector of 256 floating point values. Outputs on different images are comparable in cosine distance.
    '''
    def calculate_embeddings(self, face_crop):
        features_blob = cv2.dnn.blobFromImage(face_crop,
                                        size=(128, 128),
                                        swapRB=False,
                                        crop=False,
                                        ddepth=cv2.CV_32F)
        self.faceID_model.setInput(features_blob)
        features = self.faceID_model.forward()
        features = features[0,:,0,0]
        return features
    
    def align_faces(self, face_crop, face_shape, landmarks, debug_output=True):
        # left eye, right eye, tip of nose, left lip corner, right lip corner
        landmark_reference = np.float32([(0.68262291666666670, 0.4615741071428571),
                                        (0.31556875000000000, 0.4615741071428571),
                                        (0.50026249999999990, 0.6405053571428571),
                                        (0.65343645833333330, 0.8246919642857142),
                                        (0.34947187500000004, 0.8246919642857142)])                     
        landmark_ref = landmark_reference * np.float32([face_shape[1], face_shape[0]])
        # here is the output of landmark network
        landmark_filtered = np.float32([landmarks['left_eye_left'], landmarks['right_eye_right'], 
                landmarks['nose_tip'], landmarks['mouth_left'], landmarks['mouth_right']]) * np.float32([face_shape[1],face_shape[0]])
        if debug_output:
            print(landmark_filtered)
            print(landmark_ref)
        # Get transformation Matrix
        M = cv2.getAffineTransform(landmark_filtered[0:3], landmark_ref[0:3])
        face_crop_aligned = cv2.warpAffine(face_crop, M, (face_shape[1], face_shape[0]))
        if debug_output:
            cv2.imshow("transform", face_crop_aligned)
            cv2.waitKey(1000)
        return face_crop_aligned
    
    def get_sub_image(self, rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15):
        if enlarge_box:
            scale = enlarge_scale
            orig_h, orig_w, c = rgb_image.shape

            x0 = bounding_box[0]
            y0 = bounding_box[1]
            x1 = bounding_box[2]
            y1 = bounding_box[3]

            m_x = (x1 + x0) / 2.0
            m_y = (y1 + y0) / 2.0

            b_w = x1 - x0
            b_h = y1 - y0

            b_w = scale * b_w
            b_h = scale * b_h

            x0 = int(round(m_x - (b_w/2.0)))
            x1 = int(round(m_x + (b_w/2.0)))
            y0 = int(round(m_y - (b_h/2.0)))
            y1 = int(round(m_y + (b_h/2.0)))

            x0 = max(0, x0)
            x1 = min(orig_w, x1)
            y0 = max(0, y0)
            y1 = min(orig_h, y1)
        else: 
            x0 = int(round(bounding_box[0]))
            y0 = int(round(bounding_box[1]))
            x1 = int(round(bounding_box[2]))
            y1 = int(round(bounding_box[3]))

        actual_bounding_box = [x0, y0, x1, y1]
        image_to_crop = rgb_image
        sub_image = image_to_crop[y0:y1, x0:x1, :]
        return sub_image, actual_bounding_box

    def get_cropped_faces(self, rgb_image, bounding_boxes):
        faces = []
        for bounding_box in bounding_boxes:
            sub_image, actual_bounding_box = self.get_sub_image(rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15)
            faces.append(sub_image)
        return faces

    def loadEmbeddings(self):
        filelist = glob.glob(os.path.join(in_path, '*.npy'))
        features = {}
        for filename in filelist:
            embedding = np.load(filename)
            person_id = filename[0:-4]
            features[person_id] = embedding

    def apply_to_image(self, rgb_image, bounding_boxes, face_shape, landmarks, draw_output=False):
        if draw_output:
            output_image = rgb_image.copy()
        else:
            output_image = None
            
        ground_truth_features = self.loadEmbeddings()
        ground_truth_length = np.linalg.norm(ground_truth_features) # The length of feature
        cropped_faces = self.align_faces(self, rgb_image, bounding_boxes)
        for face in cropped_faces:
            face_crop_aligned = self.get_sub_image(face, face_shape, landmarks)
            feature = self.calculate_embeddings(face_crop_aligned)
            for gt in ground_truth_features:
                test_length = np.linalg.norm(feature)
                similarity = np.dot(gt, feature)/(ground_truth_length * test_length)
                if similarity > self.cosine_similarity_ths:
                    #if draw_output:
                    cv2.putText(output_image, 'truth', (300, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 1, cv2.LINE_AA)

        return _, output_image
