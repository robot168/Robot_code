import pyrealsense2 as rs
import numpy as np
import cv2
import tensorflow as tf
# from realsense_depth import *



W = 640
H = 480
point = (300, 300)
distance = 0
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)


print("[INFO] start streaming...")
pipeline.start(config)

#depth_sensor = profile.get_device().first_depth_sensor()
#depth_scale = depth_sensor.get_depth_scale()
#print(" Dept scale factor:", depth_scale)

aligned_stream = rs.align(rs.stream.color) # alignment between color and depth
point_cloud = rs.pointcloud()

print("[INFO] loading model...")
PATH_TO_CKPT = r"frozen_inference_graph.pb"
# download model from: https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API#run-network-in-opencv

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.compat.v1.GraphDef()
    with tf.compat.v1.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.compat.v1.import_graph_def(od_graph_def, name='')
    sess = tf.compat.v1.Session(graph=detection_graph)

# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
# Each score represents level of confidence for each of the objects.
# The score is shown on the result image, together with the class label.
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')
# code source of tensorflow model loading: https://www.geeksforgeeks.org/ml-training-image-classifier-using-tensorflow-object-detection-api/
#x = 300
#y = 300
#point = (300, 300)
def depth_s():
    global distance
    while True:
    # ret, depth_frame, color_frame = dc.get_frame()
        frames = pipeline.wait_for_frames()
        frames = aligned_stream.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        points = point_cloud.calculate(depth_frame)
        verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, W, 3)  # xyz

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        #print("color image ")
        #print(color_image[0, 0])
        depth_image = np.asanyarray(depth_frame.get_data())
        scaled_size = (int(W), int(H))
        # expand image dimensions to have shape: [1, None, None, 3]
        # i.e. a single-column array, where each item in the column has the pixel RGB value
        image_expanded = np.expand_dims(color_image, axis=0)
        # Perform the actual detection by running the model with the image as input
        (boxes, scores, classes, num) = sess.run([detection_boxes, detection_scores, detection_classes, num_detections],
                                                feed_dict={image_tensor: image_expanded})

        boxes = np.squeeze(boxes)
        classes = np.squeeze(classes).astype(np.int32)
        scores = np.squeeze(scores)

    # print("[INFO] drawing bounding box on detected objects...")
    # print("[INFO] each detected object has a unique color")
        #global x
        #global y
        #global point 
        #point = (x, y)
        for idx in range(int(num)):
            class_ = classes[idx]
            score = scores[idx]
            box = boxes[idx]
        #    print(" [DEBUG] class : ", class_, "idx : ", idx, "num : ", num)

            if score > 0.95 and class_ == 2: # 1 for human
                #global point
                left = box[1] * W
                top = box[0] * H
                right = box[3] * W
                bottom = box[2] * H
                width = right - left
                height = bottom - top
                bbox = (int(left), int(top), int(width), int(height))
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                print("p1[0] =" + str(p1[0]) + "p1[1] = " + str(p1[1]))
                print("p2[0] =" + str(p2[0]) + "p2[1] = " + str(p2[1]))
                x = p1[0]+((p2[0]-p1[0])/2)
                y = p1[1]+((p2[1]-p1[1])/2)
                point = (int(x), int(y))
                #point = (int(((p2[0]-p2[1])/2)+p2[1]), int(((p1[0]-p1[1])/2)+p1[1]))
                print("point[0] =" + str(point[0]) + "point[1] = " + str(point[1]))
                #point1 = (int(point[0]),int(point[1]))
                # draw circle
                cv2.circle(color_image, point, 4, (0, 0, 255))
                # show color on the circled pixel
                print("Detected RGB = ")
                print(color_image[int(x), int(y)])
                # draw box
                cv2.rectangle(color_image, p1, p2, (255,0,0), 2, 1)
                # Calculate distance to object
                distance = depth_image[point[1], point[0]]
                distance_txt = "distance :" + str(distance) + " [mm]"
                print("distance to object is" + str(distance))
                # x,y,z of bounding box
                # obj_points = verts[int(bbox[1]):int(bbox[1] + bbox[3]), int(bbox[0]):int(bbox[0] + bbox[2])].reshape(-1, 3)
                # zs = obj_points[:, 2]
                # z = np.m
                # edian(zs)
                # xs = obj_points[:, 0]
                # xs = np.delete(xs, np.where(
                #     (zs < z - 1) | (zs > z + 1)))  # take only y for close z to prevent including background

                # mx = np.amin(xs, initial=1)
                # Mx = np.amax(xs, initial=-1)
                # print("min_y" + str(mx))
                # print("max_y" + str(Mx))
                
                # ys = obj_points[:, 1]
                # ys = np.delete(ys, np.where(
                #     (zs < z - 1) | (zs > z + 1)))  # take only y for close z to prevent including background
                # my = np.amin(ys, initial=1)
                # My = np.amax(ys, initial=-1)
                # print("min_y" + str(my))
                # print("max_y" + str(My))
                # mid_x = (Mx+mx)/2
                # mid_y = (My+my)/2
                # height = (My - my)  # add next to rectangle print of height using cv library
                # height = float("{:.2f}".format(height))
                # print("[INFO] object height is: ", height, "[m]")
                # height_txt = "height :" + str(height) + "[m]"
                # #height1_txt = str(height1) 

                # Write some Text
                font = cv2.FONT_HERSHEY_SIMPLEX
                bottomLeftCornerOfText = (p1[0], p1[1] - 20)
                buttomRightCornerOfText = (p2[0], p2[1] + 20)
                mid = (point[0], point[1]+20)
                fontScale = 1
                fontColor = (0, 255, 128)
                fontColor1 = (250, 128, 128)
                lineType = 2
                # cv2.putText(color_image, height_txt,
                #             bottomLeftCornerOfText,
                #             font,
                #             fontScale,
                #             fontColor,
                #             lineType)
                label= "pot"
                cv2.putText(color_image, label,
                            buttomRightCornerOfText,
                            font,
                            fontScale,
                            fontColor,
                            lineType)
                cv2.putText(color_image, distance_txt,
                            mid,
                            font,
                            fontScale,
                            fontColor1,
                            lineType)

                return distance
        # Show images
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', color_image)
        # cv2.waitKey(1)

# # Stop streaming
# pipeline.stop()


# x = depth_s()
# print (x)