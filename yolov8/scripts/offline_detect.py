#!/usr/bin/env python3

from ultralytics import YOLO
#from PIL import Image
import cv2
import inspect
import math

from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Bool

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from scipy.spatial.transform import Rotation as Rot

MODEL_DATA_PATH = "/home/gino/projects/denny_ws/src/kuka_screwdriving_utils/yolov8/best_2marker_model.pt" 
IMAGE_PATH = "/home/gino/projects/denny_ws/src/kuka_screwdriving_utils/yolov8/image_1.jpeg" #acquisizione1.jpeg
DEPTH_FROM_CSV='/home/gino/projects/denny_ws/src/kuka_screwdriving_utils/yolov8/depth1.csv'

DEPTH_FILE_NAME='/home/gino/projects/denny_ws/src/kuka_screwdriving_utils/yolov8/depth111.csv'
IMAGE_SAVE_NAME='/home/gino/projects/denny_ws/src/kuka_screwdriving_utils/yolov8/image_111.jpeg'

# To see Results class: https://docs.ultralytics.com/modes/predict/#working-with-results
class imageDetection(Node):
    def __init__(self):
        super().__init__('image_detection')
        #self.publisher_ = self.create_publisher(Bool, 'detection_status', 10) #per permettere al servizio di controllare lo stato
        self.subscription_depth = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.callback_depth, 10)
        self.subscription_rgb = self.create_subscription(Image, '/camera/camera/color/image_raw', self.callback_rgb, 10)
        ###############################################################################################################
        ###############################################################################################################
        ###############################################################################################################
        self.depth_ok = True    # <<<<<<<<<<<<<<<<<<<-------------------------------------------------||||||||||||||||
        self.rgb_ok = True    # <<<<<<<<<<<<<<<<<<<-------------------------------------------------||||||||||||||||
        self.bridge = CvBridge()
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.goal = [1.0, 1.0, 1.0]

        ###########################################################################
        # creo diverse rotazioni per la presa
        self.rotation = [3.14, 0, 0] #[-1.57, -1.57, 0.0] #[[-1.57, -1.57, 0.0],[0.0, -2.35, -2.17],[0.0, -2.35, -0.97]]#[[0.0, -2.35, -1.57],[0.0, -2.35, -2.17],[0.0, -2.35, -0.97]]# (ai, aj, ak) radianti
        #self.frames=['frameGripper1','frameGripper2','frameGripper3']
        self.frame='screw_frame'
        self.marker_translation = [0.0, 0.0, 0.0]
        self.marker_rotation = [0.0, 0.0, 0.0]
        self.marker_x_translation = [0.0, 0.0, 0.0]
        self.marker_x_rotation = [0.0, 0.0, 0.0]
        self.marker_y_translation = [0.0, 0.0, 0.0]
        self.marker_y_rotation = [0.0, 0.0, 0.0]
        self.realsense_translation = [0.0, 0.0, 0.0]
        self.realsense_rotation = [0.0, 0.0, 0.0]      
        self.Po_translation = [0.0, 0.0, 0.0]
        self.Px_translation = [0.0, 0.0, 0.0]
        self.Py_translation = [0.0, 0.0, 0.0]
    
    def callback_rgb(self, data):
        if not self.rgb_ok:
            self.rgb_raw = data
            try:
                #Convert ROS Image message to OpenCV2
                self.rgb_cv2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
            else:
                #Save OpenCV2 image as jpeg
                cv2.imwrite(IMAGE_SAVE_NAME, self.rgb_cv2)
            self.rgb_ok = True

    def callback_depth(self, data):
        if not self.depth_ok:
            self.depth_raw = data
            self.depth = self.bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth_ok = True

    def make_transforms(self, traslation, rotation, frame_name, from_frame="world"):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()

        t.header.frame_id = from_frame #"map"
        t.child_frame_id = frame_name

        t.transform.translation.x = float(traslation[0])
        t.transform.translation.y = float(traslation[1])
        t.transform.translation.z = float(traslation[2])
        #################################################################################################
        quat = quaternion_from_euler(
            float(rotation[0]), float(rotation[1]), float(rotation[2]))
        #print(rotation)
        t.transform.rotation.x = quat[0]#float(rotation[0])#
        t.transform.rotation.y = quat[1]#float(rotation[1])#
        t.transform.rotation.z = quat[2]#float(rotation[2])#
        t.transform.rotation.w = quat[3]#float(rotation[3])#
        self.tf_broadcaster.sendTransform(t)
        #self.get_logger().info("Published " + frame_name)

    def on_timer(self):
        #Publish realsense frame
        self.make_transforms(self.realsense_translation, self.realsense_rotation, 'realsense')
        #Publish Po
        self.make_transforms(self.Po_translation, [0.0, 0.0, 0.0], 'Po','realsense')
        #Publish Px
        self.make_transforms(self.Px_translation, [0.0, 0.0, 0.0], 'Px','realsense')
        #Publish Py
        self.make_transforms(self.Py_translation, [0.0, 0.0, 0.0], 'Py','realsense')
        # Publish marker frame
        self.make_transforms(self.marker_translation, self.marker_rotation, 'marker')
        #Publish marker frame x
        self.make_transforms(self.marker_x_translation, self.marker_x_rotation, 'marker_x')
        #Publish marker frame y
        self.make_transforms(self.marker_y_translation, self.marker_y_rotation, 'marker_y')
        # Publish gripper frame
        #for idx, frame in enumerate(self.frames):
        #   self.make_transforms(self.goal, self.rotation[idx], frame)#self.rotation, frame)
        
        # Publish frame screw
        self.make_transforms(self.goal, self.rotation, self.frame)

def get_rect_points(xywh):
    x0 = xywh[0] - xywh[2] / 2.0
    x1 = xywh[0] + xywh[2] / 2.0
    y0 = xywh[1] - xywh[3] / 2.0
    y1 = xywh[1] + xywh[3] / 2.0

    start_point = (int(x0), int(y0))
    end_point =  (int(x1), int(y1))
    return start_point, end_point

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def main(args=None):
    rclpy.init(args=args)
    image_detection = imageDetection()

    # attendi finchè le immagini sono disponibili
    while rclpy.ok() and (not image_detection.rgb_ok or not image_detection.depth_ok):
        print('wait')
        rclpy.spin_once(image_detection)

    model = YOLO(MODEL_DATA_PATH)

    # TRASFORMARE LE IMMAGINI DA Image FORMAT A IMMAGINE COLORATA
    ###############################################################################################################
    ###############################################################################################################
    ###############################################################################################################    
    rgb_source = IMAGE_PATH #image_detection.rgb_cv2 #IMAGE_PATH
    image_detection.depth = np.loadtxt(DEPTH_FROM_CSV, delimiter=',')#, fmt='%d')
    depth_source = image_detection.depth # <<<<<<<<<<<<<<<<<<<-------------------------------------------------||||||||||||||||
    #print('###############################################')
    #print(depth_source)
    #print('###############################################')

    results = model.predict(source = rgb_source, conf=0.1 ,show=True, max_det=3)
    max_area=0
    min_area=float('inf')

    for result in results:
        #print('Results:')
        #print(result)

        img = result.orig_img.copy() # copia dell'immagine originale
        boxes_class = result.boxes.cls.detach().cpu().numpy().astype(int)   # estrae la classe, come indice intero, delle bounding boxes 
        boxes_conf = result.boxes.conf.detach().cpu().numpy() # estra l'intervallo di confidenza delle boxes
        classes_names = result.names # estrae nomi classi
        for id_box, box in enumerate(result.boxes.xywh): # per ogni bounding box trovata
            xywh_np = box.detach().cpu().numpy() #array numpy con coordinate(x_centro, y_centro, width, heigth)
            area = xywh_np[2]*xywh_np[3]
            # determino marker piu grande
            if area > max_area:
                max_area = area
                idx_origin = id_box
                box_origin = box
            
            # determino marker piu piccolo
            if area < min_area:
                min_area = area
                idx_smallest = id_box
                box_smallest = box

            start_point, end_point = get_rect_points(xywh_np) # converte array in formato per disegnare boxes in openCV
            #print(f"Obj type: {classes_names[boxes_class[id_box]]},  Center: {(xywh_np[0],xywh_np[1])}, Confidence: {boxes_conf[id_box]}")
            #cv2.circle(img,(int(box.detach().cpu().numpy()[0]),int(box.detach().cpu().numpy()[1])),2,(255,0,0),2)
            cv2.rectangle(img, start_point, end_point, color=(0,255,0), thickness=2) # disegna bounding box con openCV

        # determino marker intermedio
        if idx_origin*idx_smallest == 0:
            if idx_smallest==1 or idx_origin==1:
                idx_medium=2
            else:
                idx_medium=1
        else:
            idx_medium = 0
        box_medium=results[0].boxes.xywh[idx_medium]
        
        x_origin = int(box_origin.detach().cpu().numpy()[0])
        y_origin = int(box_origin.detach().cpu().numpy()[1])
        cv2.circle(img,(x_origin,y_origin),2,(0,0,255),2) # red
        x_smallest = int(box_smallest.detach().cpu().numpy()[0])
        y_smallest = int(box_smallest.detach().cpu().numpy()[1])
        cv2.circle(img,(x_smallest,y_smallest),2,(255,0,0),2) #blue
        x_medium = int(box_medium.detach().cpu().numpy()[0])
        y_medium = int(box_medium.detach().cpu().numpy()[1])
        cv2.circle(img,(x_medium,y_medium),2,(0,255,0),2) #green

        # disegno sistema di riferimento
        #cv2.arrowedLine(img, (x_origin,y_origin), (x_medium,y_medium), (255,0,255), 1) # asse x
        #cv2.putText(img, "y", (x_medium,y_medium), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 2, cv2.LINE_AA)
        #cv2.arrowedLine(img, (x_origin,y_origin), (x_smallest,y_smallest), (255,0,255), 1) # asse y
        #cv2.putText(img, "x", (x_smallest,y_smallest), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 2, cv2.LINE_AA)

        #np.savetxt(DEPTH_FILE_NAME, image_detection.depth, delimiter=',' , fmt='%d') #<<<<<<<<-------------
        #cv2.imshow("Infered Image", img)
        
        #cv2.waitKey(10)#-1 3000

        # ricavo parametri intrinseci leggendo il topic camera/color/camera_info
        fx= 613.7064208984375 #m?
        fy= 614.0816650390625 #m?
        cx = 315.96002197265625
        cy = 245.59445190429688

        #print coordinate marker
        #print('coordinate marker in pixel: ')
        #print([x_origin,y_origin])
        #print([x_medium,y_medium])
        #print([x_smallest,y_smallest]) 

        # ricavo z da depth        
        z0 = depth_source[y_origin,x_origin]/1000
        z_y = depth_source[y_medium,x_medium]/1000 # z del punto sull'asse delle x (punto con area media) 
        z_x = depth_source[y_smallest,x_smallest]/1000 # z del punto sull'asse delle y (punto con area piu piccola) 
        #print(z0)

        # trasformo in coordiante cartesiane con parametri intriseci
        x0 = z0*(x_origin-cx)/fx
        y0 = z0*(y_origin-cy)/fy
        x_y = z_y*(x_medium-cx)/fx
        y_y = z_y*(y_medium-cy)/fy
        x_x = z_x*(x_smallest-cx)/fx
        y_x = z_x*(y_smallest-cy)/fy

        # calcolo matrice M
        Po = np.array([z0,-x0,-y0])#np.array([x0,y0,z0]) #n 
        Px = np.array([z_x,-x_x,-y_x])#np.array([x_x,y_x,z_x])
        Py = np.array([z_y,-x_y,-y_y])#np.array([x_y,y_y,z_y])
        image_detection.Po_translation = Po
        image_detection.Px_translation = Px
        image_detection.Py_translation = Py
        # print('3 marker (Po, Px, Py):')
        # print(Po)
        # print(Px)
        # print(Py)
        
        # calcolo versori x, y e z del SR sui marker
        y_axis = (Py - Po)/np.linalg.norm(Py - Po)
        x_axis_first = (Px - Po)
        x_axis_norm = x_axis_first - np.dot(x_axis_first, y_axis)/(np.dot(y_axis,y_axis))*y_axis
        x_axis = x_axis_norm/np.linalg.norm(x_axis_norm)
        z_axis = np.cross(x_axis, y_axis)

        R = np.array([x_axis,y_axis,z_axis]).T
        M = np.array([[R[0,0], R[0,1], R[0,2], Po[0]],
                    [R[1,0], R[1,1], R[1,2], Po[1]],
                    [R[2,0], R[2,1], R[2,2], Po[2]],
                    [0.0, 0.0, 0.0, 1.0]])     
        # print('versori (x,y,z):')
        # print(x_axis)
        # print(y_axis)
        # print(z_axis)
        #print(M)

        # riporto tutto al frame map
        # matrice structure_link_1 -> telecamera
        T1=np.array([[0.924129, -0.0455689, -0.379355, 0.708856],
                    [-0.379709, 0.000954597, -0.925106, 0.242205],
                    [0.0425182, 0.998961, -0.0164207, 2.05499],
                    [0, 0, 0, 1.0]])
        # traslazione guida: 0.933

        # matrice world -> kuka_base_link (traslato)
        T2=np.array([[0.0, 0.0, 1.0, 1.016],
                    [0.0, 1.0000, 0.0 , 0.1800],
                    [-1.0000, 0.0, 0.0 , 3.0450],
                    [0, 0, 0, 1.0]])
        
        # matrice world -> structure_link_1 (traslato)
        #T2=np.array([[1.0, 0.0, 0.0, 0.707],
        #             [0.0, 1.0, 0.0 , 0.215],
        #             [0.0, 0.0, 1.0 , 2.720],
        #             [0, 0, 0, 1.0]])
        
        # matrice world -> map  | controllo rotazioni
        #T3=np.array([[1.0, 0, 0, 2.39],
        #             [0, 1.0, 0, 3.53],
        #             [0, 0, 1.0, 0],
        #             [0, 0, 0, 1.0]])
        
        T = np.dot(T2,T1)
        
        ############## SOLO PER PRINT ##################
        #TPRINT = np.dot((np.dot(T3,T2)),T1)
        #print('T print:')
        #print(TPRINT)
        #T = np.dot((np.dot(T3,T2)),T1)

        # ruoto il frame realsense in modo che sia come voglio io
        T_adjusted1 = np.array([[0.0, 0.0, 1.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [-1.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0]])
        T_adjusted2 = np.array([[0.0, -1.0, 0.0, 0.0],
                                [1.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0]])
        T_adjusted=np.dot(T_adjusted1,T_adjusted2)        
        #T = np.dot(T_,T_adjusted)
        
        # realsense translation and rotation
        R_realsense=T[:3,:3]
        R_realsense = Rot.from_matrix(R_realsense)
        image_detection.realsense_rotation = R_realsense.as_euler('xyz', degrees=False)
        #print(R_realsense_quat)
        T_realsense = T[:, -1]
        image_detection.realsense_translation = T_realsense[:3]

        P = np.dot(T,M)
        #print("QUI QUI ")
        #print(P)
        
        # marker translation and rotation
        R_marker = P[:3,:3]
        R_marker = Rot.from_matrix(R_marker)
        image_detection.marker_rotation = R_marker.as_euler('xyz', degrees=False)
        T_marker = P[:, -1]
        image_detection.marker_translation = T_marker[:3]

        #print("QUI QUI ")

        # marker x translation and rotation
        Px_world = np.dot(T,[Px[0], Px[1], Px[2], 1.0])
        Px_world = Px_world[:3]
        image_detection.marker_x_translation = Px_world
        # marker y translation and rotation
        Py_world = np.dot(T,[Py[0], Py[1], Py[2], 1.0])
        Py_world = Py_world[:3]
        image_detection.marker_y_translation = Py_world   

        #print("QUI QUI ")   

        # coordinate cilindro
        #v0 = np.array([0.030, 0.580, 0.2, 1.0]) # z=50 mm, così afferro il cilindro a metà

        # coordinate viti
        v1 = np.array([0.060, 0.450, 0.460, 1.0]) #coord cilindro
        #v1 = np.array([0.306, 0.596, 0.3, 1.0]) #([0.106, 0.596, 0.04, 1.0]) # rialzate di 0.04 mm per rappresentare posizione TF 'reale' su rviz
        v2 = np.array([0.155, 0.597, 0.04, 1.0]) # rialzate di 0.04 mm per rappresentare posizione TF 'reale' su rviz
        v3 = np.array([0.206, 0.598, 0.04, 1.0]) # 
        
        #v0_map = np.dot(P,v0)
        #v0_map=v0_map[:3]

        # riporto coordinate viti nel SR world
        v1_world = np.dot(P,v1)
        v1_world = v1_world[:3]
        v2_world = np.dot(P,v2)
        v2_world = v2_world[:3]
        v3_world = np.dot(P,v3)
        v3_world = v3_world[:3]

        # ricavo posizione cilindro
        #image_detection.goal = v0_map#T_realsense#v0_map
        ##image_detection.frames=['realsense']
        ##image_detection.rotation=R_realsense_quat

        # ricavo posizione di una vite
        image_detection.goal = v1_world
        #print("QUI QUI ")

        # pubblico TF
        image_detection.timer = image_detection.create_timer(0.1, image_detection.on_timer)
        
        #print("QUI QUI ")
        cv2.destroyAllWindows()
        
        #detected = True
        #msg = Bool()
        #msg.data = detected
        #image_detection.publisher_.publish(msg)
        #image_detection.get_logger().info('Detection status published: %s' % str(detected))

        try:
            rclpy.spin(image_detection)
        except KeyboardInterrupt:
            pass

        image_detection.destroy_node()
        rclpy.shutdown()
        


if __name__ == "__main__":
  main()

