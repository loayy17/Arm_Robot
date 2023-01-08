# import necessary packages
from pydoc import classname
import cv2
import numpy as np
import mediapipe as mp
import tensorflow as tf
from tensorflow.keras.models import load_model
from sqlite3 import Time
from time import clock_gettime
from std_msgs.msg import Header
from operator import index
from PIL import Image, ImageOps
from rclpy.node import Node
from sensor_msgs.msg import JointState
import rclpy
# initialize mediapipe
mpHands = mp.solutions.hands
hands = mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mpDraw = mp.solutions.drawing_utils

# Load the gesture recognizer model
model = load_model('/home/loay/loay_ws/src/arm_project/python+model/mp_hand_gesture') # replace path 

# Load class names
f = open('/home/loay/loay_ws/src/arm_project/python+model/gesture.names', 'r')
classNames = f.read().split('\n')
f.close()


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('classes')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 2     
        self.video = cv2.VideoCapture(0)# seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        
        base_inside = 0. #initial value for first joint as float value
        inside_arm = 0.#initial value for second joint as float value
        rl_r1 = 0.#initial value for third joint as float value
        rl_l1 = 0.
# Initialize the webcam
        
        

        
        while True:
            # Read each frame from the webcam
            _, frame = self.video.read()
            x, y, c = frame.shape
            msg_str = JointState() #save msg in variable 
            msg_str.header = Header() #save msg in variable 
            msg_str.name = ['base_inside', 'inside_arm', 'rl_r1' ,'rl_l1'] #called names of joints
            msg_str.velocity = [] 
            msg_str.effort = []
            # Flip the frame vertically
            frame = cv2.flip(frame, 1)
            framergb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Get hand landmark prediction
            result = hands.process(framergb)

            # print(result)
            
            className = ''
            
            # post process the result
            if result.multi_hand_landmarks:
                landmarks = []
                for handslms in result.multi_hand_landmarks:
                    for lm in handslms.landmark:
                        # print(id, lm)
                        lmx = int(lm.x * x)
                        lmy = int(lm.y * y)

                        landmarks.append([lmx, lmy])

                    # Drawing landmarks on frames
                    mpDraw.draw_landmarks(frame, handslms, mpHands.HAND_CONNECTIONS)

                    # Predict gesture
                    prediction = model.predict([landmarks])
                    # print(prediction)
                    classID = np.argmax(prediction)
                    className = classNames[classID]
                    
            if className=='okay':
                print("ccw")
                inside_arm -= 0.3 
                className="ccw"
                if inside_arm <-3.14:
                    inside_arm =-3.14 
                print(inside_arm)
            if className=='peace':
                print("cw") 
                inside_arm += 0.3
                className="cw" 
                if inside_arm >0:
                    inside_arm =0 
                print(inside_arm)  
            if className=='rock':
                print("close_gripper") 
                rl_r1 -= 0.1 
                rl_l1 +=0.1
                className="close_gripper"
                if rl_r1 <-0.523:
                    rl_r1 =-0.523
                if rl_l1 >0.523:
                    rl_l1 =0.523
                print(rl_r1)  
                print(rl_l1) 
            if className=='stop':
                print("open_gripper")
                rl_r1 += 0.1 
                rl_l1 -=0.1
                className="open_gripper"
                if rl_r1 >0:
                    rl_r1 =0
                if rl_l1 <0:
                    rl_l1 =0
                print(rl_r1)  
                print(rl_l1) 
            if className=='thumbs up':     
                print("Up")
                base_inside += 0.05 
                className="Up" 
                if base_inside >0.25:
                    base_inside =0.25 
                print(base_inside)
            if className=='thumbs down': 
                print("down")
                base_inside -= 0.05 
                className="down"  
                if base_inside <0:
                    base_inside =0 
                print(base_inside)
            if className=="fist":
                className=""
            if className=="smile":
                className=""   
            if className=="live long":
                className=""  
            if className=="call me":
                className=""         
            msg_str.header.stamp =self.get_clock().now().to_msg() #set stamp to time execute
            msg_str.position = [float(base_inside), float(inside_arm), float(rl_r1),float(rl_l1)]
            #the value of position must be float
            self.publisher_.publish(msg_str)               
            # show the prediction on the frame
            cv2.putText(frame, className, (250, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                        1, (0,0,255), 2, cv2.LINE_AA)

            # Show the final output
            cv2.imshow("Output", frame) 

            if cv2.waitKey(1) == ord('q'):
                break
            
        # release the webcam and destroy all active windows
        self.video.release()

cv2.destroyAllWindows()
def main(args=None):
  
  rclpy.init(args=args)
  
  image_publisher = ImagePublisher()
  

  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()