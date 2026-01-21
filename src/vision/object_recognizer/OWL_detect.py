# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from vision_interfaces.srv import Owlv2Service
import cv2
import torch
from transformers import Owlv2Processor, Owlv2ForObjectDetection

# dictionary of possible aliases for known objects
OBJECT_ALIASES = {
    "coffee mate": [
        "red bottle",
        "bottle",
        "plastic bottle",
        "creamer bottle"
    ],
    "sprite": [
        "green bottle",
        "soda can",
        "green can"
    ],
    "pringles": [
        "chips can",
        "cylindrical can",
        "snack can"
    ],
}

def normalize_label(detected_label: str):
    detected_label = detected_label.lower()

    for official_name, aliases in OBJECT_ALIASES.items():
        for a in aliases:
            if a in detected_label:
                return official_name

    return detected_label


class OwlServer(Node):
    def __init__(self):
        super().__init__('owl_server')

        self.bridge = CvBridge()
        self.last_image = None
        #models, suscription and service
        self.create_subscription(Image, "/kinect/image_raw", self.cb_image, 10)
        self.debug_pub = self.create_publisher(Image, "/owl/debug_image", 10)

        self.get_logger().info("Loading OWLv2...")
        self.processor = Owlv2Processor.from_pretrained("google/owlv2-base-patch16")
        self.model = Owlv2ForObjectDetection.from_pretrained(
            "google/owlv2-base-patch16"
        ).to("cuda").half().eval()
        self.get_logger().info("OWLv2 loaded on GPU")

        self.srv = self.create_service(Owlv2Service, "classifyowl", self.callback)
        self.get_logger().info("OWL service ready.")

    def cb_image(self, msg):
        self.last_image = msg

    def callback(self, req, resp):
        self.get_logger().info(f"OWL request with {len(req.queries)} queries")

        resp.poses = []
        resp.names = []
        resp.pt_min = []
        resp.pt_max = []
        resp.confidence = []
        resp.debug_images = []

        if self.last_image is None:
            self.get_logger().warn("No image received yet")
            return resp

        # Convertir imagen
        cv_img = self.bridge.imgmsg_to_cv2(self.last_image,desired_encoding= "bgr8")
        img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        texts = [req.queries]

        inputs = self.processor(text=texts, images=img_rgb, return_tensors="pt").to("cuda")

        with torch.no_grad():
            outputs = self.model(**inputs)

        target_sizes = torch.tensor([img_rgb.shape[:2]]).to("cuda")
        results = self.processor.post_process_object_detection(
            outputs, threshold=0.2, target_sizes=target_sizes
        )[0]

        debug_img = img_rgb.copy()

        if len(results["boxes"]) == 0:
            self.get_logger().info("No detections")
            debug_bgr = cv2.cvtColor(debug_img, cv2.COLOR_RGB2BGR)
            msg = self.bridge.cv2_to_imgmsg(debug_bgr, encoding="bgr8")
            self.debug_pub.publish(msg)
            resp.debug_images.append(msg)
            return resp
        
        best_per_class = {}

        for box, score, label in zip(results["boxes"], results["scores"], results["labels"]):
            conf = float(score.cpu())
            cls = int(label.cpu())
            #best for class
            if conf < 0.4:
                continue

            if cls not in best_per_class or conf > best_per_class[cls]["conf"]:
                best_per_class[cls] = {
                    "box": box,
                    "conf": conf
                }
        if len(best_per_class) == 0:
            self.get_logger().warn("NO CONFIABLE DETECTIONS")
            return resp
        
        for cls, data in best_per_class.items():
            box = data["box"]
            conf = data["conf"]

            x1, y1, x2, y2 = box.int().cpu().numpy()
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            
            self.get_logger().info(f"Detected {req.queries[cls]} at ({cx}, {cy}) with conf {conf:.2f}")

            raw_name = req.queries[int(cls)]
            class_name = normalize_label(raw_name)
            # bbox
            cv2.rectangle(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.circle(debug_img, (cx, cy), 5, (0,0,255), -1)
            cv2.putText(debug_img, f"{class_name} {conf:.2f}", (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            
            p = Pose()
            p.orientation.w = 1.0
            

            resp.poses.append(p)
            resp.names.append(class_name)
            resp.pt_min.extend([float(x1), float(y1)])
            resp.pt_max.extend([float(x2), float(y2)])
            resp.confidence.append(conf)
            resp.cx.extend([float(cx)])
            resp.cy.extend([float(cy)])

        """
        only one detection - best score

        scores = results["scores"]
        best_idx = torch.argmax(scores)
        box = results["boxes"][best_idx]
        score = results["scores"][best_idx]
        label = results["labels"][best_idx]

        x1, y1, x2, y2 = box.int().cpu().numpy()
        conf = float(score.cpu())
        if conf < 0.3:
            self.get_logger().warn("NO CONFIABLE DETECTIONS")
            return resp 
        class_name = req.queries[int(label)]

        # Dibujar bbox
        cv2.rectangle(debug_img, (x1,y1), (x2,y2), (0,255,0), 2)
        cv2.putText(debug_img, f"{class_name} {conf:.2f}", (x1, y1-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # Pose vacía (para no romper tu servicio)
        p = Pose()
        p.orientation.w = 1.0

        resp.poses.append(p)
        resp.names.append(class_name)
        resp.pt_min.extend([float(x1), float(y1)])
        resp.pt_max.extend([float(x2), float(y2)])
        resp.confidence.append(conf)"""

        # imagen debug
        debug_bgr = cv2.cvtColor(debug_img, cv2.COLOR_RGB2BGR)
        msg = self.bridge.cv2_to_imgmsg(debug_bgr, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "kinect_rgb"
        self.debug_pub.publish(msg)
        resp.debug_images.append(msg)

        return resp


def main():
    rclpy.init()
    node = OwlServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()




