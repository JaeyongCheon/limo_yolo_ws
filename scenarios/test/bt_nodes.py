import math
import os
import cv2

os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveSequence, ReactiveFallback
from modules.base_bt_nodes_ros import ConditionWithROSTopics

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None


# BT Node List (Telegram 제거)
CUSTOM_ACTION_NODES = [
    'CaptureCameraImage',
    'DisplayYOLODetection',
    'CaptureScreen',
]

CUSTOM_CONDITION_NODES = [
    'IsDetectedSomething'
]

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)


# Helper function for IoU calculation
def _calculate_iou(box1, box2):
    """
    IoU(Intersection over Union) 계산 함수.
    두 바운딩 박스의 겹침 정도를 0.0 ~ 1.0으로 반환.
    """
    x1_inter = max(box1[0], box2[0])
    y1_inter = max(box1[1], box2[1])
    x2_inter = min(box1[2], box2[2])
    y2_inter = min(box1[3], box2[3])

    if x2_inter < x1_inter or y2_inter < y1_inter:
        return 0.0

    intersection_area = (x2_inter - x1_inter) * (y2_inter - y1_inter)
    box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union_area = box1_area + box2_area - intersection_area

    if union_area == 0:
        return 0.0

    return intersection_area / union_area


class CaptureCameraImage(ConditionWithROSTopics):
    """
    지정된 카메라 토픽을 구독하여 최신 이미지를 blackboard에 저장.
    - blackboard['camera_image']에 최신 이미지 저장
    - 이미지가 없으면 False (ConditionWithROSTopics 기반이라 RUNNING/FAILURE 해석은 BT 프레임워크에 따름)
    """
    def __init__(self, name, agent, camera_topic="bms"):
        super().__init__(name, agent, [
            (Image, camera_topic, 'camera_image'),
        ])
        self.type = "Action"
        self.camera_topic = camera_topic
        agent.ros_bridge.node.get_logger().info(f'CaptureCameraImage: 구독 시작 - 토픽: {camera_topic}')

    def _predicate(self, agent, blackboard) -> bool:
        cache = self._cache
        if "camera_image" in cache:
            blackboard['camera_image'] = cache["camera_image"]
            agent.ros_bridge.node.get_logger().debug(
                f'CaptureCameraImage: 이미지 수신됨 (토픽: {self.camera_topic})'
            )
            return True
        else:
            if blackboard.get('camera_image') is not None:
                agent.ros_bridge.node.get_logger().warning(
                    f'CaptureCameraImage: 이미지 수신 실패 - 토픽 "{self.camera_topic}"에서 메시지를 받지 못함. blackboard 초기화.'
                )
            blackboard['camera_image'] = None
            return False


# 모듈 레벨 변수로 초기화 상태 관리
_display_yolo_initialized = False
_display_yolo_publisher = None
_display_yolo_yolo_model = None
_display_yolo_window_created = False


class DisplayYOLODetection(Node):
    """
    YOLO 검출을 수행하고 결과를 ROS 토픽으로 퍼블리시하는 노드.
    - blackboard['camera_image']에서 이미지 가져오기
    - YOLO 모델 경로: blackboard['yolo_model_path'] 또는 기본값 'best.pt'
    - ROS 토픽: blackboard['output_topic'] 또는 기본값 '/yolo/image'
    - blackboard에:
        - 'yolo_detections' (리스트)
        - 'current_frame' (원본 frame)
        - 'annotated_frame' (bbox 그려진 frame)
    """
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self._bridge = CvBridge()
        self.type = "Action"

    async def run(self, agent, blackboard):
        global _display_yolo_initialized, _display_yolo_publisher
        global _display_yolo_yolo_model, _display_yolo_window_created

        model_path = blackboard.get('yolo_model_path', 'best.pt')
        output_topic = blackboard.get('output_topic', '/yolo/image')

        if not _display_yolo_initialized:
            if not self._initialize(model_path, output_topic):
                self.status = Status.FAILURE
                return self.status

        camera_image_msg = blackboard.get('camera_image')
        if camera_image_msg is None:
            self.ros.node.get_logger().warning(
                'No camera image in blackboard yet - DisplayYOLODetection cannot display'
            )
            self.status = Status.RUNNING
            return self.status

        try:
            frame = self._bridge.imgmsg_to_cv2(camera_image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.ros.node.get_logger().error(f'Failed to convert image: {e}', exc_info=True)
            self.status = Status.FAILURE
            return self.status

        try:
            results = _display_yolo_yolo_model(frame, verbose=False)
            res = results[0]

            detections = self._parse_detections(res)

            annotated_frame = res.plot()
            blackboard['yolo_detections'] = detections
            blackboard['current_frame'] = frame
            blackboard['annotated_frame'] = annotated_frame

            try:
                cv2.imshow("YOLO Detection", annotated_frame)
                cv2.waitKey(1)
            except Exception as e:
                self.ros.node.get_logger().error(f'Failed to display OpenCV window: {e}', exc_info=True)

            self._publish_frame(annotated_frame)

            self.status = Status.SUCCESS
            return self.status

        except Exception as e:
            self.ros.node.get_logger().error(f'YOLO processing failed: {e}', exc_info=True)
            self.status = Status.FAILURE
            return self.status

    def _initialize(self, model_path, output_topic):
        global _display_yolo_initialized, _display_yolo_publisher
        global _display_yolo_yolo_model, _display_yolo_window_created

        try:
            if YOLO is None:
                self.ros.node.get_logger().error(
                    'ultralytics YOLO package not found. Install `ultralytics` to use this node.'
                )
                return False

            _display_yolo_yolo_model = YOLO(model_path)
            self.ros.node.get_logger().info(f'YOLO model loaded: {model_path}')

            _display_yolo_publisher = self.ros.node.create_publisher(Image, output_topic, 10)

            try:
                cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)
                _display_yolo_window_created = True
                self.ros.node.get_logger().info('OpenCV window "YOLO Detection" created successfully')
            except Exception as e:
                self.ros.node.get_logger().error(f'Failed to create OpenCV window: {e}', exc_info=True)
                return False

            _display_yolo_initialized = True
            self.ros.node.get_logger().info(
                f'YOLO Detection initialized: model={model_path}, topic={output_topic}'
            )
            return True

        except Exception as e:
            self.ros.node.get_logger().error(f'Initialization failed: {e}', exc_info=True)
            return False

    def _parse_detections(self, res):
        detections = []
        if hasattr(res, 'boxes') and res.boxes is not None:
            try:
                boxes = res.boxes.xyxy.cpu().numpy()
                confs = res.boxes.conf.cpu().numpy()
                cls_inds = res.boxes.cls.cpu().numpy().astype(int)

                for box, conf, cls_i in zip(boxes, confs, cls_inds):
                    class_name = (
                        _display_yolo_yolo_model.names[int(cls_i)]
                        if hasattr(_display_yolo_yolo_model, 'names')
                        else str(int(cls_i))
                    )
                    detections.append({
                        'class': class_name,
                        'confidence': float(conf),
                        'bbox': [float(box[0]), float(box[1]), float(box[2]), float(box[3])]
                    })
            except Exception as e:
                self.ros.node.get_logger().error(f'Failed to parse detections: {e}', exc_info=True)
        return detections

    def _publish_frame(self, annotated_frame):
        try:
            msg = self._bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            msg.header.stamp = self.ros.node.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            _display_yolo_publisher.publish(msg)
        except Exception as e:
            self.ros.node.get_logger().error(f'Failed to publish: {e}', exc_info=True)

    def halt(self):
        pass


# IsDetectedSomething 노드의 이전 바운딩박스 및 마지막 감지 시간 추적
_is_detected_something_previous_boxes = {}  # {node_name: [(bbox, timestamp), ...]}
_is_detected_something_last_detection = {}  # {node_name: timestamp}


class IsDetectedSomething(Node):
    """
    blackboard에서 YOLO 검출 결과를 확인하여 새로운 객체가 감지되었는지 확인.
    - XML 파라미터 target_class (기본값 "person")
    - confidence_threshold, cooldown_time, iou_threshold 지원
    """
    def __init__(self, name, agent, target_class="person", confidence_threshold=0.5,
                 cooldown_time=10.0, iou_threshold=0.3):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.target_class = target_class.lower()
        self.default_confidence_threshold = confidence_threshold
        self.cooldown_time = float(cooldown_time)
        self.iou_threshold = float(iou_threshold)
        self.type = "Condition"

        agent.ros_bridge.node.get_logger().info(
            f'IsDetectedSomething: 감지 대상 = "{self.target_class}", '
            f'confidence = {confidence_threshold}, cooldown = {cooldown_time}초, '
            f'iou_threshold = {iou_threshold}'
        )

    async def run(self, agent, blackboard):
        import time
        global _is_detected_something_previous_boxes
        global _is_detected_something_last_detection

        detections = blackboard.get('yolo_detections', [])

        threshold_key = f'{self.target_class}_confidence_threshold'
        threshold = blackboard.get(threshold_key, self.default_confidence_threshold)

        detected_objects = []
        for detection in detections:
            class_name = detection.get('class', '').lower()
            if class_name == self.target_class:
                confidence = detection.get('confidence', 0.0)
                if confidence >= threshold:
                    detected_objects.append(detection)

        if not detected_objects:
            self.status = Status.FAILURE
            return self.status

        current_time = time.time()

        if self.name not in _is_detected_something_previous_boxes:
            _is_detected_something_previous_boxes[self.name] = []

        previous_boxes = _is_detected_something_previous_boxes[self.name]

        new_objects = []
        for obj in detected_objects:
            bbox = obj.get('bbox')
            if not bbox:
                continue

            is_new = True
            for prev_bbox, prev_time in previous_boxes:
                iou = _calculate_iou(bbox, prev_bbox)
                if iou > self.iou_threshold:
                    is_new = False
                    break

            if is_new:
                new_objects.append(obj)

        if not new_objects:
            self.status = Status.FAILURE
            return self.status

        last_detection_time = _is_detected_something_last_detection.get(self.name, 0)
        time_since_last = current_time - last_detection_time

        if time_since_last < self.cooldown_time:
            self.status = Status.FAILURE
            return self.status

        best_object = max(new_objects, key=lambda x: x.get('confidence', 0.0))
        best_bbox = best_object.get('bbox')

        blackboard[f'detected_{self.target_class}'] = best_object
        blackboard[f'{self.target_class}_confidence'] = best_object.get('confidence', 0.0)

        _is_detected_something_previous_boxes[self.name].append((best_bbox, current_time))

        cutoff_time = current_time - (self.cooldown_time * 2)
        _is_detected_something_previous_boxes[self.name] = [
            (bbox, timestamp)
            for bbox, timestamp in _is_detected_something_previous_boxes[self.name]
            if timestamp > cutoff_time
        ]

        _is_detected_something_last_detection[self.name] = current_time

        self.ros.node.get_logger().info(
            f'새로운 {self.target_class} 감지! (신뢰도: {best_object.get("confidence", 0.0):.2f})'
        )

        self.status = Status.SUCCESS
        return self.status


class CaptureScreen(Node):
    """
    현재 프레임을 이미지 파일로 저장.
    - blackboard['annotated_frame'] 또는 blackboard['current_frame']에서 프레임 가져오기
    - 저장 경로: blackboard['save_path'] 또는 기본값 '<this_file_dir>/captured_images'
    """
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
        self.default_save_dir = os.path.join(os.path.dirname(__file__), 'captured_images')

    async def run(self, agent, blackboard):
        save_dir = blackboard.get('save_path', self.default_save_dir)
        os.makedirs(save_dir, exist_ok=True)

        frame = blackboard.get('annotated_frame')
        if frame is None:
            frame = blackboard.get('current_frame')

        if frame is None:
            self.ros.node.get_logger().warn('No frame available in blackboard for capture')
            self.status = Status.FAILURE
            return self.status

        try:
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]

            detected_object = None
            object_class = None

            # detected_* 중 아무거나 잡아서 파일명에 반영
            for key in blackboard.keys():
                if key.startswith('detected_'):
                    detected_object = blackboard.get(key)
                    object_class = key.replace('detected_', '')
                    break

            if detected_object and object_class:
                confidence = detected_object.get('confidence', 0.0)
                filename = os.path.join(
                    save_dir, f"captured_{object_class}_{confidence:.2f}_{timestamp}.jpg"
                )
            else:
                filename = os.path.join(save_dir, f"captured_frame_{timestamp}.jpg")

            cv2.imwrite(filename, frame)

            blackboard['last_captured_image'] = filename
            blackboard['capture_result'] = 'succeeded'

            self.ros.node.get_logger().info(f'Image saved: {filename}')
            self.status = Status.SUCCESS
            return self.status

        except Exception as e:
            self.ros.node.get_logger().error(f'Failed to save image: {e}', exc_info=True)
            self.status = Status.FAILURE
            return self.status
