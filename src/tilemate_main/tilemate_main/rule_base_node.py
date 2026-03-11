import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

# 🌟 커스텀 메시지 배열 임포트
from custom_tile_msgs.msg import TileArray, InspectionResult, InspectionResultArray

class RuleBasedAnomalyNode(Node):
    def __init__(self):
        super().__init__('rule_based_anomaly_node')
        self.bridge = CvBridge()

        # ==========================================
        # 💡 1. 세팅 및 9개 패턴별 안전지대 생성
        # ==========================================
        
        # 🔥 본인의 YOLO CLS 모델이 뱉는 '패턴 이름'과 '기준 이미지 경로'를 매칭해주세요!
        self.ref_image_paths = {
            "pattern_1": "/home/rokey/kmg/rule_based_img/ref_pattern_1.jpg",
            "pattern_2": "/home/rokey/kmg/rule_based_img/ref_pattern_2.jpg",
            "pattern_3": "/home/rokey/kmg/rule_based_img/ref_pattern_3.jpg",
            "pattern_4": "/home/rokey/kmg/rule_based_img/ref_pattern_4.jpg",
            "pattern_5": "/home/rokey/kmg/rule_based_img/ref_pattern_5.jpg",
            "pattern_6": "/home/rokey/kmg/rule_based_img/ref_pattern_6.jpg",
            "pattern_7": "/home/rokey/kmg/rule_based_img/ref_pattern_7.jpg",
            "pattern_8": "/home/rokey/kmg/rule_based_img/ref_pattern_8.jpg",
            "pattern_9": "/home/rokey/kmg/rule_based_img/ref_pattern_9.jpg",
        }

        # 생성된 안전지대를 저장할 딕셔너리
        self.safe_zones = {}
        kernel = np.ones((11,11), np.uint8) 
        self.pixel_threshold = 60

        # 딕셔너리를 돌면서 각 패턴별로 Canny 엣지 + 팽창(Dilate) 수행
        for pattern_name, img_path in self.ref_image_paths.items():
            if not os.path.exists(img_path):
                self.get_logger().warn(f"🚨 기준 이미지 없음: {img_path} ({pattern_name} 검사 불가)")
                continue
            
            ref_img = cv2.imread(img_path)
            ref_edges = self.get_canny_edges(ref_img)
            self.safe_zones[pattern_name] = cv2.dilate(ref_edges, kernel, iterations=1)

        self.get_logger().info(f"✅ {len(self.safe_zones)}개 패턴의 안전지대 로드 완료! (허용 기준: {self.pixel_threshold}픽셀)")

        # 2. 통신망 세팅
        self.tile_sub = self.create_subscription(TileArray, '/yolo/tile_array', self.inference_callback, 10)
        self.web_pub = self.create_publisher(InspectionResultArray, '/web/inspection_results', 10)
        self.debug_pub = self.create_publisher(Image, '/anomaly/debug_rule_based', 10)

    # 🛠️ Canny 전처리 함수 (유지)
    def get_canny_edges(self, cv_img):
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5,5), 0)
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)) 
        gray_clahe = clahe.apply(blurred) 
        edges = cv2.Canny(gray_clahe, 35, 95) 

        h, w = edges.shape
        cv2.rectangle(edges, (0, 0), (w - 1, h - 1), 0, thickness=5)
        return edges

    def inference_callback(self, msg):
        try:
            result_array_msg = InspectionResultArray()
            result_array_msg.header = msg.header
            result_array_msg.total_tiles = len(msg.tiles)

            for idx, tile in enumerate(msg.tiles):
                
                # 🌟 타일의 패턴 이름 가져오기
                current_pattern = tile.pattern_name
                
                # 웹으로 보낼 기본 결과 폼
                res_msg = InspectionResult()
                res_msg.tile_id = idx + 1
                res_msg.pattern_name = current_pattern
                res_msg.pose = tile.pose      
                res_msg.width = tile.width    
                res_msg.height = tile.height  

                # 🌟 패턴에 맞는 안전지대가 있는지 확인
                if current_pattern not in self.safe_zones:
                    self.get_logger().warn(f"⚠️ 등록되지 않은 패턴({current_pattern}) 감지! 검사 생략.")
                    res_msg.is_defective = False
                    res_msg.defect_type = "Unknown Pattern"
                    result_array_msg.results.append(res_msg)
                    continue

                # 🌟 해당 패턴 전용 안전지대(Safe Zone) 꺼내오기!
                target_safe_zone = self.safe_zones[current_pattern]

                # 1. 이미지 변환
                curr_img = self.bridge.imgmsg_to_cv2(tile.cropped_image, desired_encoding='bgr8')
                
                # 🌟 강제 리사이즈 기준을 target_safe_zone의 모양으로 설정
                curr_img = cv2.resize(curr_img, (target_safe_zone.shape[1], target_safe_zone.shape[0]))

                # 2. 엣지 추출
                curr_edges = self.get_canny_edges(curr_img)

                # 3. 🌟 빼기 연산 (현재 엣지 - 타겟 패턴의 안전지대 마스크)
                defect_edges = cv2.subtract(curr_edges, target_safe_zone)

                # 4. 불량 판정
                defect_pixel_count = np.sum(defect_edges > 0)
                is_defect = bool(defect_pixel_count > self.pixel_threshold)

                # 5. 결과 기록
                res_msg.is_defective = is_defect
                res_msg.defect_type = "Crack" if is_defect else "None"
                result_array_msg.results.append(res_msg)

                # 6. 시각화 
                curr_img_debug = curr_img.copy()
                curr_img_debug[defect_edges > 0] = [0, 0, 255] 
                
                debug_msg = self.bridge.cv2_to_imgmsg(curr_img_debug, encoding="bgr8")
                self.debug_pub.publish(debug_msg)

                # 로그 출력
                status = "🚨 폐기" if is_defect else "✅ 정상"
                self.get_logger().info(f"[{current_pattern}] 타일 {idx+1}: {status} | 크랙: {defect_pixel_count}")

            # 7. 웹 전송
            if len(result_array_msg.results) > 0:
                self.web_pub.publish(result_array_msg)

        except Exception as e:
            self.get_logger().error(f"🚨 룰 베이스 추론 중 에러: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RuleBasedAnomalyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()