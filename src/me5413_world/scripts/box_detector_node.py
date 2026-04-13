#!/usr/bin/env python3
"""
Box Detector Node
Detects numbered boxes (1-9) from the robot's camera feed using OCR,
tracks unique boxes by position to avoid double counting,
and publishes the count of each number.
"""

import rospy
import cv2
import numpy as np
import pytesseract
import tf2_ros
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class BoxDetector:
    def __init__(self):
        rospy.init_node('box_detector_node')

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Tracked boxes: list of (number, x, y) in map frame
        self.detected_boxes = []
        self.dedup_distance = 2.0  # meters

        # Detection timing
        self.last_detection_time = rospy.Time.now()
        self.detection_interval = rospy.Duration(0.3)

        # Handover to C++: write rarest digit to a file for easy IPC.
        # Also auto-exit when no new box has been confirmed for
        # `idle_timeout` seconds, so the C++ caller can block on exit.
        self.output_file = rospy.get_param(
            '~output_file', '/tmp/rarest_box.txt')
        self.idle_timeout = rospy.Duration(
            rospy.get_param('~idle_timeout', 45.0))
        self.min_boxes_before_exit = rospy.get_param(
            '~min_boxes_before_exit', 1)
        self.last_confirm_time = rospy.Time.now()
        rospy.on_shutdown(self.write_result_file)

        # Box region constraints
        self.min_box_area = 2000
        self.max_box_area = 80000

        # Confidence: require N consistent readings before logging a box
        self.pending_detections = {}  # key: (digit, rounded_x, rounded_y) -> count
        self.confirm_threshold = 3

        # Publishers
        self.count_pub = rospy.Publisher(
            '/box_detector/counts', String, queue_size=10, latch=True)
        self.debug_img_pub = rospy.Publisher(
            '/box_detector/debug_image', Image, queue_size=1)

        # Subscriber
        self.image_sub = rospy.Subscriber(
            '/front/image_raw', Image, self.image_callback, queue_size=1,
            buff_size=2**24)

        rospy.loginfo("Box Detector Node started")

    def get_robot_position(self):
        """Get robot position in map frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rospy.Time(0), rospy.Duration(0.5))
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return (x, y)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def is_duplicate(self, number, pos):
        """Check if this box was already detected nearby."""
        for (det_num, det_x, det_y) in self.detected_boxes:
            if det_num == number:
                dist = np.hypot(pos[0] - det_x, pos[1] - det_y)
                if dist < self.dedup_distance:
                    return True
        return False

    def find_box_regions(self, frame):
        """
        Detect candidate box face regions using edge detection.
        Finds rectangular contours that could be box faces with digits.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Use Canny edge detection — works regardless of absolute brightness
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 30, 100)

        # Dilate edges to close gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        edges = cv2.dilate(edges, kernel, iterations=2)

        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        regions = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (self.min_box_area < area < self.max_box_area):
                continue

            # Approximate to polygon — box face should be roughly rectangular
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            if len(approx) < 4 or len(approx) > 8:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            bbox_area = w * h
            if bbox_area > self.max_box_area or bbox_area < self.min_box_area:
                continue
            aspect = float(w) / h
            if not (0.4 < aspect < 2.5):
                continue

            # Check that the region has contrast (not a plain wall)
            roi_gray = gray[y:y+h, x:x+w]
            std_dev = roi_gray.std()
            if std_dev < 20:
                # Too uniform — likely a plain wall, not a box with a digit
                continue

            regions.append((x, y, w, h))

        return regions

    def ocr_digit(self, roi):
        """
        Run Tesseract OCR on a cropped box face ROI.
        Returns detected digit (1-9) or None.
        """
        if roi.size == 0:
            return None

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if len(roi.shape) == 3 else roi

        # Fixed threshold at 80: Gazebo box faces have gray bg (~128)
        # and black digits (~0). Otsu fails here because the gray bg
        # is too close to the digit intensity.
        _, binary = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)

        # Scale up 3x for better OCR accuracy
        binary = cv2.resize(binary,
                            (binary.shape[1] * 3, binary.shape[0] * 3),
                            interpolation=cv2.INTER_CUBIC)

        # Add white border padding for Tesseract
        binary = cv2.copyMakeBorder(
            binary, 30, 30, 30, 30,
            cv2.BORDER_CONSTANT, value=255)

        # OCR with Tesseract — single line mode works better than single char
        config = '--psm 7 -c tessedit_char_whitelist=123456789'
        try:
            result = pytesseract.image_to_string(binary, config=config).strip()
        except Exception:
            return None

        if result and len(result) == 1 and result.isdigit():
            digit = int(result)
            if 1 <= digit <= 9:
                return digit

        return None

    def confirm_detection(self, digit, pos):
        """
        Require multiple consistent detections before confirming a box.
        Returns True when confirmed.
        """
        # Round position to 1m grid for grouping nearby detections
        key = (digit, round(pos[0]), round(pos[1]))

        self.pending_detections[key] = self.pending_detections.get(key, 0) + 1

        if self.pending_detections[key] >= self.confirm_threshold:
            del self.pending_detections[key]
            return True
        return False

    def compute_counts(self):
        counts = {}
        for (num, x, y) in self.detected_boxes:
            counts[num] = counts.get(num, 0) + 1
        return counts

    def rarest_digit(self, counts):
        if not counts:
            return None
        # Tie-break: lowest count, then lowest digit (deterministic).
        return min(counts.items(), key=lambda kv: (kv[1], kv[0]))[0]

    def publish_counts(self):
        """Publish current box counts."""
        counts = self.compute_counts()
        if not counts:
            return

        sorted_counts = sorted(counts.items())
        count_str = ", ".join([f"Box {n}: {c}" for n, c in sorted_counts])
        total = sum(counts.values())
        rarest_num = self.rarest_digit(counts)
        msg = f"Total: {total} | {count_str} | Rarest: {rarest_num}"

        self.count_pub.publish(String(data=msg))
        rospy.loginfo(msg)

    def write_result_file(self):
        """Write rarest digit to file for C++ handover. Called on shutdown."""
        counts = self.compute_counts()
        rarest = self.rarest_digit(counts)
        if rarest is None:
            rospy.logwarn("No boxes confirmed; writing 0 to %s",
                          self.output_file)
            payload = "0\n"
        else:
            payload = f"{rarest}\n"
        try:
            with open(self.output_file, 'w') as f:
                f.write(payload)
            rospy.loginfo("Wrote rarest=%s to %s",
                          payload.strip(), self.output_file)
        except OSError as e:
            rospy.logerr("Failed to write %s: %s", self.output_file, e)

    def image_callback(self, msg):
        """Process camera frames."""
        now = rospy.Time.now()
        if (now - self.last_detection_time) < self.detection_interval:
            return
        self.last_detection_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr_throttle(5, f"CV Bridge error: {e}")
            return

        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return

        regions = self.find_box_regions(frame)
        debug_frame = frame.copy()

        # Diagnostic: log image stats
        gray_diag = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rospy.loginfo_throttle(3,
            f"Frame {frame.shape}, regions: {len(regions)}, "
            f"gray min/max/mean: {gray_diag.min()}/{gray_diag.max()}/{gray_diag.mean():.0f}, "
            f"pixels>190: {(gray_diag>190).sum()}, pixels>150: {(gray_diag>150).sum()}")

        for (x, y, w, h) in regions:
            # Crop with slight inward padding to avoid box edges
            pad_x = int(w * 0.1)
            pad_y = int(h * 0.1)
            roi = frame[y + pad_y:y + h - pad_y, x + pad_x:x + w - pad_x]

            if roi.size == 0:
                continue

            # Save debug crops periodically
            if not hasattr(self, '_debug_count'):
                self._debug_count = 0
            if self._debug_count < 10:
                cv2.imwrite(f'/home/rosuser/catkin_ws/debug_roi_{self._debug_count}.png', roi)
                self._debug_count += 1

            digit = self.ocr_digit(roi)
            rospy.loginfo_throttle(2, f"Region ({x},{y},{w},{h}) -> OCR result: {digit}")

            if digit is not None:
                # Draw candidate on debug image
                cv2.rectangle(debug_frame, (x, y), (x + w, y + h),
                              (0, 255, 255), 2)
                cv2.putText(debug_frame, f"{digit}?", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                if self.is_duplicate(digit, robot_pos):
                    continue

                if self.confirm_detection(digit, robot_pos):
                    self.detected_boxes.append(
                        (digit, robot_pos[0], robot_pos[1]))
                    self.last_confirm_time = rospy.Time.now()
                    rospy.loginfo(
                        f"CONFIRMED BOX: {digit} at "
                        f"({robot_pos[0]:.2f}, {robot_pos[1]:.2f})")
                    self.publish_counts()

                    # Green box for confirmed
                    cv2.rectangle(debug_frame, (x, y), (x + w, y + h),
                                  (0, 255, 0), 3)
                    cv2.putText(debug_frame, str(digit), (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            self.debug_img_pub.publish(debug_msg)
        except Exception:
            pass


    def run(self):
        """Spin with an idle-timeout watchdog so the node self-terminates."""
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            counts = self.compute_counts()
            total = sum(counts.values())
            idle = rospy.Time.now() - self.last_confirm_time
            if (total >= self.min_boxes_before_exit
                    and idle > self.idle_timeout):
                rospy.loginfo(
                    "Idle %.1fs since last confirmation; finishing.",
                    idle.to_sec())
                rospy.signal_shutdown("box detection idle timeout")
                break
            rate.sleep()


if __name__ == '__main__':
    try:
        detector = BoxDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
    # rospy.on_shutdown hook writes the result file; exit(0) for C++ caller.
    import sys
    sys.exit(0)
