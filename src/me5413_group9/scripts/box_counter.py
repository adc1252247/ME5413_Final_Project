#!/usr/bin/env python3
import math, os
import rospy, cv2, numpy as np, pytesseract
import tf2_ros, actionlib
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler

BOX_SIZE = 0.8
MIN_BOX_SPACING = 1.5
WORLD_SPAWN_X = (-18.0, -2.0)
WORLD_SPAWN_Y = (-8.0, 8.0)

SWEEP_WAYPOINTS_WORLD = [
    (-17, -7,  math.pi / 2),   # enter bottom-left
    (-17,  6,  0),              # up left wall
    (-13,  6, -math.pi / 2),    # step into Room 1 interior, face south
    (-13, -6,  0),              # sweep Room 1 down-center, face east
    (-9,  -6,  math.pi / 2),    # cross partition, face north
    (-9,   6,  0),              # Room 2 left-center, face east
    (-5,   6, -math.pi / 2),    # top-right corner
    (-5,  -7,  math.pi),        # down right wall
    (-15, -10, 0),              # exit
]


class BoxCounter:
    def __init__(self):
        rospy.init_node('box_counter_node')
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # map baseline
        self.wall_mask = None
        self.map_info = None
        self.map_loaded = False
        self.wall_buffer_m = 0.15

        # lidar point accumulation
        self.new_object_points = []
        self.collecting_points = False
        self.live_boxes = []
        self.live_merge_dist = 2.0

        # detected boxes
        self.box_positions = []
        self.box_digits = []

        # camera detections
        self.camera_detections = []
        self.ocr_dedup_dist = 3.5
        self.ocr_confirm = {}
        self.ocr_confirm_radius = 2.0
        self.ocr_confirm_threshold = 3
        self.ocr_active = False
        self.last_ocr_time = None

        # clustering
        self.cluster_eps = 0.3
        self.cluster_min_pts = 3
        self.box_extent_min = 0.1
        self.box_extent_max = 1.0
        self.grid_cell = 0.05

        # digit detection
        self.latest_frame = None
        self.match_threshold = 0.35
        self.match_margin = 0.03
        self.templates = {}
        self.template_scales = [0.2, 0.3, 0.4, 0.5, 0.6]
        self._load_templates()

        # amcl drift detection
        self.last_good_pose = None
        self.last_pose_time = None
        self.pose_jump_threshold = 1.0
        self.amcl_cov_threshold = 0.5
        self.amcl_covariance = None
        self.initial_pose_pub = rospy.Publisher(
            '/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                         self._amcl_pose_cb, queue_size=1)

        # navigation
        self.move_client = None
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.latest_scan = None          # raw LaserScan for escape heading
        self.recent_stall_xy = []        # (x,y,time) for repeat-stall detection

        # frame calibration
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.spawn_min_x = self.spawn_max_x = 0.0
        self.spawn_min_y = self.spawn_max_y = 0.0

        # handover to C++: write rarest digit to a file so the next task can
        # consume it without a ROS client. Latest counts are stashed in
        # self.final_counts at the end of run() and re-read on shutdown.
        self.output_file = rospy.get_param(
            '~output_file', '/tmp/rarest_box.txt')
        self.final_counts = None
        rospy.on_shutdown(self.write_result_file)

        # publishers
        self.count_pub = rospy.Publisher('/box_counter/counts', String, queue_size=10, latch=True)
        self.marker_pub = rospy.Publisher('/box_counter/box_markers', MarkerArray, queue_size=10, latch=True)
        self.debug_img_pub = rospy.Publisher('/box_counter/debug_image', Image, queue_size=1)

        # subscribers
        rospy.Subscriber('/front/scan', LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber('/front/image_raw', Image, self.image_callback,
                         queue_size=1, buff_size=2**24)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        rospy.loginfo("Box Counter Node started")

    # -- templates --

    def _load_templates(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        tdir = os.path.normpath(os.path.join(script_dir, '..', '..', 'me5413_group9', 'boxes', 'cropped'))
        for d in range(1, 10):
            img = cv2.imread(os.path.join(tdir, 'img{}.png'.format(d)), cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            self.templates[d] = [cv2.resize(img, (int(img.shape[1]*s), int(img.shape[0]*s)))
                                 for s in self.template_scales
                                 if int(img.shape[1]*s) >= 15 and int(img.shape[0]*s) >= 15]
        rospy.loginfo("Loaded %d digit templates from %s", len(self.templates), tdir)

    # -- map --

    def map_callback(self, msg):
        if self.map_loaded:
            return
        raw = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        occupied = ((raw > 50) | (raw < 0)).astype(np.uint8)
        buf_px = max(1, int(self.wall_buffer_m / msg.info.resolution))
        kernel = np.ones((2*buf_px+1, 2*buf_px+1), np.uint8)
        self.wall_mask = cv2.dilate(occupied, kernel, iterations=1)
        self.map_info = msg.info
        self.map_loaded = True
        rospy.loginfo("Map loaded: %dx%d res=%.3f", msg.info.width, msg.info.height, msg.info.resolution)

    def is_known_wall(self, mx, my):
        if not self.map_loaded:
            return False
        info = self.map_info
        px = int((mx - info.origin.position.x) / info.resolution)
        py = int((my - info.origin.position.y) / info.resolution)
        if px < 0 or px >= info.width or py < 0 or py >= info.height:
            return True
        return self.wall_mask[py, px] > 0

    # -- pose & amcl --

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(0.5))
            x, y = t.transform.translation.x, t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return x, y, yaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def _amcl_pose_cb(self, msg):
        cov = msg.pose.covariance
        self.amcl_covariance = (cov[0], cov[7], cov[35])

    def _reset_pose(self, x, y, yaw, reason):
        rospy.logwarn("AMCL RESET (%s): (%.1f, %.1f, %.0f deg)", reason, x, y, math.degrees(yaw))
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = q[0], q[1]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q[2], q[3]
        msg.pose.covariance[0] = 0.1
        msg.pose.covariance[7] = 0.1
        msg.pose.covariance[35] = 0.05
        self.initial_pose_pub.publish(msg)
        try:
            from std_srvs.srv import Empty
            rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
        except Exception:
            pass
        rospy.sleep(0.5)

    def check_amcl_drift(self):
        pose = self.get_robot_pose()
        if pose is None:
            if self.last_good_pose is not None:
                self._reset_pose(*self.last_good_pose, reason="TF failed")
            return
        now = rospy.Time.now()
        x, y, yaw = pose

        if self.amcl_covariance is not None:
            cxx, cyy, cyaw = self.amcl_covariance
            if math.isnan(cxx) or math.isnan(cyy) or math.isnan(cyaw) or \
                    cxx > self.amcl_cov_threshold or cyy > self.amcl_cov_threshold:
                if self.last_good_pose is not None:
                    self._reset_pose(*self.last_good_pose, reason="cov blowup")
                    self.amcl_covariance = None
                return

        if self.last_good_pose is not None and self.last_pose_time is not None:
            dt = (now - self.last_pose_time).to_sec()
            if dt < 0.1:
                return
            jump = math.hypot(x - self.last_good_pose[0], y - self.last_good_pose[1])
            if jump > max(2.0 * dt, self.pose_jump_threshold):
                self._reset_pose(*self.last_good_pose, reason="jump %.2fm" % jump)
                return

        self.last_good_pose = (x, y, yaw)
        self.last_pose_time = now

    # -- lidar scan --

    def scan_callback(self, msg):
        self.latest_scan = msg
        if not self.collecting_points or not self.map_loaded:
            return
        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        for i, r in enumerate(msg.ranges):
            if r < 0.3 or r > 6.0 or r < msg.range_min or r > msg.range_max or math.isinf(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment + ryaw
            px = rx + r * math.cos(angle)
            py = ry + r * math.sin(angle)

            if not (self.spawn_min_x <= px <= self.spawn_max_x and
                    self.spawn_min_y <= py <= self.spawn_max_y):
                continue
            # Base/dispenser zone: the U-shaped base structure (pillars + walls)
            # returns heavy LiDAR that spawned phantom "boxes" (observed at
            # (5.7,-0.5) and (5.0,1.4)) blocking sub-goal 1 in the local costmap.
            # Skip the full base rectangle — real task boxes spawn past x=6.5.
            if 4.5 <= px <= 6.5 and -1.0 <= py <= 2.0:
                continue
            if self.is_known_wall(px, py):
                continue

            self.new_object_points.append([px, py])

            best_idx, best_dist = -1, self.live_merge_dist
            for j, lb in enumerate(self.live_boxes):
                d = math.hypot(px - lb[0], py - lb[1])
                if d < best_dist:
                    best_dist, best_idx = d, j

            if best_idx >= 0:
                lb = self.live_boxes[best_idx]
                n = lb[2]
                lb[0] = (lb[0]*n + px) / (n+1)
                lb[1] = (lb[1]*n + py) / (n+1)
                lb[2] = n + 1
                rospy.loginfo_throttle(2.0,
                    "LiDAR: merged pt (%.1f,%.1f) -> box #%d (%.1f,%.1f) hits=%d (dist=%.2fm < %.1fm)",
                    px, py, best_idx, lb[0], lb[1], lb[2], best_dist, self.live_merge_dist)
            else:
                self.live_boxes.append([px, py, 1])
                rospy.loginfo("LiDAR: new object #%d at (%.1f,%.1f) [total live=%d]",
                              len(self.live_boxes)-1, px, py, len(self.live_boxes))

    # -- clustering --

    def cluster_boxes(self):
        n_raw = len(self.new_object_points)
        if n_raw < self.cluster_min_pts:
            return []
        pts = self._grid_downsample(np.array(self.new_object_points), self.grid_cell)
        rospy.loginfo("Clustering: %d raw -> %d cells", n_raw, len(pts))
        clusters = self._dbscan(pts, self.cluster_eps, self.cluster_min_pts)

        candidates = []
        for cl in clusters:
            cx, cy = np.mean(cl[:,0]), np.mean(cl[:,1])
            ext_x, ext_y = np.ptp(cl[:,0]), np.ptp(cl[:,1])
            mx_ext, mn_ext = max(ext_x, ext_y), min(ext_x, ext_y)
            if not (self.box_extent_min <= mx_ext <= self.box_extent_max):
                continue
            # Shape filter removed: a single-face box view is a thin line
            # (ratio ~0.1), which the old filter rejected. Extent bounds above
            # already constrain size.
            candidates.append((cx, cy, len(cl)))

        merged = self._merge_nearby(candidates, 1.0)
        rospy.loginfo("Clustered %d -> merged %d boxes", len(candidates), len(merged))
        return merged

    @staticmethod
    def _grid_downsample(points, cell):
        seen = {}
        for p in points:
            key = (int(p[0]/cell), int(p[1]/cell))
            if key not in seen:
                seen[key] = p
        return np.array(list(seen.values()))

    @staticmethod
    def _dbscan(points, eps, min_pts):
        n = len(points)
        visited = np.zeros(n, dtype=bool)
        clusters = []
        for i in range(n):
            if visited[i]:
                continue
            visited[i] = True
            idxs, queue = [i], [i]
            while queue:
                idx = queue.pop(0)
                dists = np.linalg.norm(points - points[idx], axis=1)
                nbs = np.where((dists < eps) & (~visited))[0]
                for nb in nbs:
                    visited[nb] = True
                    idxs.append(nb)
                    queue.append(nb)
            if len(idxs) >= min_pts:
                clusters.append(points[idxs])
        return clusters

    @staticmethod
    def _merge_nearby(candidates, min_dist):
        if not candidates:
            return []
        used = [False] * len(candidates)
        merged = []
        for i in range(len(candidates)):
            if used[i]:
                continue
            used[i] = True
            cx, cy, cnt = candidates[i]
            sx, sy, sn = cx*cnt, cy*cnt, cnt
            absorbed = []
            for j in range(i+1, len(candidates)):
                if used[j]:
                    continue
                d = math.hypot(cx - candidates[j][0], cy - candidates[j][1])
                if d < min_dist:
                    used[j] = True
                    sx += candidates[j][0]*candidates[j][2]
                    sy += candidates[j][1]*candidates[j][2]
                    sn += candidates[j][2]
                    absorbed.append((j, candidates[j][0], candidates[j][1], d))
            if absorbed:
                for (j, ax, ay, d) in absorbed:
                    rospy.loginfo("  merge: cand #%d (%.2f,%.2f) <- #%d (%.2f,%.2f) dist=%.2fm",
                                  i, cx, cy, j, ax, ay, d)
            merged.append([sx/sn, sy/sn])
        return merged

    # -- camera detection --

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        if not self.ocr_active:
            return

        now = rospy.Time.now()
        if self.last_ocr_time and (now - self.last_ocr_time).to_sec() < 0.2:
            return
        self.last_ocr_time = now

        # skip if amcl is lost
        if self.amcl_covariance is not None:
            cxx, cyy, cyaw = self.amcl_covariance
            if math.isnan(cxx) or math.isnan(cyy) or math.isnan(cyaw) or \
                    cxx > self.amcl_cov_threshold or cyy > self.amcl_cov_threshold:
                return

        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose[0], pose[1], pose[2]
        # Project sighting forward along robot heading: OCR fires when the
        # digit is centered in the forward-facing camera, so the box is ~2.5m
        # ahead of the robot, not at the robot. This makes camera_detections
        # cluster near actual box positions for vote matching.
        # Snap camera sighting to nearest LiDAR live_box in the forward
        # hemisphere (within 5m, |rel_angle| <= pi/2). Straight forward-
        # projection was unreliable when the box was to the side — going
        # up vs down the same corridor produced sighting positions 5m apart
        # for the same physical box, breaking dedup. LiDAR live_boxes give
        # us the actual object location.
        CAM_MAX_RANGE = 5.0
        best_b = None
        best_d = CAM_MAX_RANGE
        for lb in self.live_boxes:
            lbx, lby = lb[0], lb[1]
            dx, dy = lbx - rx, lby - ry
            d = math.hypot(dx, dy)
            if d < 0.4 or d > CAM_MAX_RANGE:
                continue
            # relative angle of the object w.r.t. robot heading
            rel = math.atan2(dy, dx) - ryaw
            rel = math.atan2(math.sin(rel), math.cos(rel))  # wrap to [-pi,pi]
            if abs(rel) > math.pi / 2:
                continue  # behind robot
            if d < best_d:
                best_d, best_b = d, (lbx, lby)
        if best_b is None:
            return  # OCR fired but no LiDAR object forward — treat as noise
        bx_est, by_est = best_b
        # Reject snapped position outside spawn area (safety).
        if not (self.spawn_min_x <= bx_est <= self.spawn_max_x and
                self.spawn_min_y <= by_est <= self.spawn_max_y):
            return

        frame = self.latest_frame.copy()
        for (x, y, w, h) in self._find_box_regions(frame):
            px, py = int(w*0.1), int(h*0.1)
            roi = frame[y+py:y+h-py, x+px:x+w-px]
            if roi.size == 0:
                continue
            digit = self._match_digit(roi)
            if digit is None:
                continue

            # spatial confirmation (uses projected box position, not robot pos)
            matched_key = None
            for k in self.ocr_confirm:
                if k[0] == digit and math.hypot(bx_est-k[1], by_est-k[2]) < self.ocr_confirm_radius:
                    matched_key = k
                    break
            if matched_key:
                self.ocr_confirm[matched_key] += 1
            else:
                matched_key = (digit, bx_est, by_est)
                self.ocr_confirm[matched_key] = 1
            if self.ocr_confirm[matched_key] < self.ocr_confirm_threshold:
                continue

            # dedup (projected pos)
            is_dup = any(dd == digit and math.hypot(bx_est-dx, by_est-dy) < self.ocr_dedup_dist
                         for dd, dx, dy in self.camera_detections)
            if not is_dup:
                self.camera_detections.append((digit, bx_est, by_est))
                rospy.loginfo("CAMERA: digit %d at ~(%.1f,%.1f) [robot (%.1f,%.1f)] total=%d",
                              digit, bx_est, by_est, rx, ry, len(self.camera_detections))

    @staticmethod
    def _find_box_regions(frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blurred, 30, 100)
        edges = cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_RECT, (5,5)), iterations=2)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        regions = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            area = w * h
            if area < 2000 or area > 80000:
                continue
            if not (0.4 < float(w)/h < 2.5):
                continue
            if gray[y:y+h, x:x+w].std() < 20:
                continue
            regions.append((x, y, w, h))
        return regions

    def _match_digit(self, roi):
        """Hybrid: tesseract for digit ID, template matching as cross-check."""
        if roi.size == 0:
            return None
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if len(roi.shape) == 3 else roi

        # tesseract
        _, binary = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
        padded = cv2.copyMakeBorder(binary, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=255)
        try:
            result = pytesseract.image_to_string(
                padded, config='--psm 10 -c tessedit_char_whitelist=123456789').strip()
        except Exception:
            return None
        if not result or len(result) != 1 or not result.isdigit():
            return None
        tess_digit = int(result)
        if tess_digit < 1 or tess_digit > 9:
            return None

        # template cross-check
        if self.templates:
            rh, rw = gray.shape[:2]
            scores = {}
            for digit, tmpls in self.templates.items():
                best = -1.0
                for tmpl in tmpls:
                    th, tw = tmpl.shape[:2]
                    if tw >= rw or th >= rh:
                        continue
                    _, mv, _, _ = cv2.minMaxLoc(cv2.matchTemplate(gray, tmpl, cv2.TM_CCOEFF_NORMED))
                    if mv > best:
                        best = mv
                scores[digit] = best

            tmpl_best = max(scores, key=scores.get)
            if tmpl_best == tess_digit:
                return tess_digit
            # Disagreement. Tesseract confuses 2<->7 etc. on block digits;
            # templates are usually more reliable here. Prefer template when
            # its top is clearly above second-best; reject when both unsure.
            ranked = sorted(scores.values(), reverse=True)
            if len(ranked) > 1 and ranked[0] - ranked[1] >= self.match_margin:
                # template is confident and contradicts Tesseract -> trust template
                return tmpl_best
            # template is not confident -> reject as ambiguous
            return None

        return tess_digit

    # -- navigation: move_base client + goal helpers --

    def _ensure_move_client(self):
        if self.move_client is None:
            self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            rospy.loginfo("Waiting for move_base...")
            self.move_client.wait_for_server(rospy.Duration(10.0))

    def _cluster_camera_detections(self, radius=1.5):
        """Group camera_detections into box-sized clusters by position.
        Each cluster's digit is the majority vote. Returns list of
        (cx, cy, digit, votes_dict)."""
        if not self.camera_detections:
            return []
        clusters = []  # each: [sum_x, sum_y, n, votes_dict]
        for digit, dx, dy in self.camera_detections:
            best = None
            best_d = radius
            for k, cl in enumerate(clusters):
                cx, cy = cl[0]/cl[2], cl[1]/cl[2]
                d = math.hypot(dx-cx, dy-cy)
                if d < best_d:
                    best_d, best = d, k
            if best is None:
                clusters.append([dx, dy, 1, {digit: 1}])
            else:
                cl = clusters[best]
                cl[0] += dx; cl[1] += dy; cl[2] += 1
                cl[3][digit] = cl[3].get(digit, 0) + 1
        out = []
        for cl in clusters:
            cx, cy = cl[0]/cl[2], cl[1]/cl[2]
            votes = cl[3]
            dig = max(votes, key=votes.get)
            out.append((cx, cy, dig, votes))
        return out

    def _viewpoint_ok(self, vx, vy, target_xy=None):
        """Viewpoint is clear if inside spawn area, not on a wall, and not
        inside any NON-target box's 0.8m footprint. The box we're trying to
        view is allowed to be within the offset distance (by design)."""
        if not (self.spawn_min_x <= vx <= self.spawn_max_x and
                self.spawn_min_y <= vy <= self.spawn_max_y):
            return False
        if self.is_known_wall(vx, vy):
            return False
        for bx, by in self.box_positions:
            if target_xy is not None and bx == target_xy[0] and by == target_xy[1]:
                continue
            if math.hypot(vx-bx, vy-by) < 0.8:
                return False
        return True

    def _compute_viewpoint(self, box_xy, prev_xy, offset=2.0):
        """Drive-by viewpoint `offset` m short of box_xy along line prev->box.
        Yaw faces the box. Returns (vx, vy, yaw) or None if no safe spot."""
        bx, by = box_xy
        px, py = prev_xy
        dx, dy = bx - px, by - py
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            # prev and box coincide; pick arbitrary approach from +x
            ux, uy = 1.0, 0.0
        else:
            ux, uy = dx / dist, dy / dist
        # nominal: on the line, `offset` m short of box
        vx, vy = bx - offset * ux, by - offset * uy
        yaw = math.atan2(by - vy, bx - vx)
        if self._viewpoint_ok(vx, vy, target_xy=box_xy):
            return (vx, vy, yaw)
        # perpendicular offset fallback (both sides)
        for sign in (+1, -1):
            vx2 = vx + sign * 0.8 * (-uy)
            vy2 = vy + sign * 0.8 * ( ux)
            if self._viewpoint_ok(vx2, vy2, target_xy=box_xy):
                yaw2 = math.atan2(by - vy2, bx - vx2)
                return (vx2, vy2, yaw2)
        return None

    def _make_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        return goal

    @staticmethod
    def _interpolate_waypoints(waypoints, step=3.0):
        """Produce dense sub-goals. Every sub-goal's yaw points in the
        direction of travel (prev -> this), INCLUDING the waypoint endpoints,
        so TEB never has to pivot at a corner. Only the very final waypoint
        may keep its authored yaw (for post-sweep pose). This eliminates the
        90-degree stops that were causing AMCL drift."""
        result = []
        last_idx = len(waypoints) - 1
        for i in range(len(waypoints)):
            mx, my, myaw = waypoints[i]
            if i == 0:
                # first waypoint: yaw in direction of next waypoint if possible
                if last_idx >= 1:
                    nx, ny, _ = waypoints[1]
                    iyaw = math.atan2(ny - my, nx - mx)
                else:
                    iyaw = myaw
                result.append((mx, my, iyaw))
                continue
            px, py, _ = waypoints[i-1]
            dx, dy = mx - px, my - py
            dist = math.hypot(dx, dy)
            travel_yaw = math.atan2(dy, dx)
            endpoint_yaw = myaw if i == last_idx else travel_yaw
            if dist <= step:
                result.append((mx, my, endpoint_yaw))
            else:
                n = int(math.ceil(dist / step))
                for s in range(1, n+1):
                    t = s / n
                    iyaw = travel_yaw if s < n else endpoint_yaw
                    result.append((px + dx*t, py + dy*t, iyaw))
        return result

    # -- navigation: recovery helpers --

    def _clear_costmaps(self):
        try:
            from std_srvs.srv import Empty
            rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
        except Exception:
            pass

    def _drive(self, lin, ang, duration):
        """Open-loop drive for `duration` seconds."""
        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        r = rospy.Rate(20)
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            r.sleep()
        self.cmd_vel_pub.publish(Twist())

    def _is_recent_stall_nearby(self, x, y, radius=1.0, window_s=15.0):
        now = rospy.Time.now()
        self.recent_stall_xy = [s for s in self.recent_stall_xy
                                if (now - s[2]).to_sec() < window_s]
        return any(math.hypot(x-sx, y-sy) < radius for sx, sy, _ in self.recent_stall_xy)

    def _escape_stage1(self):
        """Fastest escape: cancel, clear, brief wait for replan."""
        self.move_client.cancel_goal()
        self._clear_costmaps()
        rospy.sleep(0.5)

    def _escape_stage2(self):
        """Longer back-up + double clear. No rotation — AMCL drifts in tight areas."""
        self.move_client.cancel_goal()
        self._clear_costmaps()
        self._drive(-0.3, 0.0, 3.0)   # back up ~0.9 m straight
        self._clear_costmaps()
        if self.last_good_pose is not None:
            self._reset_pose(*self.last_good_pose, reason="post-stuck-stage2")
        rospy.sleep(0.3)

    def _segment_clear(self, x0, y0, x1, y1, clearance=0.6):
        """Straight-line clearance check using static wall mask + live box centroids."""
        if not self.map_loaded:
            return True  # can't check, be optimistic
        dist = math.hypot(x1-x0, y1-y0)
        if dist < 1e-3:
            return True
        step = max(self.map_info.resolution, 0.1)
        n = int(dist / step) + 1
        for k in range(1, n+1):
            t = k / float(n)
            sx = x0 + (x1-x0)*t
            sy = y0 + (y1-y0)*t
            if self.is_known_wall(sx, sy):
                return False
            for lb in self.live_boxes:
                if lb[2] < 3:  # ignore weakly-supported blips
                    continue
                if math.hypot(sx-lb[0], sy-lb[1]) < clearance:
                    return False
        return True

    def _next_clear_subgoal(self, sub_goals, i, lookahead=3):
        """Index of next sub-goal with clear LOS from current pose; fallback i+1."""
        pose = self.get_robot_pose()
        if pose is None:
            return i + 1
        rx, ry, _ = pose
        upper = min(len(sub_goals), i + 1 + lookahead)
        for j in range(i + 1, upper):
            gx, gy, _ = sub_goals[j]
            if self._segment_clear(rx, ry, gx, gy):
                return j
        return i + 1

    # -- navigation: sweep + inline inspection --

    def sweep_waypoints(self, waypoints, proximity=1.5, timeout_per_wp=8.0):
        self._ensure_move_client()
        rate = rospy.Rate(5)
        sub_goals = self._interpolate_waypoints(waypoints, step=3.0)
        rospy.loginfo("Sweep: %d waypoints -> %d sub-goals", len(waypoints), len(sub_goals))

        # Inline inspection state. Reset each sweep call.
        self._insp_visited = []            # list of (x,y) we already inspected
        self._insp_count = 0               # how many inspections done so far
        self._insp_cap = 6                 # hard cap
        self._insp_budget = 150.0          # wall-clock seconds total sweep budget
        self._insp_start = rospy.Time.now()

        i = 0
        while i < len(sub_goals):
            if rospy.is_shutdown():
                return
            mx, my, myaw = sub_goals[i]
            if i % 3 == 0 or i == len(sub_goals) - 1:
                rospy.loginfo("Sub-goal %d/%d (%.1f,%.1f) pts=%d cam=%d",
                              i+1, len(sub_goals), mx, my,
                              len(self.new_object_points), len(self.camera_detections))

            stall_count = 0
            skip_ahead = False

            self.move_client.send_goal(self._make_goal(mx, my, myaw))
            start = rospy.Time.now()
            last_pose = self.get_robot_pose()
            last_move_time = start

            while not rospy.is_shutdown():
                elapsed = (rospy.Time.now() - start).to_sec()
                self.check_amcl_drift()

                cur = self.get_robot_pose()
                if cur is not None:
                    if math.hypot(cur[0]-mx, cur[1]-my) < proximity:
                        break
                    if last_pose is not None:
                        if math.hypot(cur[0]-last_pose[0], cur[1]-last_pose[1]) > 0.15:
                            last_pose, last_move_time = cur, rospy.Time.now()

                    if (rospy.Time.now() - last_move_time).to_sec() > 1.8:
                        stall_count += 1
                        nearby = self._is_recent_stall_nearby(cur[0], cur[1])
                        self.recent_stall_xy.append((cur[0], cur[1], rospy.Time.now()))

                        if stall_count == 1 and not nearby:
                            rospy.logwarn("Sub-goal %d stall#1 -> clear+replan", i+1)
                            self._escape_stage1()
                            self.move_client.send_goal(self._make_goal(mx, my, myaw))
                            last_pose, last_move_time = cur, rospy.Time.now()
                        elif stall_count == 2 or nearby:
                            rospy.logwarn("Sub-goal %d stall#%d -> backup+rotate",
                                          i+1, stall_count)
                            self._escape_stage2()
                            self.move_client.send_goal(self._make_goal(mx, my, myaw))
                            last_pose, last_move_time = cur, rospy.Time.now()
                        else:
                            rospy.logwarn("Sub-goal %d stall#%d -> skip-ahead",
                                          i+1, stall_count)
                            self.move_client.cancel_goal()
                            skip_ahead = True
                            break

                if elapsed > timeout_per_wp:
                    rospy.logwarn("Sub-goal %d TIMEOUT, skip-ahead", i+1)
                    self.move_client.cancel_goal()
                    skip_ahead = True
                    break

                state = self.move_client.get_state()
                if state in [3, 4, 5, 9]:
                    if state == 4:
                        self._escape_stage1()
                    break
                rate.sleep()

            # Inline inspection: between sub-goals, check for nearby LiDAR
            # boxes that have no camera digit yet. Visit up to `_insp_cap` and
            # stop if wall-clock budget exhausted.
            cur_pose = self.get_robot_pose()
            if cur_pose is not None:
                self._inline_inspect(cur_pose)

            if skip_ahead:
                i = self._next_clear_subgoal(sub_goals, i)
            else:
                i += 1

        self.move_client.cancel_goal()

    def _inline_inspect(self, cur_pose, nearby_radius=4.0, dwell=0.6,
                        timeout_per_wp=8.0, already_dist=1.5):
        """Between sub-goals, opportunistically inspect LiDAR boxes near the
        robot that have no camera digit yet. Budget- and cap-bounded."""
        if self._insp_count >= self._insp_cap:
            return
        elapsed = (rospy.Time.now() - self._insp_start).to_sec()
        if elapsed > self._insp_budget:
            return
        if len(self.new_object_points) < 50:
            return  # not enough LiDAR yet

        lidar_boxes = self.cluster_boxes()
        if not lidar_boxes:
            return
        camera_boxes = self._cluster_camera_detections(radius=1.5)

        cx, cy = cur_pose[0], cur_pose[1]
        # Candidates: LiDAR boxes NOT covered by any camera cluster,
        # NOT already inspected, within nearby_radius of robot.
        candidates = []
        for (bx, by) in lidar_boxes:
            if any(math.hypot(bx-vx, by-vy) < already_dist for (vx, vy) in self._insp_visited):
                continue
            if any(math.hypot(bx-ccx, by-ccy) < 2.0 for (ccx, ccy, _, _) in camera_boxes):
                continue
            d = math.hypot(bx-cx, by-cy)
            if d > nearby_radius:
                continue
            candidates.append((d, (bx, by)))
        if not candidates:
            return
        candidates.sort(key=lambda t: t[0])

        prev = (cx, cy)
        remaining_cap = self._insp_cap - self._insp_count
        for _, b in candidates[:remaining_cap]:
            if (rospy.Time.now() - self._insp_start).to_sec() > self._insp_budget:
                rospy.logwarn("  inline-inspect: budget exhausted")
                break
            vp = self._compute_viewpoint(b, prev, offset=2.0)
            if vp is None:
                rospy.logwarn("  inline-inspect: (%.1f,%.1f) no safe VP", b[0], b[1])
                self._insp_visited.append(b)  # don't retry
                continue
            vx, vy, vyaw = vp
            rospy.loginfo("  inline-inspect #%d: box (%.1f,%.1f) -> VP (%.1f,%.1f) yaw=%.0f",
                          self._insp_count+1, b[0], b[1], vx, vy, math.degrees(vyaw))
            self.move_client.send_goal(self._make_goal(vx, vy, vyaw))
            self.move_client.wait_for_result(rospy.Duration(timeout_per_wp))
            self.move_client.cancel_goal()
            rospy.sleep(dwell)
            self._insp_visited.append(b)
            self._insp_count += 1
            prev = b

    # -- rviz markers --

    def publish_markers(self):
        ma = MarkerArray()
        for i, (bx, by) in enumerate(self.box_positions):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = rospy.Time.now()
            m.ns, m.id, m.type, m.action = 'boxes', i, Marker.CUBE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = bx, by, 0.4
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = BOX_SIZE
            c = (0.0, 1.0, 0.0) if self.box_digits[i] is not None else (1.0, 1.0, 0.0)
            m.color.r, m.color.g, m.color.b, m.color.a = c[0], c[1], c[2], 0.6
            ma.markers.append(m)

            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = rospy.Time.now()
            t.ns, t.id, t.type, t.action = 'labels', i, Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = bx, by, 1.2
            t.scale.z = 0.5
            t.text = str(self.box_digits[i]) if self.box_digits[i] is not None else '?'
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            ma.markers.append(t)
        self.marker_pub.publish(ma)

    # -- frame calibration --

    def world_to_map(self, wx, wy):
        return (wx + self.offset_x, wy + self.offset_y)

    def calibrate_frames(self):
        rx, ry, _ = self.get_robot_pose()
        try:
            from gazebo_msgs.srv import GetModelState
            rospy.wait_for_service('/gazebo/get_model_state', timeout=5)
            state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)('jackal', 'world')
            self.offset_x = rx - state.pose.position.x
            self.offset_y = ry - state.pose.position.y
        except Exception:
            self.offset_x, self.offset_y = rx, ry
        rospy.loginfo("World->map offset: (%.2f, %.2f)", self.offset_x, self.offset_y)

    # -- main --

    def run(self):
        rate = rospy.Rate(1)
        rospy.loginfo("Waiting for TF...")
        while not rospy.is_shutdown():
            if self.get_robot_pose() is not None:
                break
            rate.sleep()
        rospy.sleep(1.0)
        self.calibrate_frames()

        rospy.loginfo("Waiting for map...")
        while not rospy.is_shutdown() and not self.map_loaded:
            rate.sleep()

        self.spawn_min_x = WORLD_SPAWN_X[0] + self.offset_x
        self.spawn_max_x = WORLD_SPAWN_X[1] + self.offset_x
        self.spawn_min_y = WORLD_SPAWN_Y[0] + self.offset_y
        self.spawn_max_y = WORLD_SPAWN_Y[1] + self.offset_y
        rospy.loginfo("Spawn area (map): X[%.1f,%.1f] Y[%.1f,%.1f]",
                      self.spawn_min_x, self.spawn_max_x, self.spawn_min_y, self.spawn_max_y)

        sweep_pts = [(wx + self.offset_x, wy + self.offset_y, wyaw)
                     for wx, wy, wyaw in SWEEP_WAYPOINTS_WORLD]

        # sweep
        rospy.loginfo("=== Starting sweep ===")
        self.collecting_points = True
        self.ocr_active = True
        self.sweep_waypoints(sweep_pts, proximity=1.5, timeout_per_wp=10.0)
        self.collecting_points = False
        self.ocr_active = False

        rospy.loginfo("=" * 60)
        rospy.loginfo("SWEEP DONE. Raw LiDAR pts=%d | live_boxes=%d | camera sightings=%d",
                      len(self.new_object_points), len(self.live_boxes), len(self.camera_detections))
        rospy.loginfo("Live boxes (pre-DBSCAN, live_merge_dist=%.1fm):", self.live_merge_dist)
        for i, lb in enumerate(self.live_boxes):
            rospy.loginfo("  live #%d: (%.2f,%.2f) hits=%d", i, lb[0], lb[1], lb[2])

        # cluster lidar
        self.box_positions = self.cluster_boxes()
        self.box_digits = [None] * len(self.box_positions)
        rospy.loginfo("-" * 60)
        rospy.loginfo("FINAL MERGED BOX COUNT (LiDAR): %d", len(self.box_positions))
        for i, (bx, by) in enumerate(self.box_positions):
            rospy.loginfo("  box #%d: (%.2f,%.2f)", i, bx, by)
        rospy.loginfo("=" * 60)

        # === Camera-first counting ===
        # Camera sightings are already projected ~2.5m ahead of the robot, so
        # they cluster near real box centers. Treat each camera-cluster as an
        # authoritative box with its majority digit. LiDAR clusters that don't
        # overlap any camera_box are "unknown" and trigger a targeted Pass 2
        # inspection. Final count = camera_boxes + any inspected-unknowns that
        # acquire a digit during Pass 2.
        camera_boxes = self._cluster_camera_detections(radius=1.5)
        rospy.loginfo("-" * 60)
        rospy.loginfo("CAMERA BOXES (from sightings, merge=1.5m): %d", len(camera_boxes))
        for i, (cx, cy, dig, v) in enumerate(camera_boxes):
            rospy.loginfo("  cam #%d (%.2f,%.2f) digit=%d votes=%s", i, cx, cy, dig, v)

        # Find LiDAR-only unknowns: LiDAR clusters not covered by any camera_box.
        COVER_RADIUS = 2.0
        unknowns = []
        for (bx, by) in self.box_positions:
            covered = any(math.hypot(bx-cx, by-cy) < COVER_RADIUS
                          for (cx, cy, _, _) in camera_boxes)
            if not covered:
                unknowns.append((bx, by))
        rospy.loginfo("LiDAR-only unknowns (no nearby camera cluster): %d", len(unknowns))
        for u in unknowns:
            rospy.loginfo("  unknown (%.2f,%.2f)", u[0], u[1])

        # Post-sweep inspection intentionally removed: unknowns should have
        # been picked up inline during sweep_waypoints via _inline_inspect().
        # If any remain here, they're either LiDAR false positives (walls that
        # passed clustering) or boxes the robot never got close enough to,
        # which we can't fix post-hoc without another timed pass.
        if unknowns:
            rospy.logwarn("Leftover unknowns (LiDAR w/o camera): %d — "
                          "accepted as uncounted to respect 3-min budget",
                          len(unknowns))

        # Final counts come from camera_boxes (authoritative).
        self.publish_markers()
        cam_counts = {}
        for (_, _, dig, _) in camera_boxes:
            cam_counts[dig] = cam_counts.get(dig, 0) + 1

        self.final_counts = dict(cam_counts)
        if cam_counts:
            sorted_c = sorted(cam_counts.items())
            total = sum(cam_counts.values())
            # Deterministic tie-break: lowest count, then lowest digit.
            rarest = min(cam_counts.items(), key=lambda kv: (kv[1], kv[0]))[0]
            rospy.loginfo("BOX COUNTS (camera):")
            for num, cnt in sorted_c:
                rospy.loginfo("  Number %d: %d boxes", num, cnt)
            rospy.loginfo("  TOTAL: %d boxes", total)
            rospy.loginfo("  RAREST: number %d (count=%d)", rarest, cam_counts[rarest])
            msg = "Total: {} | {} | Rarest: {}".format(
                total, ", ".join("Box {}: {}".format(n, c) for n, c in sorted_c), rarest)
            self.count_pub.publish(String(data=msg))
        else:
            rospy.logwarn("NO DIGITS DETECTED")

        rospy.loginfo("Done. Results on /box_counter/counts")

        # Guarantee arrival at the final waypoint so the next task can begin.
        # Retries with clear+backup between attempts. Blocks until within 1m
        # of target or attempt cap reached.
        # Return to the FIRST sweep waypoint (bottom-left interior corner)
        # instead of the doorway/exit. The exit waypoint sits in a narrow
        # passage near the cone where AMCL loses features and drifts; the
        # entry corner has two walls in view, so the relocalization holds
        # and the next task starts from a known-good pose.
        end_wx, end_wy, end_wyaw = SWEEP_WAYPOINTS_WORLD[0]
        end_x = end_wx + self.offset_x
        end_y = end_wy + self.offset_y
        self._return_to_waypoint(end_x, end_y, end_wyaw, proximity=1.0, max_attempts=5)

        # Sweep complete. Trigger shutdown so the on_shutdown hook writes the
        # result file and the process exits cleanly for the C++ caller.
        rospy.loginfo("Sweep complete; shutting down for handoff.")
        rospy.signal_shutdown("box counting complete")

    def write_result_file(self):
        """Write rarest digit to ~output_file for C++ handover."""
        counts = self.final_counts or {}
        if counts:
            rarest = min(counts.items(), key=lambda kv: (kv[1], kv[0]))[0]
            payload = "{}\n".format(rarest)
        else:
            rospy.logwarn("No digits detected; writing 0 to %s",
                          self.output_file)
            payload = "0\n"
        try:
            with open(self.output_file, 'w') as f:
                f.write(payload)
            rospy.loginfo("Wrote rarest=%s to %s",
                          payload.strip(), self.output_file)
        except OSError as e:
            rospy.logerr("Failed to write %s: %s", self.output_file, e)

    def _return_to_waypoint(self, x, y, yaw, proximity=1.0, max_attempts=5,
                            per_attempt_timeout=25.0):
        """Drive the robot to (x,y,yaw) with retries. Intended as a hard
        guarantee for task handoff — the next task assumes the robot is here."""
        rospy.loginfo("=" * 60)
        rospy.loginfo("RETURN TO END: target (%.1f,%.1f)", x, y)
        self._ensure_move_client()
        for attempt in range(1, max_attempts + 1):
            cur = self.get_robot_pose()
            if cur is not None and math.hypot(cur[0]-x, cur[1]-y) < proximity:
                rospy.loginfo("  already within %.1fm of end, done", proximity)
                return True
            rospy.loginfo("  attempt %d/%d", attempt, max_attempts)
            self.move_client.send_goal(self._make_goal(x, y, yaw))
            start = rospy.Time.now()
            last_pose = cur
            last_move = start
            reached = False
            while not rospy.is_shutdown():
                if (rospy.Time.now() - start).to_sec() > per_attempt_timeout:
                    rospy.logwarn("  attempt %d timeout", attempt)
                    self.move_client.cancel_goal()
                    break
                cur = self.get_robot_pose()
                if cur is not None:
                    if math.hypot(cur[0]-x, cur[1]-y) < proximity:
                        reached = True
                        break
                    if last_pose is not None:
                        if math.hypot(cur[0]-last_pose[0], cur[1]-last_pose[1]) > 0.15:
                            last_pose, last_move = cur, rospy.Time.now()
                    if (rospy.Time.now() - last_move).to_sec() > 3.0:
                        rospy.logwarn("  stalled, clear+backup")
                        self._escape_stage1()
                        self._drive(-0.3, 0.0, 1.5)
                        self.move_client.send_goal(self._make_goal(x, y, yaw))
                        last_move = rospy.Time.now()
                state = self.move_client.get_state()
                if state in [3, 4, 5, 9]:
                    break
                rospy.sleep(0.1)
            if reached:
                rospy.loginfo("  REACHED end waypoint on attempt %d", attempt)
                return True
            self._escape_stage1()
        rospy.logwarn("RETURN TO END: failed after %d attempts, handing off anyway",
                      max_attempts)
        return False


if __name__ == '__main__':
    import sys
    try:
        BoxCounter().run()
    except rospy.ROSInterruptException:
        pass
    # rospy.on_shutdown writes the result file; explicit exit(0) so the
    # C++ caller gets a clean return code.
    sys.exit(0)