import cv2
import os
import glob

root = "/home/ferdi/ME5413_Final_Project/src/me5413_group9/boxes/cropped/"
image_paths = glob.glob(f"{root}*.png") \
            + glob.glob(f"{root}*.jpg")

detectors = {
    "AKAZE": cv2.AKAZE_create(),
    "ORB":   cv2.ORB_create(nfeatures=2000),
    "SIFT":  cv2.SIFT_create(),
}

avg_pts = {
    "AKAZE": 0,
    "ORB": 0,
    "SIFT": 0
}

for path in image_paths:
    # try:
    #     int(path[-5])
    # except ValueError:
    #     os.remove(path)
    #     continue

# for path in image_paths:
    print(path)
    img = cv2.imread(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    stem = os.path.splitext(path)[0]

    for name, detector in detectors.items():
        kpts = detector.detect(gray, None)
        out = cv2.drawKeypoints(img, kpts, None,
                                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # cv2.putText(out, f"{name}: {len(kps)} keypoints",
        #             (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # cv2.imwrite(f"{stem}_{name}_{len(kps)}kpts.png", out)
        # print(f"Saved {stem}_{name}.png  ({len(kps)} keypoints)")
        avg_pts[name] += len(kpts) / 9

for name, num in avg_pts.items():
    print(f"{name}: {num}")