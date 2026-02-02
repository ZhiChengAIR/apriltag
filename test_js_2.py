import cv2
import numpy as np
from apriltag import apriltag

# --- Load image & detect tags ---
imagepath = 'data/snapshot_002.png'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
detector = apriltag("tagStandard41h12")

detections = detector.detect(image)
print(f"Detected {len(detections)} tags")

# --- Camera intrinsics (示例，替换成你的相机真实标定值) ---
# intrinsics: fx, fy, cx, cy; dist coeffs
fx = 605.68
fy = 605.21
cx = 323.22
cy = 253.70





camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]], dtype=np.float64)

dist_coeffs = np.zeros((5, 1), dtype=np.float64)  # 若没有畸变可设为 0

# --- Tag real size (单位: 米，例如 0.032 m = 32 mm) ---
tag_size_m = 0.032
half = tag_size_m / 2

# 世界坐标系中 tag 角点 (Z = 0)
object_points = np.array([
    [-half,  half, 0],  # left-top
    [ half,  half, 0],  # right-top
    [ half, -half, 0],  # right-bottom
    [-half, -half, 0],  # left-bottom
], dtype=np.float64)

for detection in detections:
    print(detection)

    # 取角点 (图像像素坐标)
    # apriltag 的 corners 顺序通常是 [lb, rb, rt, lt]
    corners = np.array([
        detection['lb-rb-rt-lt'][3],  # left-top
        detection['lb-rb-rt-lt'][2],  # right-top
        detection['lb-rb-rt-lt'][1],  # right-bottom
        detection['lb-rb-rt-lt'][0],  # left-bottom
    ], dtype=np.float64)
    # PnP solve
    success, rvec, tvec = cv2.solvePnP(
        object_points, corners,
        camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE
    )

    if not success:
        print("Pose estimation failed")
        continue

    # 旋转向量 -> 旋转矩阵
    R, _ = cv2.Rodrigues(rvec)

    # 欧拉角 (rad)
    roll  = np.arctan2(R[2,1], R[2,2])
    pitch = np.arctan2(-R[2,0], np.sqrt(R[0,0]**2 + R[1,0]**2))
    yaw   = np.arctan2(R[1,0], R[0,0])

    # 位移 (m -> mm)
    x_mm = tvec[0,0] * 1000
    y_mm = tvec[1,0] * 1000
    z_mm = tvec[2,0] * 1000

    # 输出 pose
    print(f"Tag ID: {detection['id']}")
    print(f"Translation (mm): X={x_mm:.1f}, Y={y_mm:.1f}, Z={z_mm:.1f}")
    print(f"Orientation (rad): roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f}")

    # 画角点 & 坐标轴
    img_color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    for pt in corners:
        cv2.circle(img_color, tuple(pt.astype(int)), 4, (0,255,0), -1)

    # 画坐标轴 (3 cm)
    axis = np.float32([[0.03,0,0], [0,0.03,0], [0,0,0.03]]).reshape(-1,3)
    proj_axis, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
    origin = tuple(corners[0].astype(int))
    cv2.line(img_color, origin, tuple(proj_axis[0].ravel().astype(int)), (0,0,255), 2)
    cv2.line(img_color, origin, tuple(proj_axis[1].ravel().astype(int)), (0,255,0), 2)
    cv2.line(img_color, origin, tuple(proj_axis[2].ravel().astype(int)), (255,0,0), 2)

cv2.imshow("Pose", img_color)
cv2.waitKey(0)
cv2.destroyAllWindows()
