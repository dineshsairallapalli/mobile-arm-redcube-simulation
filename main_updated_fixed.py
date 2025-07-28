
import time, math, numpy as np, cv2, pybullet as p, pybullet_data, random

# ───── 1.  SIMULATION SETUP ─────────────────────────────────────────
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# ───── 2.  LOAD ROBOT & MAP JOINTS/LINKS ───────────────────────────
robot_id = p.loadURDF("mycar.urdf",
                      basePosition=[random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5), 0.15], useFixedBase=False)

num_joints = p.getNumJoints(robot_id)
joint_map = {p.getJointInfo(robot_id, i)[1].decode(): i for i in range(num_joints)}
link_map  = {p.getJointInfo(robot_id, i)[12].decode(): i for i in range(num_joints)}
link_map["base_link"] = -1

W_LF, W_RF = joint_map["front_left_wheel_joint"], joint_map["front_right_wheel_joint"]
W_LR, W_RR = joint_map["rear_left_wheel_joint"],  joint_map["rear_right_wheel_joint"]
ARM_SHO, ARM_ELB = joint_map["joint1"], joint_map["joint2"]
LNK_CAM = link_map["camera_link"]
LNK_TIP = link_map["tapper_link"]

# ───── 3.  HELPERS ──────────────────────────────────────────────────
def drive(l: float, r: float, force: float = 120):
    for j in (W_LF, W_LR):
        p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL,
                                targetVelocity=l, force=force)
    for j in (W_RF, W_RR):
        p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL,
                                targetVelocity=r, force=force)

def closest(body_a, body_b, link_a=-1, max_d=10.0):
    pts = p.getClosestPoints(bodyA=body_a, bodyB=body_b,
                             distance=max_d, linkIndexA=link_a)
    return None if not pts else min(pt[8] for pt in pts)

# ───── 4.  CUBES ─────────────────────────────────────────────────────
CUBE_HALF = 0.05
cube_ids = []
colors = {
    'red': [1, 0, 0, 1],
    'green': [0, 1, 0, 1],
    'blue': [0, 0, 1, 1],
    'purple': [0.5, 0, 0.5, 1],
    'yellow': [1, 1, 0, 1],
}
np.random.seed(42)
for i, (name, rgba) in enumerate(colors.items()):
    pos = [0.3 + 0.2 * np.cos(i * 2 * np.pi / len(colors)),
           0.3 + 0.2 * np.sin(i * 2 * np.pi / len(colors)), CUBE_HALF]
    cid = p.createMultiBody(
        0.1,
        p.createCollisionShape(p.GEOM_BOX, halfExtents=[CUBE_HALF]*3),
        p.createVisualShape(p.GEOM_BOX, halfExtents=[CUBE_HALF]*3, rgbaColor=rgba),
        pos)
    if name == 'red':
        cube_ids.append(cid)

# ───── 5.  CAMERA & DETECTION ───────────────────────────────────────
H, W, FOV = 256, 256, 70
def cam_rgba():
    pos, orn = p.getLinkState(robot_id, LNK_CAM, True)[:2]
    R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3,3)
    view = p.computeViewMatrix(pos, pos + R@[1,0,0], R@[0,0,1])
    proj = p.computeProjectionMatrixFOV(FOV, W/H, 0.1, 5)
    _,_,rgba,_,_ = p.getCameraImage(W, H, view, proj,
                                    renderer=p.ER_TINY_RENDERER)
    return rgba

def blob_offset(rgba):
    rgb = np.asarray(rgba, np.uint8).reshape(H, W, 4)[..., :3]
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = (cv2.inRange(hsv,(0,50,50),(20,255,255)) |
            cv2.inRange(hsv,(160,50,50),(180,255,255)))
    if mask.sum() < 20*255: return None
    M = cv2.moments(mask)
    cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
    print(f"[BLOB DETECTED] offset: {(cx-W/2)/(W/2):.2f}, {(cy-H/2)/(H/2):.2f}")
    return (cx-W/2)/(W/2), (cy-H/2)/(H/2)

# ───── 6.  CONSTANTS ────────────────────────────────────────────────
SEARCH_SPIN, FWD_MAX = 4.0, 8.0
BRAKE_DIST, STOP_DIST = 0.40, 0.25
BASE_TURN_GAIN = 3.0
CREEP_SPEED = 1.5
HOME_SHO, HOME_ELB = -0.15, 1.2
ARM_GAIN_SHO = -1.5
L1, L2 = 0.30, 0.20
TIP_RADIUS = 0.02
AIM_MARGIN = 0.002
SHO_MIN, SHO_MAX = -1.6, 1.6
ELB_MIN, ELB_MAX =  0.2, 2.2
CONTACT_DIST = 0.01

# ───── 7.  INIT ARM POSE ────────────────────────────────────────────
for j, t in ((ARM_SHO, HOME_SHO), (ARM_ELB, HOME_ELB)):
    p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL,
                            targetPosition=t, force=120)
for _ in range(120):
    p.stepSimulation(); time.sleep(1/240)

# ───── 8.  2‑DOF PLANAR IK ─────────────────────────────────────────
def solve_planar(x, z):
    d = max(1e-4, math.hypot(x, z))
    cos_e = (L1**2 + L2**2 - d**2)/(2*L1*L2)
    elb = math.acos(np.clip(cos_e, -1, 1))
    cos_s = (L1**2 + d**2 - L2**2)/(2*L1*d)
    sho = -(math.atan2(z, x) + math.acos(np.clip(cos_s, -1, 1)))
    return (np.clip(sho, SHO_MIN, SHO_MAX),
            np.clip(elb, ELB_MIN, ELB_MAX))

# ───── 9.  STATE MACHINE ───────────────────────────────────────────
phase = "search"; print("phase: search")
lost_counter = 0
MAX_LOST_FRAMES = 60
while True:
    p.stepSimulation(); time.sleep(1/240)

    try:
        rgba = cam_rgba()
        offs = blob_offset(rgba)
        rgb = np.asarray(rgba, np.uint8).reshape(H, W, 4)[..., :3]
        if rgb.sum() == 0:
            print("[WARN] Camera sees nothing. Adjust camera_link pose.")
        cv2.line(rgb, (W//2 - 10, H//2), (W//2 + 10, H//2), (0,255,0), 1)
        cv2.line(rgb, (W//2, H//2 - 10), (W//2, H//2 + 10), (0,255,0), 1)
        cv2.imshow("Robot Camera", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except Exception as e:
        print(f"[CAMERA ERROR] {e}")
        offs = None

    if phase == "search":
        drive(-SEARCH_SPIN, SEARCH_SPIN)
        if offs is not None:
            phase = "approach"; drive(0,0); print("phase: approach")

    elif phase == "approach":
        if offs is None:
            lost_counter += 1
            if lost_counter > MAX_LOST_FRAMES:
                print("❌ Cube lost too long, exiting.")
                break
            phase = "search"; print("phase: search (lost)"); continue
        else:
            lost_counter = 0

        dist_front = closest(robot_id, cube_ids[0])
        if dist_front and dist_front < STOP_DIST:
            print(f"[DEBUG] Switching to TAP: distance = {dist_front:.3f} m")
            phase = "tap"; drive(0,0); print("phase: tap"); continue

        fwd = (FWD_MAX if dist_front is None or dist_front > BRAKE_DIST
               else FWD_MAX * (dist_front - STOP_DIST) / (BRAKE_DIST - STOP_DIST))

        cur_sho = p.getJointState(robot_id, ARM_SHO)[0]
        tgt_sho = np.clip(cur_sho + ARM_GAIN_SHO*offs[0], SHO_MIN, SHO_MAX)
        for j,t in ((ARM_SHO,tgt_sho),(ARM_ELB,1.9)):
            p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL,
                                    targetPosition=t, force=80)

        turn_err = p.getJointState(robot_id, ARM_SHO)[0] - HOME_SHO
        drive(fwd - BASE_TURN_GAIN*turn_err,
              fwd + BASE_TURN_GAIN*turn_err)

    elif phase == "tap":
        if (d_tip := closest(robot_id, cube_ids[0], link_a=LNK_TIP)) is not None and d_tip <= CONTACT_DIST:
            drive(0,0); print(f"✓ Contact: {d_tip:.3f} m"); break

        cube_pos, _ = p.getBasePositionAndOrientation(cube_ids[0])
        sho_pos,  sho_orn = p.getLinkState(robot_id, ARM_SHO, True)[:2]
        inv_pos, inv_orn  = p.invertTransform(sho_pos, sho_orn)
        (x_raw, _, z_raw), _ = p.multiplyTransforms(inv_pos, inv_orn, cube_pos, [0,0,0,1])
        v_len = math.hypot(x_raw, z_raw)
        face_offset = CUBE_HALF + TIP_RADIUS - AIM_MARGIN
        d_target = max(v_len - face_offset, 0.0)
        scale = d_target / v_len if v_len > 1e-6 else 0
        x, z = x_raw*scale, z_raw*scale

        tgt_sho, tgt_elb = solve_planar(x, z)
        print(f"[IK] Target xz = ({x:.3f}, {z:.3f}) → SHO={tgt_sho:.2f}, ELB={tgt_elb:.2f}")
        for j,t in ((ARM_SHO,tgt_sho),(ARM_ELB,tgt_elb)):
            p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL,
                                    targetPosition=t, force=200)
        drive(CREEP_SPEED, CREEP_SPEED)

# ───── CLEANUP ─────────────────────────────────────────────────────
try:
    drive(0,0)
finally:
    if p.isConnected():
        p.disconnect()
    cv2.destroyAllWindows()
    print("Finished.")
