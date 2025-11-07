import math, time
import numpy as np
import pybullet as p
import pybullet_data
AREA_W = 10.0
AREA_H = 6.0
LANE_SPACING = 1.0
WHEELBASE = 0.8
BODY_LEN  = 1.0
BODY_WID  = 0.6
WHEEL_DIAM = 0.26
WHEEL_WID  = 0.10
WHEEL_RADIUS = WHEEL_DIAM * 0.5
V_BASE = 1.6
DT = 1.0 / 60.0          
LOOKAHEAD = 0.8
MAX_STEER_DEG = 60.0
MAX_STEER_RATE = math.radians(60.0)
SENSOR_RADIUS = 1.6
GOAL_TOL = 0.12
WAYPOINT_TOL = 0.18
MAX_SEG_LOOK = 2
CAM_YAW, CAM_PITCH, CAM_DIST = 45, -30, 8.0
def generate_lawnmower(x_min, x_max, y_min, y_max, spacing):
    pts, y, direction = [], y_min, 1
    while y <= y_max + 1e-9:
        if direction == 1: pts += [(x_min, y), (x_max, y)]
        else:               pts += [(x_max, y), (x_min, y)]
        y += spacing
        direction *= -1
    out = []
    for pnt in pts:
        if not out or (abs(out[-1][0]-pnt[0])>1e-9 or abs(out[-1][1]-pnt[1])>1e-9):
            out.append(pnt)
    return np.array(out, dtype=float)
path = generate_lawnmower(0.0, AREA_W, 0.0, AREA_H, LANE_SPACING)
def find_lookahead_point_from(path, pos, lookahead, start_seg, heading, max_seg_look=10):
    px, py = pos
    hx, hy = heading
    n = len(path)
    candidates = []
    end_seg = min(n - 1, start_seg + max_seg_look)
    for i in range(start_seg, end_seg):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        dx, dy = x2 - x1, y2 - y1
        a = dx*dx + dy*dy
        if a < 1e-12: continue
        b = 2 * (dx*(x1 - px) + dy*(y1 - py))
        c = (x1 - px)**2 + (y1 - py)**2 - lookahead**2
        disc = b*b - 4*a*c
        if disc < 0: continue
        s = math.sqrt(disc)
        for t in [(-b - s)/(2*a), (-b + s)/(2*a)]:
            if 0.0 <= t <= 1.0:
                lx, ly = x1 + t*dx, y1 + t*dy
                if (lx - px)*hx + (ly - py)*hy > 1e-9:
                    candidates.append((i, t, np.array([lx, ly])))
    if candidates:
        i_best, t_best, p_best = min(candidates, key=lambda k: (k[0], k[1]))
        return p_best, i_best, t_best
    if start_seg < n - 1:
        x1, y1 = path[start_seg]
        x2, y2 = path[start_seg + 1]
        dx, dy = x2 - x1, y2 - y1
        seg_len2 = dx*dx + dy*dy
        if seg_len2 < 1e-12:
            return find_lookahead_point_from(path, pos, lookahead, start_seg + 1, heading, max_seg_look)
        t0 = ((px - x1)*dx + (py - y1)*dy) / seg_len2
        seg_len = math.sqrt(seg_len2)
        t_tar = min(1.0, max(0.0, t0) + lookahead/seg_len)
        vx, vy = x1 + t_tar*dx, y1 + t_tar*dy
        if (vx - px)*hx + (vy - py)*hy <= 0.0:
            t_tar = min(1.0, t_tar + 1e-3)
            vx, vy = x1 + t_tar*dx, y1 + t_tar*dy
        return np.array([vx, vy]), start_seg, t_tar
    return np.array(path[-1]), n - 2, 1.0
def pure_pursuit_steer_from(path, rear_pos, yaw, lookahead, wheelbase, max_steer_rad, start_seg):
    heading = (math.cos(yaw), math.sin(yaw))
    look_pt, seg_idx, _ = find_lookahead_point_from(path, rear_pos, lookahead, start_seg, heading)
    dx = look_pt[0] - rear_pos[0]
    dy = look_pt[1] - rear_pos[1]
    x_f =  math.cos(yaw)*dx + math.sin(yaw)*dy
    y_l = -math.sin(yaw)*dx + math.cos(yaw)*dy
    if x_f <= 0.0:
        look_pt, seg_idx, _ = find_lookahead_point_from(path, rear_pos, lookahead, min(start_seg+1, len(path)-2), heading)
        dx = look_pt[0] - rear_pos[0]; dy = look_pt[1] - rear_pos[1]
        x_f =  math.cos(yaw)*dx + math.sin(yaw)*dy
        y_l = -math.sin(yaw)*dx + math.cos(yaw)*dy
    if abs(x_f) < 1e-9 and abs(y_l) < 1e-9:
        return 0.0, look_pt, seg_idx
    curvature = 2.0 * y_l / (lookahead**2)
    steer = math.atan(curvature * wheelbase)
    steer = max(-max_steer_rad, min(max_steer_rad, steer))
    return steer, look_pt, seg_idx
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setTimeStep(DT)
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf", [0, 0, 0])
p.resetDebugVisualizerCamera(
    cameraDistance=CAM_DIST,
    cameraYaw=CAM_YAW,
    cameraPitch=CAM_PITCH,
    cameraTargetPosition=[AREA_W*0.5, AREA_H*0.5, 0.0]
)
grid_color = [0.8, 0.8, 0.8]
for gx in np.arange(0, AREA_W+1, 1.0):
    p.addUserDebugLine([gx, 0, 0.001], [gx, AREA_H, 0.001], grid_color, lineWidth=0.5, lifeTime=0)
for gy in np.arange(0, AREA_H+1, 1.0):
    p.addUserDebugLine([0, gy, 0.001], [AREA_W, gy, 0.001], grid_color, lineWidth=0.5, lifeTime=0)
for a,b in [([0,0,0.01],[AREA_W,0,0.01]),
            ([AREA_W,0,0.01],[AREA_W,AREA_H,0.01]),
            ([AREA_W,AREA_H,0.01],[0,AREA_H,0.01]),
            ([0,AREA_H,0.01],[0,0,0.01])]:
    p.addUserDebugLine(a,b,[0.4,0.4,0.4],2)
for i in range(len(path)-1):
    a = [path[i,0],   path[i,1],   0.02]
    b = [path[i+1,0], path[i+1,1], 0.02]
    p.addUserDebugLine(a, b, [0.0, 0.8, 0.0], 2, lifeTime=0)
robot = p.loadURDF(
    "simple_car.urdf",
    basePosition=[0.1, 0.1, WHEEL_RADIUS + 0.01],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    useFixedBase=False
)
def attach_saw_to_robot(robot_id):
    saw_offset = [0.05, 0.0, 0.05] 
    saw_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[0.03, 1.1, 0.02], 
        rgbaColor=[1, 0, 0, 1]
    )
    saw_collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[0.03, 1.1, 0.02]
    )
    saw_body = p.createMultiBody(
        baseMass=0.1,
        baseVisualShapeIndex=saw_visual,
        baseCollisionShapeIndex=saw_collision,
        basePosition=[0, 0, 0]
    )
    p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=-1,
        childBodyUniqueId=saw_body,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=saw_offset,
        childFramePosition=[0, 0, 0]
    )
attach_saw_to_robot(robot)
num_joints = p.getNumJoints(robot)
joint_map = {p.getJointInfo(robot, ji)[1].decode("utf-8"): ji for ji in range(num_joints)}
print("Joint map:", joint_map)
p.changeVisualShape(robot, -1, rgbaColor=[0.0, 0.0, 1.0, 0.7])
p.changeDynamics(robot, -1, linearDamping=1.0, angularDamping=1.0)
rear_x, rear_y, yaw = 0.5, 0.5, 0.0
steer = 0.0
trajectory = [(rear_x, rear_y)]
path_done = False
current_seg = 0
max_steer = math.radians(MAX_STEER_DEG)
prev_steer = 0.0
traj_color = [0.0, 0.4, 1.0]
heading_color = [0.0, 0.0, 0.0]
look_color = [1.0, 0.0, 0.0]
heading_line_id = None
look_point_id = None
sensor_circle_ids = []
panel_text_id = None
def draw_circle(center, radius, color, seg=40, z=0.03, lw=1, life=0.1):
    ids, cx, cy = [], center[0], center[1]
    pts = [[cx + radius*math.cos(2*math.pi*k/seg),
            cy + radius*math.sin(2*math.pi*k/seg), z] for k in range(seg+1)]
    for i in range(seg):
        ids.append(p.addUserDebugLine(pts[i], pts[i+1], color, lw, life))
    return ids
def set_debug_text(text, pos=[0.02, 0.02, 0.5], color=[1,1,1], size=1.2):
    global panel_text_id
    if panel_text_id is not None:
        p.removeUserDebugItem(panel_text_id)
    panel_text_id = p.addUserDebugText(
        text, textPosition=pos, textColorRGB=color, textSize=size,
        lifeTime=0.1, parentObjectUniqueId=robot, parentLinkIndex=-1
    )
print("Starting PyBullet URDF pure-pursuit sim with front saw...")
while p.isConnected():
    if path_done:
        time.sleep(0.2)
        p.stepSimulation()
        continue
    desired_steer, look_pt, seg_found = pure_pursuit_steer_from(
        path, (rear_x, rear_y), yaw, LOOKAHEAD, WHEELBASE, max_steer, current_seg
    )
    current_seg = max(current_seg, seg_found)
    max_dsteer = MAX_STEER_RATE * DT
    dsteer = desired_steer - prev_steer
    if dsteer > max_dsteer: desired_steer = prev_steer + max_dsteer
    elif dsteer < -max_dsteer: desired_steer = prev_steer - max_dsteer
    prev_steer = desired_steer
    steer_abs = abs(desired_steer)
    v = V_BASE * max(0.35, 1.0 - (steer_abs / max_steer))
    omega = (v / WHEELBASE) * math.tan(steer)
    rear_x += v * math.cos(yaw) * DT
    rear_y += v * math.sin(yaw) * DT
    yaw    += omega * DT
    steer   = desired_steer
    trajectory.append((rear_x, rear_y))
    while current_seg < len(path)-1:
        nx, ny = path[current_seg+1]
        if math.hypot(nx - rear_x, ny - rear_y) < WAYPOINT_TOL: current_seg += 1
        else: break
    base_pos = [rear_x, rear_y, WHEEL_RADIUS + 0.05]
    base_orn = p.getQuaternionFromEuler([0, 0, yaw])
    p.resetBasePositionAndOrientation(robot, base_pos, base_orn)
    wheel_velocity = v / max(WHEEL_RADIUS, 1e-6)
    for jn in ["wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"]:
        if jn in joint_map:
            p.setJointMotorControl2(robot, joint_map[jn],
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=wheel_velocity, force=5.0)
    if abs(steer) > 1e-6:
        R = WHEELBASE / math.tan(steer)
        if steer > 0:
            delta_left  = math.atan(WHEELBASE / (R - BODY_WID/2))
            delta_right = math.atan(WHEELBASE / (R + BODY_WID/2))
        else:
            R = abs(R)
            delta_left  = -math.atan(WHEELBASE / (R + BODY_WID/2))
            delta_right = -math.atan(WHEELBASE / (R - BODY_WID/2))
    else:
        delta_left = delta_right = 0.0
    if "steer_fl_joint" in joint_map:
        p.setJointMotorControl2(robot, joint_map["steer_fl_joint"],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=delta_left, force=10.0)
    if "steer_fr_joint" in joint_map:
        p.setJointMotorControl2(robot, joint_map["steer_fr_joint"],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=delta_right, force=10.0)
    head_len = 0.6
    hx = rear_x + head_len * math.cos(yaw)
    hy = rear_y + head_len * math.sin(yaw)
    if heading_line_id: p.removeUserDebugItem(heading_line_id)
    heading_line_id = p.addUserDebugLine([rear_x, rear_y, 0.05], [hx, hy, 0.05], [0,0,0], 3, 0.1)
    if look_point_id: p.removeUserDebugItem(look_point_id)
    look_point_id = p.addUserDebugLine([look_pt[0], look_pt[1], 0.08],[look_pt[0], look_pt[1], 0.28],[1,0,0],6,0.1)
    for cid in sensor_circle_ids: p.removeUserDebugItem(cid)
    sensor_circle_ids = draw_circle((rear_x, rear_y), SENSOR_RADIUS, [0.5,0.5,0.5], seg=36, z=0.03, lw=1, life=0.12)
    draw_circle((rear_x, rear_y), LOOKAHEAD, [0.0,0.4,1.0], seg=36, z=0.031, lw=2, life=0.12)
    if len(trajectory) >= 2:
        x1,y1 = trajectory[-2]; x2,y2 = trajectory[-1]
        p.addUserDebugLine([x1,y1,0.025],[x2,y2,0.025], traj_color, 2, 0)
    dx = v * math.cos(yaw) * DT
    dy = v * math.sin(yaw) * DT
    dtheta = omega * DT
    Rcurv = (WHEELBASE / math.tan(steer)) if abs(steer)>1e-6 else float('inf')
    text = (f"Sim (PyBullet + Gergaji)\n"
            f"rear=({rear_x:.2f},{rear_y:.2f}) m\n"
            f"yaw={math.degrees(yaw):.1f}° | steer={math.degrees(steer):.1f}°\n"
            f"seg={current_seg}  v={v:.2f} m/s\n"
            f"R={'∞' if Rcurv==float('inf') else f'{Rcurv:.2f} m'}")
    set_debug_text(text, color=[1,0,1])
    if math.hypot(path[-1,0]-rear_x, path[-1,1]-rear_y) < GOAL_TOL and current_seg >= len(path)-2:
        print("Reached final goal — stopping.")
        path_done = True
    p.stepSimulation()
    time.sleep(DT)
