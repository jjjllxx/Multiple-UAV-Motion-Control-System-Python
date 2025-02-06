import numpy as np
import structs

def find_key_pts(path):
    path_size = len(path)
    key_pts = [(0, path[0])]
    
    for i in range(1, path_size - 1):
        last_time, _ = key_pts[-1] 
        if abs(path[i - 1].y - path[i].y - path[i].y + path[i + 1].y) > 0.2 and i - 1 != last_time:
            key_pts.append((i, path[i]))

    key_pts.append((path_size - 1, path[-1]))

    return key_pts

def generate_traj(key_pts):   
    coef_x = np.zeros((4, len(key_pts)))
    coef_y = np.zeros((4, len(key_pts)))
    coef_z = np.zeros((4, len(key_pts)))

    for i in range(len(key_pts) - 1):
        time_diff = key_pts[i + 1][0] - key_pts[i][0]
        coef_x[:, i] = solve_coef(key_pts[i][1].x, key_pts[i + 1][1].x, time_diff)
        coef_y[:, i] = solve_coef(key_pts[i][1].y, key_pts[i + 1][1].y, time_diff)
        coef_z[:, i] = solve_coef(key_pts[i][1].z, key_pts[i + 1][1].z, time_diff)
    
    return calc_traj(key_pts, coef_x, coef_y, coef_z)

def slow_traj(key_pts, col_time):
    pos = 0
    up = len(col_time)
    coef_x = np.zeros((4, len(key_pts)))
    coef_y = np.zeros((4, len(key_pts)))
    coef_z = np.zeros((4, len(key_pts)))
    
    for i in range(len(key_pts) - 1):
        while pos < up and col_time[pos] < key_pts[i + 1][0]:
            pos += 1

        key_pts[i + 1] = (key_pts[i + 1][0] + pos - 2, key_pts[i + 1][1])        
        time_diff = key_pts[i + 1][0] - key_pts[i][0]
        coef_x[:, i] = solve_coef(key_pts[i][1].x, key_pts[i + 1][1].x, time_diff)
        coef_y[:, i] = solve_coef(key_pts[i][1].y, key_pts[i + 1][1].y, time_diff)
        coef_z[:, i] = solve_coef(key_pts[i][1].z, key_pts[i + 1][1].z, time_diff)
    
    return calc_traj(key_pts, coef_x, coef_y, coef_z)

def calc_traj(key_pts, coef_x, coef_y, coef_z):
    traj = []
    part = 0
    for i in range(key_pts[-1][0] + 1):
        if i > key_pts[part + 1][0]:
            part += 1
        time_diff = i - key_pts[part][0]
        traj_pt = structs.Point(
            np.dot(coef_x[:, part], [time_diff**3, time_diff**2, time_diff, 1]),
            np.dot(coef_y[:, part], [time_diff**3, time_diff**2, time_diff, 1]),
            np.dot(coef_z[:, part], [time_diff**3, time_diff**2, time_diff, 1])
        )
        traj.append(traj_pt)
    
    return traj

def control_multi_collision(all_traj, all_key_pts, col_limit):
    new_trajs = [all_traj[0]]
    
    for i in range(len(all_traj) - 1):
        col_time = record_collision_time(all_traj[i], all_traj[i + 1], col_limit)
        
        if len(col_time) > 1:
            slowed_traj = slow_traj(all_key_pts[i + 1], col_time)
            new_trajs.append(slowed_traj)
        else:
            new_trajs.append(all_traj[i + 1])
    
    return new_trajs

def record_collision_time(traj1, traj2, col_thresh):
    period = min(len(traj1), len(traj2))
    col_time = []
    for i in range(period):
        if traj1[i].dist_to(traj2[i]) < col_thresh:
            col_time.append(i)
    
    return col_time

def solve_coef(coords, next_coords, time_diff):
    c = np.array([
        [0, 0, 0, 1],
        [time_diff**3, time_diff**2, time_diff, 1],
        [0, 0, 1, 0],
        [3 * time_diff**2, 2 * time_diff, 1, 0]
    ])
    d = np.array([coords, next_coords, 0, 0])
    
    return np.linalg.solve(c, d)

def calc_seg_dist(p1: structs.Point, p2: structs.Point, q1: structs.Point, q2: structs.Point):
    s1 = p2 - p1
    s2 = q2 - q1
    
    denom = s1.dot(s1) * s2.dot(s2) - s1.dot(s2) ** 2
    lambda1 = (s1.dot(s2) * (p1 - q1).dot(s2) - s2.dot(s2) * (p1 - q1).dot(s1)) / denom  # lambda1
    lambda2 = -(s1.dot(s2) * (p1 - q1).dot(s1) - s1.dot(s1) * (p1 - q1).dot(s2)) / denom  # lambda2

    if 0 <= lambda1 <= 1 and 0 <= lambda2 <= 1:
        tmp1 = q1 + lambda2 * s2
        tmp = p1 + lambda1 * s1 - tmp1

        return np.sqrt(tmp.dot(tmp)), tmp1
   
    lambda3 = np.dot(q1 - p1, s1) / s1.dot(s1)
    if 0 <= lambda3 <= 1:
        tmp = q1 - (p1 + lambda3 * s1)
        dist1 = np.sqrt(tmp.dot(tmp))
    else:
        dist1 = np.sqrt(min((q1 - p1).dot(q1 - p1), (q1 - p2).dot(q1 - p2)))

    pt1 = q1

    lambda4 = np.dot(q2 - p1, s1) / s1.dot(s1)
    if 0 <= lambda4 <= 1:
        tmp = q2 - (p1 + lambda4 * s1)
        dist2 = np.sqrt(tmp.dot(tmp))
    else:
        dist2 = np.sqrt(min(np.dot(q2 - p1, q2 - p1), (q2 - p2).dot(q2 - p2)))

    pt2 = q2

    lambda5 = (p1 - q1).dot(s2) / s2.dot(s2)
    if 0 <= lambda5 <= 1:
        tmp = p1 - (q1 + lambda5 * s2)
        dist3 = np.sqrt(tmp.dot(tmp))
        pt3 = q1 + lambda5 * s2
    else:
        dist3 = np.sqrt(min((p1 - q1).dot(p1 - q1), (p1 - q2).dot(p1 -q2)))
        pt3 = q1 if dist3 == (p1 - q1).dot(p1 - q1) else q2

    lambda6 = (p2 - q1).dot(s2) / s2.dot(s2)
    if 0 <= lambda6 <= 1:
        tmp = p2 - (q1 + lambda6 * s2)
        dist4 = np.sqrt(tmp.dot(tmp))
        pt4 = q1 + lambda6 * s2
    else:
        dist4 = np.sqrt(min((p2 - 1).dot(p2 - p1), (p2 - q2).dot(p2 - q2)))
        pt4 = q1 if dist3 == (p2 - 1).dot(p2 - p1) else q2

    min_dist = min([dist1, dist2, dist3, dist4])

    if min_dist == dist1:
        return dist1, pt1
    
    if min_dist == dist2:
        return dist2, pt2

    if min_dist == dist3:
        return dist3, pt3

    return dist4, pt4

def is_path_collided(path1, path2, col_thres):
    for i in range(len(path1) - 1):
        for j in range(len(path2) - 1):
            dist, col_pt = calc_seg_dist(path1[i], path1[i + 1], path2[j], path2[j + 1])
            if dist < col_thres:
                return i, col_pt

    return -1, None
