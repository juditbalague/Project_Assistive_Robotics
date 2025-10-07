import os
import time
import socket
import math
import numpy as np

from robodk.robolink import *   # API RoboDK
from robodk.robomath import *   # Funcions útils (Pose_2_UR, transl, etc.)

# -----------------------------
# CONSTANTS (definides abans d'usar-les)
ROBOT_IP   = '192.168.1.5'
ROBOT_PORT = 30002

# Paràmetres UR (articular) -> rad/s i rad/s^2
accel_mss = 1.20   # a per movej (rad/s^2)
speed_ms  = 0.75   # v per movej (rad/s)
blend_r   = 0.0    # blending a movej/movel (0 = sense blend)

# Durades d’espera després d’enviar moviments
timej     = 2.0    # espera típica després d’un movej
timel     = 2.0    # espera típica després d’un movel

# -----------------------------
# Obrim RoboDK i el projecte
RDK = Robolink()

# carregar rdk
rdk_candidates = [
    "/mnt/data/Assistive_UR5e.rdk",
    os.path.abspath("src/roboDK/Assistive_UR5e.rdk")
]
for rdk_path in rdk_candidates:
    if os.path.exists(rdk_path):
        RDK.AddFile(rdk_path)
        break
else:
    raise FileNotFoundError("No s'ha trobat cap projecte .rdk vàlid.")

# -----------------------------
# Items principals
robot = RDK.Item("UR5e")
base  = RDK.Item("UR5e Base")
tool  = RDK.Item("Hand")

robot.setPoseFrame(base)
robot.setTool(tool)       # coherent amb RoboDK
robot.setSpeed(20)        # velocitat lineal "per defecte" (mm/s)

# Targets principals
Init_target          = RDK.Item('Init')
Pick_drug_target     = RDK.Item('Pick_drug')
Move_drug_target     = RDK.Item('Move_drug')
Drop_drug_target     = RDK.Item('Drop_drug')
Mix_solution_target  = RDK.Item('Mix_solution')

# Targets de Wave
Wave_start = RDK.Item('Wave_start')
Wave_left  = RDK.Item('Wave_left')
Wave_right = RDK.Item('Wave_right')

# Targets de Sanitizer
App_sanitizer   = RDK.Item('App_sanitizer')
Press_sanitizer = RDK.Item('Press_sanitizer')
Ret_sanitizer   = RDK.Item('Ret_sanitizer')

# Targets d'Adjust light
App_light    = RDK.Item('App_light')
Adjust_left  = RDK.Item('Adjust_left')
Adjust_right = RDK.Item('Adjust_right')

# -----------------------------
# Helpers
def fmt_list(vals):
    return "[" + ",".join(f"{v:.6f}" for v in vals) + "]"

def joints_list_rad(target_item: Item):
    """
    Retorna la llista de juntes en radians a partir d'un Target de RoboDK.
    """
    if not target_item.Valid():
        raise ValueError(f"Target no vàlid: {target_item.Name()}")
    j_deg_mat  = target_item.Joints()            # Mat(1x6) en graus
    j_rad_list = list(np.radians(j_deg_mat)[0])  # -> [rad,...]
    return j_rad_list

def pose_p_from_target(target_item: Item):
    """
    Converteix la Pose 4x4 de RoboDK a p[x,y,z,rx,ry,rz] en metres/radians (UR).
    """
    pose = target_item.Pose()
    x,y,z,rx,ry,rz = Pose_2_UR(pose)  # ja en m i rad per UR
    return [x,y,z,rx,ry,rz]

def ur_movej_cmd(joint_list_rad, accel=accel_mss, speed=speed_ms, r=blend_r):
    """
    movej sense t per tal que s'usïn a i v; fem l'espera amb sleep().
    """
    return f"movej({fmt_list(joint_list_rad)}, a={accel:.5f}, v={speed:.5f}, r={r:.4f})"

def ur_movel_p_cmd(p_list, accel=0.75, speed=0.25, r=0.0):
    """
    movel en espai cartesià amb p[...]; velocitats lineals (m/s, m/s^2).
    """
    return f"movel(p{fmt_list(p_list)}, a={accel:.5f}, v={speed:.5f}, r={r:.4f})"

def send_ur_script(sock, command):
    sock.send((command + "\n").encode())

def connect_ur(ip, port, timeout=2.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    s.connect((ip, port))
    return s

# URScript util
SET_TCP     = "set_tcp(p[0.000000, 0.000000, 0.050000, 0.000000, 0.000000, 0.000000])"
SET_PAYLOAD = "set_payload(1.00, [0,0,0])"  # ajusta-ho a la massa real de la mà

# -----------------------------
# Seqüències (SIM + URScript)
def do_init(sock=None):
    print("Init")
    if Init_target.Valid():
        robot.MoveL(Init_target, True)  # simulació
    else:
        print("Init target not found!")

    if sock:
        send_ur_script(sock, SET_TCP)
        # send_ur_script(sock, SET_PAYLOAD)  # si cal
        jlist = joints_list_rad(Init_target)
        send_ur_script(sock, ur_movej_cmd(jlist))
        time.sleep(timej)

def do_wave(sock=None, cycles=3):
    print("Wave")
    if not (Wave_start.Valid() and Wave_left.Valid() and Wave_right.Valid()):
        print("Wave targets not found!")
        return

    # SIM: entrada més lenta al start
    robot.setSpeed(20, 100, 1, 2)
    robot.MoveJ(Wave_start)
    robot.setSpeed(20, 100, 60, 120)

    # UR: moure a wave_start i fer esquerra-dreta
    if sock:
        # start
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(Wave_start)))
        time.sleep(timej)
        # cicles
        for _ in range(cycles):
            send_ur_script(sock, ur_movej_cmd(joints_list_rad(Wave_left)))
            time.sleep(timej)
            send_ur_script(sock, ur_movej_cmd(joints_list_rad(Wave_right)))
            time.sleep(timej)

        # tornar a start: més lent i sense blend
        time.sleep(0.5)
        send_ur_script(sock, ur_movej_cmd(
            joints_list_rad(Wave_start),
            accel=0.1,   
            speed=0.1,   
            r=0.0
        ))
        time.sleep(timej) 

    # SIM: tres oscil·lacions
    for _ in range(cycles):
        robot.MoveL(Wave_left)
        robot.MoveL(Wave_right)
    robot.setSpeed(20, 100, 5, 10)
    robot.MoveJ(Wave_start)
    robot.setSpeed(20, 100, 60, 120)

def do_press_sanitizer(sock=None):
    print("Press sanitizer")
    if not (App_sanitizer.Valid() and Press_sanitizer.Valid() and Ret_sanitizer.Valid()):
        print("Sanitizer targets not found!")
        return

    # SIM
    robot.setSpeed(20)
    robot.MoveL(App_sanitizer, True)
    robot.setSpeed(10)
    robot.MoveL(Press_sanitizer, True)
    time.sleep(1.0)
    robot.setSpeed(20)
    robot.MoveL(Ret_sanitizer, True)

    # UR
    if sock:
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(App_sanitizer)))
        time.sleep(timej)
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(Press_sanitizer)))
        time.sleep(0.5)  # pressió breu
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(Ret_sanitizer)))
        time.sleep(timej)

def do_adjust_light(sock=None):
    print("Adjust light")
    if not (App_light.Valid() and Adjust_left.Valid() and Adjust_right.Valid()):
        print("Adjust light targets not found!")
        return

    # SIM
    robot.setSpeed(20)
    robot.MoveL(App_light, True)
    robot.setSpeed(15)
    robot.MoveL(Adjust_left, True)
    robot.MoveL(Adjust_right, True)
    robot.MoveL(App_light, True)

    # UR
    if sock:
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(App_light)))
        time.sleep(timej)
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(Adjust_left)))
        time.sleep(timej)
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(Adjust_right)))
        time.sleep(timej)
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(App_light)))
        time.sleep(timej)

def do_pick_move_drop(sock=None):
    print("Pick/Move/Drop drug")
    # SIM
    if Pick_drug_target.Valid():
        robot.MoveL(Pick_drug_target, True)
    if Move_drug_target.Valid():
        robot.MoveL(Move_drug_target, True)
    if Drop_drug_target.Valid():
        robot.setSpeed(20,100,5,10)
        robot.MoveL(Drop_drug_target, True)
        robot.setSpeed(20, 100, 5, 10) 

    # UR
    if sock:
        if Pick_drug_target.Valid():
            send_ur_script(sock, ur_movej_cmd(joints_list_rad(Pick_drug_target)))
            time.sleep(timej)
        if Move_drug_target.Valid():
            send_ur_script(sock, ur_movej_cmd(joints_list_rad(Move_drug_target)))
            time.sleep(timej)
        if Drop_drug_target.Valid():
            send_ur_script(sock, ur_movej_cmd(joints_list_rad(Drop_drug_target), accel=0.1, speed=0.1, r=0.0))
            time.sleep(timej)

def do_mix_solution(sock=None):
    """
    SIM: agitació circular X-Z al voltant del centre (Mix_solution_target).
    UR: mateix patró amb movel p[...] i blending suau.
    """
    print("Mix solution")
    if not Mix_solution_target.Valid():
        print("Mix_solution target not found!")
        return

    # --------- SIM (ràpid i fluid amb rounding) ----------
    center_pose = Mix_solution_target.Pose()
    radius_mm = 30.0
    turns = 3
    steps = 36
    total_points = turns * steps

    previous_speed = 200
    robot.setSpeed(20)      # lineal alta per a l’agitació
    robot.setRounding(10.0)   # mm de blending per suavitzar

    try:
        approach = center_pose * transl(0, 0, 50)
        robot.MoveL(approach, True)
        robot.MoveL(center_pose, True)

        for i in range(total_points):
            angle = 2.0 * math.pi * (i / steps)
            y = radius_mm * math.sin(angle)
            z = radius_mm * math.cos(angle)
            new_pose = center_pose * transl(0, y, z)  # X fix (0)
            robot.MoveL(new_pose, True)

        robot.MoveL(center_pose, True)
        robot.MoveL(approach, True)
        print("Mix solution done (SIM, X-Z plane)")
    except Exception as e:
        print("Error during Mix_solution() SIM:", e)
    finally:
        robot.setRounding(0.0)
        robot.setSpeed(previous_speed)

    # --------- UR REAL ----------
    if sock:
        # Paràmetres d’agitació (m, rad)
        radius_m = 0.030     # 30 mm
        turns    = 3
        steps    = 36
        a_lin    = 0.75
        v_lin    = 0.25
        r_blend  = 0.005     # 5 mm

        # 1) Apropa’t al centre en articular
        q_center = joints_list_rad(Mix_solution_target)
        send_ur_script(sock, ur_movej_cmd(q_center, accel_mss, speed_ms, r=0.0))
        time.sleep(timej)

        # 2) Baixa al centre en cartesià
        cx, cy, cz, rx, ry, rz = pose_p_from_target(Mix_solution_target)
        send_ur_script(sock, ur_movel_p_cmd([cx, cy, cz, rx, ry, rz], a_lin, v_lin, r=0.0))
        time.sleep(timel)

        # 3) Trajecte circular amb blending (punt a punt)
        total_points = turns * steps
        for i in range(total_points):
            ang = 2.0 * math.pi * (i / steps)
            y   = cy + radius_m * math.sin(ang)
            z   = cz + radius_m * math.cos(ang)
            p   = [cx, y, z, rx, ry, rz]
            send_ur_script(sock, ur_movel_p_cmd(p, a_lin, v_lin, r=r_blend))
            # sleep curt: el blending ja suavitza i encadena
            time.sleep(0.02)

        # 4) Torna al centre i espera
        send_ur_script(sock, ur_movel_p_cmd([cx, cy, cz, rx, ry, rz], a_lin, v_lin, r=0.0))
        time.sleep(timel)

# -----------------------------
def main():
    # Prova connexió al robot
    sock = None
    try:
        sock = connect_ur(ROBOT_IP, ROBOT_PORT)
        print("Connexió UR OK")
        send_ur_script(sock, SET_TCP)
        send_ur_script(sock, SET_PAYLOAD)  # ajusta payload si cal
    except Exception as e:
        print(f"No s'ha pogut connectar al robot ({ROBOT_IP}:{ROBOT_PORT}). Continuo en simulació. Detall: {e}")
        sock = None

    # Seqüència completa
    do_init(sock)
    do_wave(sock, cycles=3)
    do_press_sanitizer(sock)
    do_adjust_light(sock)
    do_pick_move_drop(sock)
    do_mix_solution(sock)

    # Tanca socket si cal
    if sock:
        try:
            sock.close()
        except:
            pass

if __name__ == "__main__":
    main()
