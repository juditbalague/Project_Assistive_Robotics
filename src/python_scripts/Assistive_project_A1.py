import os
import time
import socket
import math
import numpy as np

from robodk.robolink import *   # API RoboDK
from robodk.robomath import *   # Funcions útils

# -----------------------------
# CONSTANTS (definides abans d'usar-les)
ROBOT_IP   = '192.168.1.5'
ROBOT_PORT = 30002

# Paràmetres UR (movej)
accel_mss = 1.20
speed_ms  = 0.75
blend_r   = 0.0
timej     = 6.0
timel     = 4.0   # si vols un movej "cronometral" més curt

# -----------------------------
# Obrim RoboDK i el projecte
RDK = Robolink()

# Intenta carregar el RDK que m’has passat; si no, prova el camí antic del teu repo
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
robot.setPoseTool(tool)
robot.setSpeed(20)

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
def joints_list_rad(target_item: Item):
    """
    Retorna la llista de juntes en radians a partir d'un Target de RoboDK,
    exactament com vols: list(np.radians(Target.Joints())[0]).
    (Ens assegurem que funcioni robust: Mat -> list -> radians)
    """
    if not target_item.Valid():
        raise ValueError(f"Target no vàlid: {target_item.Name()}")
    j_deg_mat = target_item.Joints()             # Mat 1x6
    j_deg_list = list(np.radians(j_deg_mat)[0])           # [deg,...]
    j_rad_list = j_deg_list    # [rad,...]
    print(j_rad_list)
    return j_rad_list

def ur_movej_cmd(joint_list_rad, accel=accel_mss, speed=speed_ms, t=timej, r=blend_r):
    return f"movej({joint_list_rad},{accel:.5f},{speed:.5f},{t:.5f},{r:.4f})"

def send_ur_script(sock, command):
    sock.send((command + "\n").encode())

def connect_ur(ip, port, timeout=2.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    s.connect((ip, port))
    return s

# URScript util
SET_TCP = "set_tcp(p[0.000000, 0.000000, 0.050000, 0.000000, 0.000000, 0.000000])"
SET_PAYLOAD = "set_payload(1.00, [0,0,0])"  # ajusta si cal

# -----------------------------
# Seqüències (SIM + URScript)
def do_init(sock=None):
    print("Init")
    if Init_target.Valid():
        robot.MoveL(Init_target, True)  # simulació
    else:
        print("Init target not found!")

    if sock:
        print(sock)
        send_ur_script(sock, SET_TCP)
        #send_ur_script(sock, SET_PAYLOAD)
        jlist = joints_list_rad(Init_target)
        cmd   = ur_movej_cmd(jlist, accel_mss, speed_ms, timel, blend_r)
        send_ur_script(sock, cmd)
        time.sleep(1.0)

def do_wave(sock=None, cycles=3):
    print("Wave")
    if not (Wave_start.Valid() and Wave_left.Valid() and Wave_right.Valid()):
        print("Wave targets not found!")
        return

    # SIM: entrada més lenta al start
    robot.setSpeed(20, 100, 10, 20)
    robot.MoveJ(Wave_start)
    robot.setSpeed(20, 100, 60, 120)

    # UR: moure a wave_start i fer esquerra-dreta
    if sock:
        # start
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(Wave_start)))
        time.sleep(0.2)
        # cicles
        for _ in range(cycles):
            send_ur_script(sock, ur_movej_cmd(joints_list_rad(Wave_left)))
            send_ur_script(sock, ur_movej_cmd(joints_list_rad(Wave_right)))
        # tornar a start
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(Wave_start)))

    # SIM: tres oscil·lacions
    for _ in range(cycles):
        robot.MoveL(Wave_left)
        robot.MoveL(Wave_right)
    robot.MoveJ(Wave_start)

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
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(Press_sanitizer)))
        time.sleep(0.5)
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(Ret_sanitizer)))

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
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(Adjust_left)))
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(Adjust_right)))
        send_ur_script(sock, ur_movej_cmd(joints_list_rad(App_light)))

def do_pick_move_drop(sock=None):
    print("Pick/Move/Drop drug")
    # SIM
    if Pick_drug_target.Valid():
        robot.MoveL(Pick_drug_target, True)
    if Move_drug_target.Valid():
        robot.MoveL(Move_drug_target, True)
    if Drop_drug_target.Valid():
        # més controlat
        robot.setSpeed(5)
        robot.MoveL(Drop_drug_target, True)
        robot.setSpeed(15)

    # UR
    if sock:
        if Pick_drug_target.Valid():
            send_ur_script(sock, ur_movej_cmd(joints_list_rad(Pick_drug_target)))
        if Move_drug_target.Valid():
            send_ur_script(sock, ur_movej_cmd(joints_list_rad(Move_drug_target)))
        if Drop_drug_target.Valid():
            send_ur_script(sock, ur_movej_cmd(joints_list_rad(Drop_drug_target)))

def do_mix_solution(sock=None):
    """
    UR: farem la posició d'aproximació, centre i sortida amb movej.
    (La trajectòria circular d’agitació detallada la mantens via RoboDK si la vols,
     però així el robot real té ordres vàlides per al "core" del moviment.)
    """
    print("Mix solution")
    if not Mix_solution_target.Valid():
        print("Mix_solution target not found!")
        return

    center_pose = Mix_solution_target.Pose()
    approach    = center_pose * transl(0, 0, 50)

    # SIM (aprox -> centre -> aprox)
    robot.MoveL(approach, True)
    robot.MoveL(center_pose, True)
    robot.MoveL(approach, True)

    # UR (via IK dels targets: fem servir els joints del Target principal; per l'aproximació,
    # fem una solució d'IK ràpida amb la pose 'approach' resolta per RoboDK)
    if sock:
        # Obtenim juntes del target centre
        j_center = joints_list_rad(Mix_solution_target)

        # Per l'aproximació, resolem IK amb la pose 'approach'
        #j_appr_mat = robot.SolveIK(approach)
        #if isinstance(j_appr_mat, Mat):
        #    j_appr = list(np.radians(j_appr_mat))  # assegura rad
        #else:
            # fallback: si no resol, usa el mateix centre
        #    j_appr = j_center
        j_appr = j_center
        send_ur_script(sock, ur_movej_cmd(j_appr))
        send_ur_script(sock, ur_movej_cmd(j_center))
        send_ur_script(sock, ur_movej_cmd(j_appr))

# -----------------------------
def main():
    # Prova connexió al robot
    sock = None
    try:
        sock = connect_ur(ROBOT_IP, ROBOT_PORT)
        print("Connexió UR OK")
        send_ur_script(sock, SET_TCP)
        send_ur_script(sock, SET_PAYLOAD)
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
