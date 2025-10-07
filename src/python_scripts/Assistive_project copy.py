from robodk import robolink, robomath
import time, socket
import numpy as np

# Connection to RoboDK
RDK = robolink.Robolink()
robot = RDK.Item('UR5e')
tool = RDK.Item('Hand')
base = RDK.Item('UR5e Base')
robot.setPoseFrame(base)
robot.setTool(tool)

# Principal Targets
Init = RDK.Item('Init')
Pick_drug = RDK.Item('Pick_drug')
Move_drug = RDK.Item('Move_drug')
Drop_drug = RDK.Item('Drop_drug')
Mix_solution = RDK.Item('Mix_solution')

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

# UR5e real robot connection
ROBOT_IP = "192.168.1.5"
ROBOT_PORT = 30002
robot_socket = None

# Motion parameters
accel_mss = 1.2
speed_ms = 0.75
blend_r = 0.0
timej = 6
timel = 4

# URScript commands
set_tcp = "set_tcp(p[0.000000,0.000000,0.050000,0.000000,0.000000,0.000000])"

# Define positions of joints
angs1=np.radians(Init.Joints()) 
angsr1=list(angs1[0])
movej_Init = f"movel({angsr1}, {accel_mss}, {speed_ms},{timel},0.0)"

angs2=np.radians(Pick_drug.Joints()) 
angsr2=list(angs2[0])
movel_Pick_drug = f"movel({angsr2}, {accel_mss}, {speed_ms},{timel},0.0)"

angs3=np.radians(Move_drug.Joints()) 
angsr3=list(angs3[0])
movel_Move_drug = f"movel({angsr3}, {accel_mss}, {speed_ms},{timel},0.0)"

angs4=np.radians(Drop_drug.Joints()) 
angsr4=list(angs4[0])
movel_Drop_drug = f"movel({angsr4}, {accel_mss}, {speed_ms},{timel},0.0)"

angs5=np.radians(Mix_solution.Joints()) 
angsr5=list(angs5[0])
movel_Mix_solution = f"movel({angsr5}, {accel_mss}, {speed_ms},{timel},0.0)"

angs6=np.radians(Wave_start.Joints()) 
angsr6=list(angs6[0])
movel_Wave_start = f"movel({angsr6}, {accel_mss}, {speed_ms},{timel},0.0)"

angs7=np.radians(Wave_left.Joints()) 
angsr7=list(angs7[0])
movel_Wave_left = f"movel({angsr7}, {accel_mss}, {speed_ms},{timel},0.0)"

angs8=np.radians(Wave_right.Joints()) 
angsr8=list(angs8[0])
movel_Wave_right = f"movel({angsr8}, {accel_mss}, {speed_ms},{timel},0.0)"

angs9=np.radians(App_sanitizer.Joints()) 
angsr9=list(angs9[0])
movel_App_sanitizer = f"movel({angsr9}, {accel_mss}, {speed_ms},{timel},0.0)"

angs10=np.radians(Press_sanitizer.Joints()) 
angsr10=list(angs10[0])
movel_Press_sanitizer = f"movel({angsr9}, {accel_mss}, {speed_ms},{timel},0.0)"

angs11=np.radians(Ret_sanitizer.Joints()) 
angsr11=list(angs11[0])
movel_Ret_sanitizer = f"movel({angsr11}, {accel_mss}, {speed_ms},{timel},0.0)"

angs12=np.radians(App_light.Joints()) 
angsr12=list(angs12[0])
movel_App_light = f"movel({angsr12}, {accel_mss}, {speed_ms},{timel},0.0)"

angs13=np.radians(Adjust_left.Joints()) 
angsr13=list(angs13[0])
movel_Adjust_left = f"movel({angsr13}, {accel_mss}, {speed_ms},{timel},0.0)"

angs14=np.radians(Adjust_right.Joints()) 
angsr14=list(angs14[0])
movel_Adjust_right = f"movel({angsr14}, {accel_mss}, {speed_ms},{timel},0.0)"

# Socket
def check_robot_port(ip, port):
    global robot_socket
    try:
        robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        robot_socket.settimeout(1)
        robot_socket.connect((ip, port))
        return True
    except (socket.timeout, ConnectionRefusedError):
        return False

def send_ur_script(command):
    robot_socket.send((command + "\n").encode())

def wait_robot(t):
    time.sleep(t)

# Waving sequence
def Waving():
    print("Waving sequence...")
    # Simulation
    robot.MoveJ(Init)
    robot.MoveL(Wave_start)
    robot.MoveL(Wave_left)
    robot.MoveL(Wave_right)
    robot.MoveL(Wave_left)
    robot.MoveL(Wave_right)
    robot.MoveL(Wave_left)
    robot.MoveL(Wave_right)
    time.sleep(1.0)

    # Real robot
    if robot_is_connected:
        send_ur_script(set_tcp)
        wait_robot(1)
        send_ur_script(movej_Init)
        wait_robot(timej)
        send_ur_script(movel_Wave_start)
        wait_robot(timel)
        send_ur_script(movel_Wave_left)
        wait_robot(2)
        send_ur_script(movel_Wave_right)
        wait_robot(timel)
        send_ur_script(movel_Wave_left)
        wait_robot(2)
        send_ur_script(movel_Wave_right)
        wait_robot(timel)
        send_ur_script(movel_Wave_left)
        wait_robot(2)
        send_ur_script(movel_Wave_right)
        wait_robot(timel)
       
    else:
        print("UR5e not connected → Simulation only")
# Sanitizer simulation
def Sanitizer():
    print("Sanitizer sequence...")
    # Simulation
    robot.MoveJ(Init)
    robot.MoveL(App_sanitizer)
    robot.MoveL(Press_sanitizer)
    robot.MoveL(Ret_sanitizer)
    time.sleep(1.0)

    # Real robot
    if robot_is_connected:
        send_ur_script(set_tcp)
        wait_robot(1)
        send_ur_script(movej_Init)
        wait_robot(timej)
        send_ur_script(movel_App_sanitizer)
        wait_robot(timel)
        send_ur_script(movel_Press_sanitizer)
        wait_robot(timel)
        send_ur_script(movel_Ret_sanitizer)
        wait_robot(timel)
       
    else:
        print("UR5e not connected → Simulation only")

# Light adjustment simulation
def Light_adjustment():
    print("Light adjustment sequence...")
    # Simulation
    robot.MoveJ(Init)
    robot.MoveL(App_light)
    robot.MoveL(Adjust_left)
    robot.MoveL(Adjust_right)
    robot.MoveL(App_light)
    time.sleep(1.0)

    # Real robot
    if robot_is_connected:
        send_ur_script(set_tcp)
        wait_robot(1)
        send_ur_script(movej_Init)
        wait_robot(timej)
        send_ur_script(movel_App_light)
        wait_robot(timel)
        send_ur_script(movel_Adjust_left)
        wait_robot(timel)
        send_ur_script(movel_Adjust_right)
        wait_robot(timel)
        send_ur_script(movel_App_light)
        wait_robot(timel)
       
    else:
        print("UR5e not connected → Simulation only")

# Drug simulation
def Drug():
    print("Drug sequence...")
    # Simulation
    robot.MoveJ(Init)
    robot.MoveL(Pick_drug)
    robot.MoveL(Move_drug)
    robot.MoveL(Drop_drug)
    robot.MoveL(Mix_solution)
    time.sleep(1.0)

    # Real robot
    if robot_is_connected:
        send_ur_script(set_tcp)
        wait_robot(1)
        send_ur_script(movej_Init)
        wait_robot(timej)
        send_ur_script(movel_Pick_drug)
        wait_robot(timel)
        send_ur_script(movel_Move_drug)
        wait_robot(timel)
        send_ur_script(movel_Drop_drug)
        wait_robot(timel)
        send_ur_script(movel_Mix_solution)
        wait_robot(timel)
       
    else:
        print("UR5e not connected → Simulation only")

# Main
def main():
    global robot_is_connected
    robot_is_connected = check_robot_port(ROBOT_IP, ROBOT_PORT)

    Waving()
    Sanitizer()
    Light_adjustment()
    Drug()

    if robot_is_connected:
        robot_socket.close()

if _name_ == "_main_":
    main()