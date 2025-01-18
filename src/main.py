# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       idajohn                                                      #
# 	Created:      12/21/2024, 5:18:59 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

from vex import *
brain=Brain()
controller = Controller()  

DriveLU = Motor(Ports.PORT20, True)
DriveLD = Motor(Ports.PORT11, True)
DriveRU = Motor(Ports.PORT10, True)
DriveRD = Motor(Ports.PORT1, True)
DriveLeft = MotorGroup(DriveLD,DriveLU)
DriveRight = MotorGroup(DriveRD,DriveRU)
intakeL = Motor(Ports.PORT2, True)
intakeR = Motor(Ports.PORT5)
intake = MotorGroup(intakeL, intakeR)
armR = Motor(Ports.PORT12,True)
armL = Motor(Ports.PORT3)
arm = MotorGroup(armR, armL)
inertial = Inertial(Ports.PORT13)

clamp = DigitalOut(brain.three_wire_port.g)
doinker = DigitalOut(brain.three_wire_port.h)

color = Optical(Ports.PORT6)
distance = Distance(Ports.PORT7)

INTAKE_REVERSE_TH_PC = 50
alliance = "Red"
trashFlag = False
IsTrue = False
INTAKE_V_PC = 100
state = 0
INTAKE_V_PC = 100
ARM_START_DEG = 25
ARM_SCORE_DEG = 336.6 #335.6
ARM_HANG_DEG = 240

def motorInit():
    print("=====NEW=RUN=====")
    brain.screen.set_font(FontType.PROP60)
    brain.screen.clear_screen()
    brain.screen.draw_rectangle(0,0,479,239,Color.RED)
    brain.screen.set_cursor(1,1)

    wait(5)

    DriveLeft.set_max_torque(100, PERCENT)
    DriveRight.set_max_torque(100, PERCENT)
    DriveLD.set_max_torque(100, PERCENT)
    DriveLU.set_max_torque(100, PERCENT)
    DriveRD.set_max_torque(100, PERCENT)
    DriveRU.set_max_torque(100, PERCENT)

    DriveLeft.set_velocity(100, PERCENT)
    DriveRight.set_velocity(100, PERCENT)
    DriveLD.set_velocity(100, PERCENT)
    DriveLU.set_velocity(100, PERCENT)
    DriveRD.set_velocity(100, PERCENT)
    DriveRU.set_velocity(100, PERCENT)

    intakeL.set_max_torque(100, PERCENT)
    intakeR.set_max_torque(100, PERCENT)
    intake.set_max_torque(100, PERCENT)
    intakeL.set_velocity(INTAKE_V_PC, PERCENT)
    intakeR.set_velocity(INTAKE_V_PC, PERCENT)
    intake.set_velocity(INTAKE_V_PC, PERCENT)

    arm.set_max_torque(100, PERCENT)
    arm.set_velocity(100, PERCENT)
    arm.set_position(0, DEGREES)
    arm.spin_to_position(-ARM_START_DEG, DEGREES)

    arm.set_max_torque(100, PERCENT)
    # arm.set_velocity(100, PERCENT)

    color.set_light(LedStateType.ON)

def calibrate_thingy():
    brain.screen.print("calibrating inertial")
    inertial.calibrate()
    while inertial.is_calibrating():
        sleep(5)
    brain.screen.clear_screen()
    brain.screen.draw_rectangle(0,0,479,239,Color.RED)
    brain.screen.set_cursor(1,1)
    brain.screen.print("waiting for auton")   
    inertial.set_rotation(0)



def PID_Drive(inches, turngoal, V_MX_MAG=60, kp=0.35, kd=0.3, ki=0.01, timeout_flag=True, timeout=6000, tkp=0.8, tkd=0.7, tki=0.01, baseV=60):
    global current_pos_DEG, Distance_DEG, error, velocity, vexcode_brain_precision, vexcode_console_precision, sign
    loop = 0
    DriveLeft.stop()
    DriveRight.stop()
    DriveLeft.set_position(0, DEGREES)    
    DriveRight.set_position(0, DEGREES)
    # setRot()
    if timeout_flag==False:
        timeout_MSEC = abs(inches * 0.1 * 1000)
    else:
        timeout_MSEC= timeout
    V_TH = 2
    velocity = 9999 #preset value so that the loop can begin
    error = 9999#0
    Ierror = 0
    Derror = 0
    pre_error = 0
    # V_MX_MAG = 60                                   
    if inches < 0:
        sign = -1
    else:
        sign = 1
    # Distance_DEG = ((inches) * 720) / 39.37007 #GR=1/2.5
    # Distance_DEG = ((inches) * 900) / 39.37007 #GR=1/2
    Distance_DEG = ((inches) * 830.7692308) / 39.37007 #GR=3/5
    Err_th = 5#10 #Distance_DEG * 0.2
    I_th = Distance_DEG * 0.1
    CONV_DEG_TO_PCT = 100/180
    turnError = 9999
    IturnError = 0
    DturnError = 0
    current_dir_DEG = 0
    preTurnError = 0
    DriveLeft.spin(FORWARD)
    DriveRight.spin(FORWARD)
    DriveLeft.set_position(0, DEGREES)    
    DriveRight.set_position(0, DEGREES)
    brain.timer.clear()
    while (abs(error) > Err_th) and brain.timer.time(MSEC) < timeout_MSEC:
        #adjusting angle
        current_dir_DEG = inertial.rotation(DEGREES)
        turnError = turngoal - current_dir_DEG
        IturnError = IturnError + turnError
        DturnError = turnError - preTurnError
        turnV = turnError * tkp + DturnError * tkd + IturnError * tki
        turnVPCT = turnV * CONV_DEG_TO_PCT
        turnVPCT = min(100, max(-100, turnVPCT))
        lv = turnVPCT
        rv = -1 * turnVPCT
        preTurnError = turnError
        #driving forward
        Lpos = DriveLeft.position(DEGREES)
        Rpos = DriveRight.position(DEGREES)
        current_pos_DEG = (Lpos - Rpos)/2
        # current_pos_DEG = Lpos
        error = Distance_DEG - current_pos_DEG
        Ierror = Ierror + error
        if error > I_th:# or error <= 3:
            Ierror = 0
        Derror = error - pre_error
        if False: #math.fabs(error) > math.fabs(Err_th):
            velocity = baseV * sign
        else:
            velocity = error * kp + Derror * kd + Ierror * ki
            velocity = min(V_MX_MAG, max(-V_MX_MAG, velocity))
        pre_error = error
        final_lv = velocity + lv
        final_rv = velocity + rv
        if final_lv > 100:
            final_lv = 100
        if final_lv < -100:
            final_lv = -100
        if final_rv > 100:
            final_rv = 100
        if final_rv < -100:
            final_rv = -100
        DriveLeft.set_velocity(final_lv, PERCENT)
        DriveRight.set_velocity(-final_rv, PERCENT)
        #debug
        # brain.screen.clear_screen()
        # brain.screen.set_cursor(1, 1)
        # brain.screen.print("d:{0:.1f},v:{1:.1f}".format(Distance_DEG, velocity))
        # brain.screen.set_cursor(2, 1)
        # brain.screen.print("e:{0:.1f},d:{1:.1f}".format(error, Derror))
        # brain.screen.set_cursor(2, 1)
        # brain.screen.print("I:{0:.1f},t:{1:.1f}".format(Ierror, brain.timer.time(SECONDS)))
        print("-------------")
        print("dis:{0:.1f},c:{1:.1f}".format(Distance_DEG, current_pos_DEG))
        print("e:{0:.1f},d:{1:.1f}".format(error, Derror))
        print("I:{0:.1f},t:{1:.1f}".format(Ierror, brain.timer.time(MSEC)))
        print("L:{0:.1f},R:{1:.1f}".format(Lpos, Rpos))
        print("FLV:{0:.1f}".format(final_lv))
        print("OV:{0:.1f}".format(velocity))
        print("loop:{}".format(loop))
        # print("{0:.1f},{1:.1f},{2:.1f},{3:.1f},{4:.1f},{5:.1f}".format(final_lv,error, current_pos_DEG,brain.timer.time(MSEC),inertial.acceleration(YAXIS),current_dir_DEG))
        # print("{0:.1f},{1:.1f},{2:.1f},{3:.1f},{4:.1f},{5:.1f},{6:.1f}".format(lv,turnError, velocity,DturnError,current_dir_DEG,final_lv, final_rv))
        # print("turngoal:{0:.1f},tc:{1:.1f}".format(turngoal, current_dir_DEG))
        # print("te:{0:.1f},td:{1:.1f}".format(turnError, DturnError))
        # print("tI:{0:.1f},t:{1:.1f}".format(IturnError, brain.timer.time(SECONDS)))
        # print("Lv:{0:.1f},Rv:{1:.1f}".format(lv, rv))
        loop = loop + 1
        wait(20, MSEC)
    brain.timer.clear()
    DriveLeft.stop(BRAKE)
    DriveRight.stop(BRAKE)
    print("-=-=End Drive=-=-")

def PID_Turn(degrees, kp=0.8, kd=0.9, ki=0.01, runtime=5, err_th=2, V_TH=3):
    brain.timer.clear()
    DriveLeft.stop()
    DriveRight.stop()
    DriveLeft.set_position(0, DEGREES)
    DriveRight.set_position(0, DEGREES)
    turngoal = degrees
    CONV_DEG_TO_PCT = 100/180
    vPCT = 9999
    error = 9999
    Ierror = 0
    pre_error = 0
    Derror = 0
    I_th = degrees * 0.1
    V_TH = 3
    err_th = 3#5
    DriveLeft.spin(FORWARD)
    DriveRight.spin(REVERSE)
    brain.screen.clear_screen() 
    brain.timer.clear()
    while ((math.fabs(vPCT) > V_TH) or (math.fabs(error) > err_th)) and (brain.timer.time(SECONDS) < runtime):
        # current_dir_DEG = brain_inertial.rotation(DEGREES)
        current_dir_DEG = inertial.rotation(DEGREES)
        error = degrees - current_dir_DEG
        Ierror = Ierror + error
        if error <= err_th or error > I_th:
            Ierror = 0
        Derror = error - pre_error
        velocity = error * kp + Derror * kd + Ierror * ki
        vPCT = velocity * CONV_DEG_TO_PCT
        vPCT = min(100, max(-100, vPCT))
        lv = vPCT 
        rv = vPCT
        pre_error = error
        DriveLeft.set_velocity(lv, PERCENT)
        DriveRight.set_velocity(rv, PERCENT)
        print("===========")
        print("d:{0:.1f},c:{1:.1f}".format(degrees,current_dir_DEG))
        print("P:{0:.1f},D:{1:.1f}".format(error, Derror))
        print("I:{0:.1f},V:{1:.1f}".format(Ierror, vPCT))
        print("t:{0:.1f}".format(brain.timer.time(SECONDS)))
        # brain.screen.set_cursor(1, 1)
        # brain.screen.print("d:{0:.1f},c:{1:.1f}".format(degrees,current_dir_DEG))
        # brain.screen.set_cursor(2, 1)
        # brain.screen.print("P:{0:.1f},D:{1:.1f}".format(error, Derror))
        # brain.screen.set_cursor(3,1)
        # brain.screen.print("I:{0:.1f}".format(Ierror))
        # brain.screen.set_cursor(4,1)
        # brain.screen.print("t:{0:.1f}".format(brain.timer.time(SECONDS)))
        wait(20)
    DriveLeft.stop()
    DriveRight.stop()
    brain.timer.clear()
    print("-=-=End Turn=-=-")


def user_control():
    return
    
def autonomous():
    calibrate_thingy()
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here
    # intake.spin(FORWARD)
    # intake.set_velocity(100, PERCENT)
    # wait(1, SECONDS)
    # intake.spin(FORWARD)
    # wait(1, SECONDS)
    # intake.stop   
    # PID_Drive(13.5, 0, 40)
    PID_Turn(-90)
    print("autoEnd")
    return   
    print("return")
    # PID_Drive(-24, -90, 25)
    # wait(0.5, SECONDS)
    # clamp.set(True)
    # PID_Turn(0)
    # PID_Drive(23.75, 0, 27)
    # PID_Turn(90)
    # PID_Drive(23, 90, 27)
    # PID_Turn(180)
    # PID_Drive(18.5, 180, 35)
    # wait(0.75, SECONDS)
    # PID_Drive(15.5, 180, 35)
    # PID_Turn(57.5)
    # PID_Drive(4.25, 57.5, 60)
    # wait(0.55, SECONDS)   
    # PID_Turn(-48.5)
    # PID_Drive(-6, -48.5, 30)
    # wait(0.15, SECONDS)
    # clamp.set(False)
    # wait(0.35, SECONDS)
    # PID_Drive(16, -48.5, 50)
    # PID_Turn(90)
    # PID_Drive(-67.5, 90, 55)
    # clamp.set(True)
    # wait(0.35, SECONDS)
    # PID_Turn(0)
    # PID_Drive(23, 0, 32)
    # PID_Turn(-90)
    # PID_Drive(24.85, -90, 32)
    # PID_Turn(-180)
    # PID_Drive(21.5, -180, 32)
    # wait(0.75, SECONDS) 
    # PID_Drive(10, -180, 35)
    # PID_Drive(-22.5, -180, 35)
    # PID_Turn(-145)
    # PID_Drive(7, -145, 37)
    # wait(0.5, SECONDS)
    # PID_Drive(-4, -145, 37)
    # PID_Turn(28)
    # PID_Drive(-28.5, 28, 25)
    # wait(0.15, SECONDS)
    # clamp.set(False)
    # wait(0.35, SECONDS)


# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts

motorInit()