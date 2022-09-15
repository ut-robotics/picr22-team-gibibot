import motion
robot = motion.OmniRobotMotion()
def find_ball(rot, ball=0):
    if (ball == 0):
        robot.sendinfo(0,rot,0,0,0)
        robot.getinfo()
        return 0
    else:
        return 1