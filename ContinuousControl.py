
import time
import RobotSocket

# test = RobotSocket.DynamicParameter(8, 0, '!d', "Test", 48.2)
# test.test()
# test += 10.0
# test -= 10.0
# test.test()

Robot = RobotSocket.RobotClass()
# Robot.closeGripper()
time.sleep(20)
# Robot.openGripper()
# time.sleep(1)
Robot.shutdownRobot()

