import binascii
import socket
import time
import threading
import queue
import struct
import numpy as np

from Kinematics import ForwardKinematics


class StaticRobotParameters:
    def __init__(self):
        # Socket parameters
        self.IP   = "192.168.1.17"
        self.Port = 30003

        # Positional arguments:
        self.jointAngleInit       = [item * np.pi / 180 for item in [61.42, -93.0, 94.65, -91.59, -90.0, 0.0]]
        self.jointAngleBrickReady = [item * np.pi / 180 for item in [6.49, -88.63, 90.35, -91.73, -89.91, 0.0]]
        self.jointAngleBrickUp    = [item * np.pi / 180 for item in [0.0, -90.0, 90.0, -90.0, 90.0, 180.0]]
        self.jointAnglePickupReady = [item * np.pi / 180 for item in [0.0, -90.0, 90.0, -90.0, 90.0, 180.0]]

        # Tolerances:
        self.eachPositionTolerance  = 5e-3
        self.totalPositionTolerance = 5e-3


class DynamicParameter(object):
    # Subclass floats for custom bookkeeping of the number of bytes, used by Universal Robots, in the format of a mutable number. See
    # https://stackoverflow.com/questions/35943789/python-can-a-subclass-of-float-take-extra-arguments-in-its-constructor
    # https://realpython.com/operator-function-overloading/
    def __init__(self, num_bytes, num_preceding_bytes, numeric_type, note, numerical_value=0):
        # super(DynamicParameter, self).__init__(num_bytes, num_preceding_bytes, type, note)
        # super(DynamicParameter, self).__init__()
        # object.__init__(num_bytes, num_preceding_bytes, type, note, numerical_value=numerical_value)
        self.sizeInBytes = num_bytes
        self.precedingBytes = num_preceding_bytes
        self.type = numeric_type
        self.note = note
        self.value = numerical_value


class DynamicRobotParameters:
    def __init__(self):
        # These parameters correspond to the 'Meaning'field in the UR excel sheet 'Client_Interface_V3'.
        self.MessageSize        = DynamicParameter(4,   0, '!i', "Total message length in bytes")
        self.Time               = DynamicParameter(8,   4, '!d', "Time elapsed since the controller was started")

        # Set the robot joint angles as attributes that are continuously updated:
        self.Base           = DynamicParameter(8, 252, '!d', "Position (angle) of the base joint")
        self.Shoulder       = DynamicParameter(8, 260, '!d', "Position (angle) of the shoulder joint")
        self.Elbow          = DynamicParameter(8, 268, '!d', "Position (angle) of the elbow joint")
        self.ElbowEnd       = DynamicParameter(8, 268, '!d', "Position (angle) of the elbow joint")
        self.Wrist1         = DynamicParameter(8, 276, '!d', "Position (angle) of the wrist2 joint")
        self.Wrist2         = DynamicParameter(8, 284, '!d', "Position (angle) of the wrist3 joint")
        self.Wrist3         = DynamicParameter(8, 292, '!d', "Position (angle) of the wrist3 joint")

        # Other dynamic parameters to read from the robot message:
        self.RobotMode          = DynamicParameter(8, 756,  '!d', "Current state of the digital inputs. These are bits encoded as int64_t: a value of 5 corresponds to bit 0 and bit 2 set high")
        self.DigitalOutputs     = DynamicParameter(8, 1044, '!d', "Digital outputs")

        # Positions only seem to start from the 588th byte
        self.toolX                  = DynamicParameter(8, 588, '!d', "Cartesian Tool Coordinate X")
        self.toolY                  = DynamicParameter(8, 596, '!d', "Cartesian Tool Coordinate Y")
        self.toolZ                  = DynamicParameter(8, 604, '!d', "Cartesian Tool Coordinate Z")
        self.toolRX                 = DynamicParameter(8, 612, '!d', "Cartesian Tool Orientation RX")
        self.toolRY                 = DynamicParameter(8, 620, '!d', "Cartesian Tool Orientation RY")
        self.toolRZ                 = DynamicParameter(8, 628, '!d', "Cartesian Tool Orientation RZ")


class RobotSocket(socket.socket):
    # This class is the lowest level of the socket interface and stores all socket info commands
    # To catch all information from the robot, we need a buffer length of 1116, due to RealTime 3.10 -> 3.13.
    # This number was established empirically.
    def __init__(self, ip, port, data_callback):
        super(RobotSocket, self).__init__(socket.AF_INET, socket.SOCK_STREAM)
        self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.settimeout(5)
        self.ip = ip
        self.port = port
        self.address = (ip, port)
        self.buffer_length = 1116
        # Make a callback function for the parent class to fetch the robot data
        self.sendInterpretedRobotData = data_callback
        # Parameters for continuous extraction of data:
        self.connectionThread = None
        self.connectionQueue = queue.Queue()
        self.connectionQueue.put(0)
        self.isConnected = False
        self.shuttingDown = False
        self.connectSafely()

    def shutdownRobot(self):
        print("Robot is shutting down.")
        self.shuttingDown = True
        self.close()

    def renewSocket(self):
        super(RobotSocket, self).__init__(socket.AF_INET, socket.SOCK_STREAM)

    def interpretAllRobotData(self, data):
        # Interpret the dynamic parameters from 'Client_Interface_V3' using https://docs.python.org/3/library/struct.html
        # The dynamic parameters declared in the robot will be automatically updated, since this function is called in a thread,
        # so that the parameter update can occur fully concurrently.
        # Additionally, a position update for all the robot joints is necessary.
        interpretedData = DynamicRobotParameters()
        for attribute in vars(interpretedData):
            parameter = getattr(interpretedData, attribute)
            value = data[parameter.precedingBytes:(parameter.precedingBytes + parameter.sizeInBytes)]
            if len(value) == 0:
                return None
            try:
                value = struct.unpack(parameter.type, bytes.fromhex(str(binascii.hexlify(value).decode("utf-8"))))[0]
            except struct.error as e:
                print("An exception occurred on {}. {}".format(attribute, e))
            if attribute == 'MessageSize' and value == 0:
                return None
            parameter.value = value
            setattr(parameter, 'value', value)
        return interpretedData

    def connectSafely(self, address_to_connect_to=None):
        try:
            address = address_to_connect_to if address_to_connect_to is not None else self.address
            self.connect(self.address) if not address else self.connect(address)
            self.connectionThread = threading.Thread(target=self.communicate, args=(self.connectionQueue, address,), daemon=True)
            self.connectionThread.start()
            print("The robot is connected.")
        except socket.timeout:
            print("The robot connection timed out.")

    def communicate(self, communication_queue, address):
        while not self.shuttingDown:
            try:
                test = communication_queue.get()  # Verify queue is empty
                data = self.recv(self.buffer_length)
                data = self.interpretAllRobotData(data)
                communication_queue.put(data)
                self.sendInterpretedRobotData(communication_queue)

            except OSError as error:
                # set connection status and recreate socket
                self.isConnected = False
                startTimeNotConnected = time.time()
                print("Robot connection lost ...")
                while not self.isConnected:
                    elapsedTimeNotConnected = time.time() - startTimeNotConnected
                    if elapsedTimeNotConnected > 5:
                        return
                    try:
                        self.renewSocket()
                        self.connect(address)
                        self.isConnected = True
                        print("Server connected again")
                    except OSError as error:
                        time.sleep(0.2)


class Robot(StaticRobotParameters, DynamicRobotParameters, RobotSocket):
    def __init__(self):
        StaticRobotParameters.__init__(self)
        DynamicRobotParameters.__init__(self)
        RobotSocket.__init__(self, self.IP, self.Port, self.updateDynamicRobotParameters)
        self.robotJointAngles = []
        self.toolPosition = []
        # # Wait for update thread to kick in
        # while len(self.robotJoints) == 0:
        #     time.sleep(0.1)

    def updateDynamicRobotParameters(self, communication_queue):
        # Update parameters of the Robot through a thread callback from the socket.
        # Loop through the new attributes and find the old attributes to be replaced.
        new_parameters = communication_queue.get()
        # print("Got parameters", new_parameters)
        if new_parameters:
            for parameter in vars(new_parameters):
                newParameter = getattr(new_parameters, parameter)
                setattr(self, parameter, newParameter)
        communication_queue.put(0)
        self.robotJointAngles = [joint.value for joint in [self.Base, self.Shoulder, self.Elbow, self.Wrist1, self.Wrist2, self.Wrist3]]
        self.toolPosition = [i.value for i in [self.toolX, self.toolY, self.toolZ, self.toolRX, self.toolRY, self.toolRZ]]

    def jointPositions(self):
        return ForwardKinematics(self.robotJointAngles)

    def openGripper(self):
        self.send(b'set_digital_out(8, False)' + b"\n")

    def closeGripper(self):
        self.send(b'set_digital_out(8, True)' + b"\n")

    def setGripperVoltage(self, voltage=24):
        self.send(b'set_tool_voltage(' + bytes(voltage) + b')' + b'\n')

    def moveJointsToAngle(self, move, position, wait=True, p=True):
        """
        DESCRIPTION: Moves the robot with the movej or movel command to pos
        :param move: movej or movel
        :param position: joint angles
        :param wait: If true the robot program, waits until the robot is in its position.
        :param p: Defines weather the position is a position or a joint position.
        """
        p = "p" if p is True else ""
        command = "{}({}{}) \n".format(move, p, position)
        print(command)

        # Send command
        self.send(str.encode(command))
        # Wait for the robot arm to reach the position
        if wait:
            self.waitForJointAnglesToBeReached(position)

    def waitForJointAnglesToBeReached(self, position):
        difference = [1000.0 for _ in position]
        # while all(d > self.eachPositionTolerance for d in difference) and \
        while sum(difference) >= self.totalPositionTolerance:
            difference = [np.abs(joint.value - pos) for joint, pos in zip(self.robotJoints, position)]
            time.sleep(0.001)

    def waitForParallelTask(self, function, arguments=None):
        thread = threading.Thread(target=function, args=[], daemon=True)
        thread.start()
        thread.join()

    def initialise(self):
        def initialiseInThread():
            self.openGripper()
            # time.sleep(0.5)
            self.closeGripper()
            time.sleep(0.5)
            self.moveJointsToAngle("movej", self.jointAngleInit, wait=True, p=False)
            # self.moveJointsToAngle("movej", self.jointAngleBrickReady, wait=True, p=False)
            # self.moveJointsToAngle("movej", self.jointAngleBrickUp, wait=True, p=False)
            # self.moveJointsToAngle("movej", self.jointAngleBrickReady, wait=True, p=False)
            # self.moveJointsToAngle("movej", self.jointAngleInit, wait=True, p=False)
            print("Initialisation Done")

        self.waitForParallelTask(function=initialiseInThread(), arguments=None)

    def pickUpObject(self):
        def initialiseInThread():
            self.openGripper()
            # time.sleep(0.5)
            self.closeGripper()
            time.sleep(0.5)
            self.moveJointsToAngle("movej", self.jointAngleInit, wait=True, p=False)
            # self.moveJointsToAngle("movej", self.jointAngleBrickReady, wait=True, p=False)
            # self.moveJointsToAngle("movej", self.jointAngleBrickUp, wait=True, p=False)
            # self.moveJointsToAngle("movej", self.jointAngleBrickReady, wait=True, p=False)
            # self.moveJointsToAngle("movej", self.jointAngleInit, wait=True, p=False)
            print("Initialisation Done")

        self.waitForParallelTask(function=initialiseInThread(), arguments=None)


if __name__ == '__main__':
    robot = Robot()
    robot.closeGripper()
    time.sleep(2)

    robot.openGripper()
    time.sleep(10)
    robot.shutdownRobot()



