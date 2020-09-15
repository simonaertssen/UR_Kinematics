import binascii
import socket
import time
import threading
import queue
import struct
import numpy as np


class StaticRobotParameters:
    def __init__(self):
        # Socket parameters
        self.Name = "RobotSocket"
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

    # def make(self, value):
    #     return self.__class__(self.sizeInBytes, self.precedingBytes, self.type, self.note, value)
    #
    # def __index__(self):
    #     return self._value
    #
    # def __int__(self):
    #     return int(self._value)
    #
    # def __float__(self):
    #     return float(self._value)
    #
    # def __repr__(self):
    #     return f"{self.__class__.__name__}({self._value})"
    #
    # def __add__(self, other):
    #     return self.make(self._value + other)
    #
    # def __sub__(self, other):
    #     return self.make(self._value - other)
    #
    # def __mul__(self, other):
    #     return self.make(self._value * other)
    #
    # def __truediv__(self, other):
    #     if other == 0 or other == 0.0:
    #         raise ZeroDivisionError
    #     else:
    #         return self.make(self._value / other)
    #
    # def __radd__(self, other):
    #     return self.__add__(other)
    #
    # def __rsub__(self, other):
    #     return self.make(other - self._value)
    #
    # def __rmul__(self, other):
    #     return self.__mul__(other)
    #
    # def __rtruediv__(self, other):
    #     if self._value == 0 or self._value == 0.0:
    #         raise ZeroDivisionError
    #     else:
    #         return self.make(other / self._value)
    #
    # def __iadd__(self, other):
    #     return self.__add__(other)
    #
    # def __isub__(self, other):
    #     return self.__sub__(other)
    #
    # def __imul__(self, other):
    #     return self.__mul__(other)
    #
    # def __idiv__(self, other):
    #     return self.__truediv__(other)
    #
    # def __neg__(self):
    #     return self.make(-self._value)
    #
    # def __abs__(self):
    #     return self.make(self._value) if self._value > 0 else self.make(-self._value)
    #
    # def set(self, new_value):
    #     self._value = new_value
    #
    # def sin(self):
    #     return np.sin(self._value)
    #
    # def cos(self):
    #     return np.cos(self._value)
    #
    # def test(self):
    #     print(type(self), "Value =", self)
    #     print(self + 10, self + 10.0, 10 + self, 10.0 + self)
    #     print(self - 10, self - 10.0, 10 - self, 10.0 - self)
    #     print(self * 10, self * 10.0, 10 * self, 10.0 * self)
    #     print(self / 10, self / 10.0, 10 / self, 10.0 / self)


class RobotJoint(DynamicParameter):
    # The robot joint is a dynamic parameter extended with X, Y, and Z coordinates and the
    # Devanit-Hartenberg parameters for computation of the robot position at each joint.
    # See: https://blog.robotiq.com/how-to-calculate-a-robots-forward-kinematics-in-5-easy-steps
    # --- Parameters:
    # origin: the previous joint
    # self/theta: the angle around the z-axis between the previous x-axis and the current x-axis.
    # a: the length of the common normal, which is the distance between the previous z-axis and the current z-axis.
    # d:  the distance between the previous x-axis and the current x-axis, along the previous z-axis.
    # alpha: the angle around the common normal to between the previous z-axis and current z-axis.
    # Following parameters are for the DynamicParameter parent class
    def __init__(self, origin, a, d, alpha, num_bytes, num_preceding_bytes, numeric_type, note, numerical_value=0):
        super(RobotJoint, self).__init__(num_bytes, num_preceding_bytes, numeric_type, note, numerical_value)
        # self.angle = DynamicParameter(num_bytes, num_preceding_bytes, numeric_type, note)
        self.origin = origin
        self.a = a
        self.d = d
        self.alpha = alpha
        self.switchAxes = False
        self.T = self.makeT()

    # # Override the 'make'method as this class needs some more parameters
    # def make(self, value):
    #     return self.__class__(self.origin, self.a, self.d, self.alpha, self.sizeInBytes, self.precedingBytes, self.type, self.note, value)

    # Override the representation as we want angles in degrees
    def __repr__(self):
        # return f"{self.__class__.__name__}({self.value * 180 / np.pi})"
        return f"{self.__class__.__name__}({self.value})"

    # def setRotationMatrix(self):
    #     # This function takes into account the permutation of the axis, so it multiplies a permutation matrix with the rotation matrix
    #     rotationAxis = [element for element, index in enumerate(self.orientation) if element == index]
    #     if len(rotationAxis) == 1:
    #         otherAxes = self.orientation.copy()
    #         rotationAxis = rotationAxis[0]
    #         otherAxes.remove(rotationAxis)
    #         if rotationAxis != 2:
    #             # Swap some columns and rows to make a rotation about the correct axis
    #             self.rotationMatrix[:, [rotationAxis, 2]] = self.rotationMatrix[:, [2, rotationAxis]]
    #             self.rotationMatrix[[rotationAxis, 2], :] = self.rotationMatrix[[2, rotationAxis], :]
    #         self.permutationMatrix[:, [otherAxes[0], otherAxes[1]]] = self.permutationMatrix[:, [otherAxes[1], otherAxes[0]]]
    #     elif rotationAxis == self.orientation:
    #         self.rotationMatrix = np.identity(3)
    #     else:
    #         self.permutationMatrix[:, [self.orientation[0], self.orientation[1]]] = self.permutationMatrix[:, [self.orientation[1], self.orientation[0]]]
    #         self.permutationMatrix[:, [self.orientation[1], self.orientation[2]]] = self.permutationMatrix[:, [self.orientation[2], self.orientation[1]]]
    #         self.rotationMatrix = self.permutationMatrix
    #     if self.permutationMatrix.sum() > 3:
    #         raise ValueError
    #     # self.rotationMatrix = self.rotationMatrix.dot(self.permutationMatrix)

    # Use functions to determine the position of the joints, as 'self' is the dynamic parameter value.
    def X(self):
        return self.a * np.cos(self.value)

    def Y(self):
        return self.a * np.sin(self.value)

    def Z(self):
        return self.d

    def XYZ(self):
        return np.array([self.X(), self.Y(), self.Z()])

    # def Told(self):
    #     theta = self._value
    #     cost = np.cos(theta)
    #     sint = np.sin(theta)
    #     cosa = np.cos(self.alpha)
    #     sina = np.sin(self.alpha)
    #     # T = np.array([[cost, -sint * cosa, sint * sina, self.X],
    #     #               [sint, cost * cosa, -cost * sina, self.Y],
    #     #               [0, sina, cosa, self.Z],
    #     #               [0, 0, 0, 1]])
    #     T = np.array([[cost, -sint*cosa,  sint*sina, self._X()],
    #                   [sint,  cost*cosa, -cost*sina, self._Y()],
    #                   [0,     sina,       cosa,      self._Z()],
    #                   [0,     0,          0,              1]])
    #     return T

    # def updateRotation(self):
    #     rotationAngle = self.origin if self.origin is not None else np.pi/2
    #     c, s = np.cos(rotationAngle), np.sin(rotationAngle)
    #     self.rotationMatrix = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])

    # def updateTransformationT(self):
    #     X, Y, Z = self.pos()
    #     theta = self._value
    #     cos_t = np.cos(theta)
    #     sin_t = np.sin(theta)
    #     cos_a = np.cos(self.alpha)
    #     sin_a = np.sin(self.alpha)
    #     T = np.array([[cos_t, -sin_t*cos_a,  sin_t*sin_a, X],
    #                   [sin_t,  cos_t*cos_a, -cos_t*sin_a, Y],
    #                   [0,            sin_a,        cos_a, Z],
    #                   [0,                0,            0, 1]])
    #     return T

    def makeT(self):
        X, Y, Z = self.XYZ()
        # self.X, self.Y, self.Z = float(self.a * np.cos(self._value)), float(self.a * np.cos(self._value)), d
        # print(type(self))
        cos_t = np.cos(self.value)
        sin_t = np.sin(self.value)
        cos_a = np.cos(self.alpha)
        sin_a = np.sin(self.alpha)
        T = np.array([[cos_t, -sin_t*cos_a,  sin_t*sin_a, X],
                      [sin_t,  cos_t*cos_a, -cos_t*sin_a, Y],
                      [0,            sin_a,        cos_a, Z],
                      [0,                0,            0, 1]])
        return T

    def position(self):
        return np.array([self.T[0, 3], self.T[1, 3], self.T[2, 3]])

    def updateCoordinates(self):
        # Update the coordinates making use of the internal axis transformation.
        # First, register internal angle and translate to xyz in the rotor plane.
        # Use those values for the current T and the transformed coordinates of the origin to compute the new position.
        # Then adjust for axis transformation and register a new T for the next joint.
        # self.X, self.Y, self.Z = self._X(), self._Y(), self._Z()
        if self.origin is None:
            self.T = self.makeT()
        else:
            self.T = self.origin.T.dot(self.T)
            # self.X, self.Y, self.Z = position[0:3, 3]
        # self.X, self.Y, self.Z = self.rotationMatrix.dot(np.array([[self.X, self.Y, self.Z]]).T)
        # self.T = self.updateTransformationT()


class DynamicRobotParameters:
    def __init__(self):
        # These parameters correspond to the 'Meaning'field in the UR excel sheet 'Client_Interface_V3'.
        # The joint parameters a, d and alpha can be found here: https://www.universal-robots.com/articles/ur-articles/parameters-for-calculations-of-kinematics-and-dynamics/
        self.MessageSize        = DynamicParameter(4,   0, '!i', "Total message length in bytes")
        self.Time               = DynamicParameter(8,   4, '!d', "Time elapsed since the controller was started")

        # Set the Robot Joints as attributes that are continuously updated:
        self.Base           = RobotJoint(origin=None,          a=0.134,    d=0.089159, alpha=np.pi/2,  num_bytes=8, num_preceding_bytes=252, numeric_type='!d', note="Position (angle) of the base joint")
        self.Shoulder       = RobotJoint(origin=self.Base,     a=-0.425,   d=0,        alpha=0,        num_bytes=8, num_preceding_bytes=260, numeric_type='!d', note="Position (angle) of the shoulder joint")
        self.Elbow          = RobotJoint(origin=self.Shoulder, a=0,        d=0.119,    alpha=0,        num_bytes=8, num_preceding_bytes=268, numeric_type='!d', note="Position (angle) of the elbow joint")
        self.ElbowEnd       = RobotJoint(origin=self.Elbow,    a=-0.39225, d=0,        alpha=0,        num_bytes=8, num_preceding_bytes=268, numeric_type='!d', note="Position (angle) of the elbow joint")
        self.Wrist1         = RobotJoint(origin=self.ElbowEnd, a=0,        d=-0.09475, alpha=np.pi/2,  num_bytes=8, num_preceding_bytes=276, numeric_type='!d', note="Position (angle) of the wrist2 joint")
        self.Wrist2         = RobotJoint(origin=self.Wrist1,   a=0,        d=0.09475,  alpha=-np.pi/2, num_bytes=8, num_preceding_bytes=284, numeric_type='!d', note="Position (angle) of the wrist3 joint")
        self.Wrist3         = RobotJoint(origin=self.Wrist2,   a=0,        d=-0.0815,  alpha=0,        num_bytes=8, num_preceding_bytes=292, numeric_type='!d', note="Position (angle) of the wrist3 joint")

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
    def __init__(self, name, ip, port, data_callback):
        super(RobotSocket, self).__init__(socket.AF_INET, socket.SOCK_STREAM)
        self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.settimeout(5)
        self.name = name
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
        for index, attribute in enumerate(vars(interpretedData)):
            parameter = getattr(interpretedData, attribute)
            # if isinstance(parameter, RobotJoint):
            #     parameter = getattr(parameter, 'angle')
            value = data[parameter.precedingBytes:(parameter.precedingBytes + parameter.sizeInBytes)]
            if value == 0 or len(value) == 0:
                return None
            # print(attribute, "[{}:{}]".format(parameter.precedingBytes, parameter.precedingBytes + parameter.sizeInBytes), value, len(value))
            try:
                value = struct.unpack(parameter.type, bytes.fromhex(str(binascii.hexlify(value).decode("utf-8"))))[0]
            except struct.error as e:
                print("An exception occurred on {}. {}".format(attribute, e))

            if index == 0 and value == 0:
                return None

            # print(attribute, "[{}:{}]".format(parameter.precedingBytes, parameter.precedingBytes + parameter.sizeInBytes), value)
            # print(attribute, parameter, vars(parameter))
            parameter.value = value
            setattr(parameter, 'value', value)
            # print(parameter.value)
            #
            # if isinstance(parameter, RobotJoint):
            #     parameter.updateCoordinates()
            # print("interpretedData", vars(interpretedData.toolX))
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


class RobotClass(StaticRobotParameters, DynamicRobotParameters, RobotSocket):
    def __init__(self, display_dynamic_robot_parameters=True):
        StaticRobotParameters.__init__(self)
        DynamicRobotParameters.__init__(self)
        RobotSocket.__init__(self, self.Name, self.IP, self.Port, self.updateDynamicRobotParameters)
        self.displayDynamicRobotParameters = display_dynamic_robot_parameters
        self.robotJoints = []
        self.toolPosition = []
        # Wait for update thread to kick in
        while len(self.robotJoints) == 0:
            time.sleep(0.1)

    def updateDynamicRobotParameters(self, communication_queue):
        # Update parameters of the RobotClass through a thread callback from the socket.
        # Loop through the new attributes and find the old attributes to be replaced.
        new_parameters = communication_queue.get()
        # print("Got parameters", new_parameters)
        if new_parameters:
            for parameter in vars(new_parameters):
                newParameter = getattr(new_parameters, parameter)
                setattr(self, parameter, newParameter)
        communication_queue.put(0)
        self.robotJoints = [self.Base, self.Shoulder, self.Elbow, self.Wrist1, self.Wrist2, self.Wrist3]
        self.toolPosition = [self.toolX, self.toolY, self.toolZ, self.toolRX, self.toolRY, self.toolRZ]
        self.updateJointPositions()
        # print(self.toolX.value, self.toolY.value, self.toolZ.value)
        angles = [joint.value / 3.14159 * 180 for joint in self.robotJoints]

    def updateJointPositions(self):
        for joint in self.robotJoints:
            # Swap x and y coordinates of base as a 90 degree rotation was confirmed
            if joint.origin is None:
                joint.T[0, 3], joint.T[1, 3] = -joint.T[1, 3], joint.T[0, 3]
            else:
                joint.T = joint.origin.T.dot(joint.T)

    def jointAngles(self):
        # return np.array([self.Base, self.Shoulder, self.Elbow, self.ElbowEnd, self.Wrist1, self.Wrist2, self.Wrist3])
        return [joint.angle for joint in self.robotJoints]

    def jointPositions(self):
        X, Y, Z = [0, 0], [0, 0], [0, self.Base.Z()]
        for joint in self.robotJoints:
            x, y, z = joint.position()
            X.append(x)
            Y.append(y)
            Z.append(z)
        return np.array(X), np.array(Y), np.array(Z)

    def openGripper(self):
        # self.renewSocket()
        # self.connectSafely()
        self.send(b'set_digital_out(8, False)' + b"\n")

    def closeGripper(self):
        # self.renewSocket()
        # self.connectSafely()
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






