import socket

import errno

from sys import exit
from weakref import ref
from queue import Queue

from threading import Thread, Event, Lock


class ParameterInfo:
    Instances = list()

    def __init__(self, decimal, address, note, method=None, numerical_value=-1):
        self.Decimal = decimal
        self.Address = address
        self.Note = note
        self.Method = method
        self.Value = numerical_value
        self.Instances.append(ref(self))

    def __repr__(self):
        return "Parameter {}: {}".format(self.Address, self.Value)

    @classmethod
    def getInstances(cls):
        # Give me a list of instances of this class for quick recovery. Lists give an ordered representation.
        # See https://effbot.org/pyfaq/how-do-i-get-a-list-of-all-instances-of-a-given-class.htm
        dead = list()
        for reference in cls.Instances:
            obj = reference()
            if obj is not None:
                yield obj
            else:
                dead.append(reference)
        for item in dead:
            cls.Instances.remove(item)


class Reader(socket.socket):
    def __init__(self, ip, port):
        super(Reader, self).__init__(socket.AF_INET, socket.SOCK_STREAM)
        self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.settimeout(1) # Timeout after one second
        self.Address = (ip, port)
        self.BufferLength = 1116
        self.ThreadLock = Lock()
        # Verify correct IP address is set, only seems to currently work with 192.168.111.6
        HOST_NAME = socket.gethostname()
        HOST_IP   = socket.gethostbyname(HOST_NAME)

        if HOST_NAME != 'MacBook-Pro-van-Simon.local' and HOST_IP != '192.168.111.6':
            # This is only a test. My personal computer was added to make testing easier.
            # Delete this condition when using the code but keep the ip check.
            raise ConnectionError("Verify IP of robotarm and cameras are on the same subnet.")

        # Disconnect to make sure the socket can connect:
        try:
            self.shutdownSafely(verbose=False)
        except OSError as e:
            if e.errno != errno.ENOTCONN:  # Catch OSError 57: socket not connected
                raise
        finally:
            self.connectSafely()

    def renewSocket(self):
        super(Reader, self).__init__(socket.AF_INET, socket.SOCK_STREAM)

    def connectSafely(self):
        print("{} connecting".format(self.Address))
        try:
            self.connect(self.Address)
            def sprint(*args, sep=" ", end="", **kwargs):
                joined_string = sep.join([str(arg) for arg in args])
                print(joined_string + "\n", sep=sep, end=end, **kwargs)
            sprint(self.Address, "is safely connected")
        except socket.timeout:
            self.shutdownSafely()
            raise ConnectionError('{} connection timed out.'.format(self.Address)) from None

    def shutdownSafely(self, verbose):
        raise NotImplementedError("shutdownSafely() method not implemented")


class ModBusReader(Reader):
    def __init__(self):
        IP = "192.168.1.17"
        PORT = 502

        self.ToolBitQueue = Queue()
        self.ToolPositionQueue = Queue()
        self.JointAngleQueue = Queue()

        self.ToolBitChanged = False
        self.SpikeOccurred = False
        self.ListOfCurrents = [0]*60
        # Read these parameters from the modbus:
        self.ToolBit       = ParameterInfo(1,   b'\x00\x01', "Vector of output bits. Only interested in number eight.", self.extractToolBit)
        self.ToolCurrent   = ParameterInfo(770, b'\x03\x02', "Current that is applied to the gripper.", self.extractToolCurrent)
        # Set the robot joint angles as attributes that are continuously updated:
        self.BaseAngle     = ParameterInfo(270, b'\x01\x0E', "Position (angle) of the base joint in milli rad.", self.extractAngle)
        self.ShoulderAngle = ParameterInfo(271, b'\x01\x0F', "Position (angle) of the shoulder joint in milli rad.", self.extractAngle)
        self.ElbowAngle    = ParameterInfo(272, b'\x01\x10', "Position (angle) of the elbow joint in milli rad.", self.extractAngle)
        self.Wrist1Angle   = ParameterInfo(273, b'\x01\x11', "Position (angle) of the wrist1 joint in milli rad.", self.extractAngle)
        self.Wrist2Angle   = ParameterInfo(274, b'\x01\x12', "Position (angle) of the wrist2 joint in milli rad.", self.extractAngle)
        self.Wrist3Angle   = ParameterInfo(275, b'\x01\x13', "Position (angle) of the wrist3 joint in milli rad.", self.extractAngle)
        # Positions only seem to start from the 588th byte
        self.toolX         = ParameterInfo(400, b'\x01\x90', "Cartesian Tool Coordinate X in tenth of mm from the base frame.", self.extractToolInfo)
        self.toolY         = ParameterInfo(401, b'\x01\x91', "Cartesian Tool Coordinate Y in tenth of mm from the base frame.", self.extractToolInfo)
        self.toolZ         = ParameterInfo(402, b'\x01\x92', "Cartesian Tool Coordinate Z in tenth of mm from the base frame.", self.extractToolInfo)
        self.toolRX        = ParameterInfo(403, b'\x01\x93', "Cartesian Tool Orientation RX in tenth of mm from the base frame.", self.extractAngle)
        self.toolRY        = ParameterInfo(404, b'\x01\x94', "Cartesian Tool Orientation RY in tenth of mm from the base frame.", self.extractAngle)
        self.toolRZ        = ParameterInfo(405, b'\x01\x95', "Cartesian Tool Orientation RZ in tenth of mm from the base frame.", self.extractAngle)

        self.StopCommunicatingEvent = Event()
        self.CommunicationThread = Thread(target=self.readContinuously, args=[self.ToolBitQueue, self.ToolPositionQueue, self.JointAngleQueue, self.StopCommunicatingEvent], daemon=True, name='ModBusReaderThread')

        # Startup parent after creation of all attributes:
        super(ModBusReader, self).__init__(IP, PORT)
        self.CommunicationThread.start()

    def readContinuously(self, tool_bit_queue, tool_position_queue, joint_angle_queue, stop_communicating_event):
        while not stop_communicating_event.is_set():
            self.read(tool_bit_queue, tool_position_queue, joint_angle_queue)

    @staticmethod
    def clearQueue(queue):
        while not queue.empty():
            queue.get()

    def storeSafelyInQueue(self, value, queue):
        if value is not None:
            with self.ThreadLock:
                self.clearQueue(queue)
                queue.put(value)

    def extractToolBit(self, data):
        allBits = [int(x) for x in bin(int(data))[2:]][:9:-1]
        gripperBit = 8
        ToolBitValue = allBits[gripperBit]
        if self.ToolBit.Value != ToolBitValue:
            self.ToolBitChanged = True
        return ToolBitValue

    def extractToolCurrent(self, data, tool_bit_queue):
        StabilisedCurrent = False
        self.ListOfCurrents.append(int(data[-2:], 16))
        self.ListOfCurrents.pop(0)
        DifferenceInCurrent = 0
        MaximumStationaryDifference = 4
        MaximumSpikeDifference = 10
        if self.ToolBitChanged:
            DifferenceInCurrent = sum([abs(i - j) for i, j in zip(self.ListOfCurrents[:-1], self.ListOfCurrents[1:])])
            if not self.SpikeOccurred:
                self.SpikeOccurred = DifferenceInCurrent > MaximumSpikeDifference
        if self.SpikeOccurred:
            if not StabilisedCurrent:
                StabilisedCurrent = DifferenceInCurrent < MaximumStationaryDifference
        if StabilisedCurrent:
            self.SpikeOccurred = False
            self.ToolBitChanged = False
        # Encode output as: as long as ToolBitChanged == True we haven't settled the current yet!
        # So as long as the output is False we haven't settled.
        self.storeSafelyInQueue((self.ToolBit.Value, not self.ToolBitChanged), tool_bit_queue)
        return not self.ToolBitChanged

    @staticmethod
    def extractAngle(data):
        value = int(data[-4:], 16)
        if value > 6284:  # If integer value > 6284 we need to take the complement
            value = -(65535 + 1 - value)
        return value * 1.0e-3

    @staticmethod
    def extractToolInfo(data):
        value = int(data[-4:], 16)
        if value > 32768:  # If integer value > 32767 we need to take the complement
            value = -(65535 + 1 - value)
        return value * 1.0e-4

    def read(self, tool_bit_queue, tool_position_queue, joint_angle_queue):
        parameters = list(ParameterInfo.getInstances())
        for index, parameter in enumerate(parameters):
            self.send(b'\x00\x04\x00\x00\x00\x06\x00\x03' + parameter.Address + b'\x00\x01')
            data = self.recv(self.BufferLength).hex()
            if len(data) == 0:
                continue
            if callable(parameter.Method):  # Call custom methods
                if parameter.Decimal == 770 and parameter.Address == b'\x03\x02':
                    # Special treatment: add the tool_bit_queue as a Queue for safe storage
                    parameter.Value = parameter.Method(data, tool_bit_queue)
                else:
                    parameter.Value = parameter.Method(data)
        parameterValues = [parameter.Value for parameter in parameters]
        self.storeSafelyInQueue(parameterValues[2:8], joint_angle_queue)  # All the robot joint angles
        self.storeSafelyInQueue(parameterValues[8:], tool_position_queue)   # All the tool info

    def getToolBitInfo(self):
        return self.ToolBitQueue.get()

    def getToolPosition(self):
        return self.ToolPositionQueue.get()

    def getJointAngles(self):
        return self.JointAngleQueue.get()

    def shutdownSafely(self, verbose=True):
        if verbose:
            print(self.Address, "shutting down safely.")
        if not self.StopCommunicatingEvent.isSet():
            self.StopCommunicatingEvent.set()
        if self.CommunicationThread.is_alive():
            self.CommunicationThread.join()
            self.StopCommunicatingEvent.clear()
        if not self._closed:  # Use private methods from the socket to test if it's alive
            self.shutdown(socket.SHUT_RDWR)
            self.close()


class RobotCCO(Reader):  # RobotChiefCommunicationOfficer
    def __init__(self):
        IP = "192.168.1.17"
        PORT = 30003
        super(RobotCCO, self).__init__(IP, PORT)

    def shutdownSafely(self, verbose=True):
        if verbose:
            print(self.Address, "shutting down safely.")
        if not self._closed:  # Use private methods from the socket to test if it's alive
            self.shutdown(socket.SHUT_RDWR)
            self.close()


if __name__ == '__main__':
    ModBusReader()
    RobotCCO()
