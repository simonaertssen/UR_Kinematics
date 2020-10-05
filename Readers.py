import time
import socket
import struct
import threading

from sys import exit
from weakref import ref
from struct import unpack
from queue import Queue


class ParameterInfo:
    Instances = list()

    def __init__(self, name, num_bytes, num_preceding_bytes, numeric_type, note, numerical_value=0):
        self.Name = name
        self.SizeInBytes = num_bytes
        self.PrecedingBytes = num_preceding_bytes
        self.Type = numeric_type
        self.Note = note
        self.Value = numerical_value
        self.Instances.append(ref(self))

    def __repr__(self):
        return "{}: {}".format(self.Name, self.Value)

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
    def __init__(self, ip, port, callback=None):
        super(Reader, self).__init__(socket.AF_INET, socket.SOCK_STREAM)
        self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.TimeOut = 3
        self.settimeout(self.TimeOut)
        self.Address = (ip, port)
        self.BufferLength = 1116
        self.Callback = callback
        self.connectSafely()

        self.ToolInfoQueue = Queue()
        self.ThreadLock = threading.Lock()
        self.CommunicationThread = threading.Thread(target=self.readContinuously, args=(), daemon=True)
        self.CommunicationThread.start()

    def renewSocket(self):
        super(Reader, self).__init__(socket.AF_INET, socket.SOCK_STREAM)

    def readContinuously(self):
        while True:
            self.read()

    def read(self):
        raise NotImplementedError

    def getToolInfo(self):
        return self.ToolInfoQueue.get()

    @staticmethod
    def clearQueue(queue):
        while not queue.empty():
            queue.get()

    def connectSafely(self):
        try:
            self.connect(self.Address)
            print(self.Address, "is safely connected.")
        except socket.timeout:
            self.close()
            exit('{} connection timed out.'.format(self.Address))

    def shutdownSafely(self):
        print(self.Address, "shutting down safely.")
        self.shutdown(socket.SHUT_RDWR)
        self.close()


class ModBusReader(Reader):
    def __init__(self, callback=None):
        IP = "192.168.1.17"
        PORT = 502
        super(ModBusReader, self).__init__(IP, PORT, callback)
        self.JointAngleQueue = Queue()
        self.ToolBit = 0
        self.ToolBitChanged = False
        self.SpikeOccurred = False
        self.ListOfCurrents = [0]*200

    def read(self):
        StabilisedCurrent = False

        # Request to read info from register 1, the output bits
        data = b''
        while len(data) == 0:
            self.send(b'\x00\x04\x00\x00\x00\x06\x00\x03\x00\x01\x00\x01')
            data = self.recv(self.BufferLength).hex()

        allBits = [int(x) for x in bin(int(data))[2:]][::-1]
        gripperBit = 8
        ToolBitValue = allBits[gripperBit]
        if self.ToolBit != ToolBitValue:
            self.ToolBitChanged = True
            self.ToolBit = ToolBitValue

        # Request to read info from register 770, the tool current (last two digits in the buffer).
        data = b''
        while len(data) == 0:
            self.send(b'\x00\x04\x00\x00\x00\x06\x00\x03\x03\x02\x00\x01')
            data = self.recv(self.BufferLength).hex()
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
        output = self.ToolBit, not self.ToolBitChanged
        if output is not None:
            with self.ThreadLock:
                self.clearQueue(self.ToolInfoQueue)
                self.ToolInfoQueue.put(output)


class RobotChiefCommunicationOfficer(Reader):
    def __init__(self, callback=None):
        IP = "192.168.1.17"
        PORT = 30003
        super(RobotChiefCommunicationOfficer, self).__init__(IP, PORT, callback)
        self.JointAngleQueue = Queue()
        # These parameters correspond to the 'Meaning'field in the UR excel sheet 'Client_Interface_V3'.
        self.MessageSize = ParameterInfo('MessageSize', 4, 0, '!i', "Total message length in bytes")
        self.Time        = ParameterInfo('Time', 8, 4, '!d', "Time elapsed since the controller was started")
        # Set the robot joint angles as attributes that are continuously updated:
        self.BaseAngle     = ParameterInfo('BaseAngle', 8, 252, '!d', "Position (angle) of the base joint")
        self.ShoulderAngle = ParameterInfo('ShoulderAngle', 8, 260, '!d', "Position (angle) of the shoulder joint")
        self.ElbowAngle    = ParameterInfo('ElbowAngle', 8, 268, '!d', "Position (angle) of the elbow joint")
        self.Wrist1Angle   = ParameterInfo('Wrist1Angle', 8, 276, '!d', "Position (angle) of the wrist1 joint")
        self.Wrist2Angle   = ParameterInfo('Wrist2Angle', 8, 284, '!d', "Position (angle) of the wrist2 joint")
        self.Wrist3Angle   = ParameterInfo('Wrist3Angle', 8, 292, '!d', "Position (angle) of the wrist3 joint")
        # Positions only seem to start from the 588th byte
        self.toolX  = ParameterInfo('toolX', 8, 588, '!d', "Cartesian Tool Coordinate X")
        self.toolY  = ParameterInfo('toolY', 8, 596, '!d', "Cartesian Tool Coordinate Y")
        self.toolZ  = ParameterInfo('toolZ', 8, 604, '!d', "Cartesian Tool Coordinate Z")
        self.toolRX = ParameterInfo('toolRX', 8, 612, '!d', "Cartesian Tool Orientation RX")
        self.toolRY = ParameterInfo('toolRY', 8, 620, '!d', "Cartesian Tool Orientation RY")
        self.toolRZ = ParameterInfo('toolRZ', 8, 628, '!d', "Cartesian Tool Orientation RZ")

    def getJointInfo(self):
        return self.JointAngleQueue.get()

    def read(self):
        data = b''
        while len(data) == 0:
            try:
                data = self.recv(self.BufferLength)
            except OSError as error:
                print('An error occurred while reading the robot info...\n', error)
                return None

        parameters = list(ParameterInfo.getInstances())
        for index, parameter in enumerate(parameters):
            value = data[parameter.PrecedingBytes:(parameter.PrecedingBytes + parameter.SizeInBytes)]
            if len(value) == 0:
                return None
            try:
                value = unpack(parameter.Type, bytes.fromhex(value.hex()))[0]
            except struct.error as e:
                print('An error occurred while reading the robot info...\n', e)
            if index == 0 and (value == 0 or value > self.BufferLength):  # Catch when the message is empty
                return None
            parameter.Value = value

            if parameter.Name == 'Wrist3Angle':  # Then all angles have been read
                output = self.BaseAngle, self.ShoulderAngle, self.ElbowAngle, self.Wrist1Angle, self.Wrist2Angle, self.Wrist3Angle
                if output is not None:
                    with self.ThreadLock:
                        self.clearQueue(self.ToolInfoQueue)
                        self.ToolInfoQueue.put(output)

            if parameter.Name == 'toolRZ':  # Then all tool parameters have been read
                output = self.toolX, self.toolY, self.toolZ, self.toolRX, self.toolRY, self.toolRZ
                if output is not None:
                    with self.ThreadLock:
                        self.clearQueue(self.JointAngleQueue)
                        self.JointAngleQueue.put(output)


