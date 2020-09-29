import socket
import weakref


class ParameterInfo:
    Instances = list()

    def __init__(self, num_bytes, num_preceding_bytes, numeric_type, note, numerical_value=0):
        self.SizeInBytes = num_bytes
        self.PrecedingBytes = num_preceding_bytes
        self.Type = numeric_type
        self.Note = note
        self.Value = numerical_value
        self.Instances.append(weakref.ref(self))

    @classmethod
    def getInstances(cls):
        # Give me a list of instances of this class for quick recovery. Lists give an ordered representation.
        # See https://effbot.org/pyfaq/how-do-i-get-a-list-of-all-instances-of-a-given-class.htm
        dead = list()
        for ref in cls.Instances:
            obj = ref()
            if obj is not None:
                yield obj
            else:
                dead.append(ref)
        for item in dead:
            cls.Instances.remove(item)


class Reader(socket.socket):
    def __init__(self, ip, port):
        super(Reader, self).__init__(socket.AF_INET, socket.SOCK_STREAM)
        self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.settimeout(5)
        self.Address = (ip, port)
        self.BufferLength = 1116

    def read(self):
        raise NotImplementedError

    def renewSocket(self):
        super(Reader, self).__init__(socket.AF_INET, socket.SOCK_STREAM)

    def connectSafely(self):
        try:
            self.connect(self.Address)
            print(__class__.__name__, "is safely connected.")
        except socket.timeout:
            print(__class__.__name__, "connection timed out.")

    def shutdownSafely(self):
        print(__class__.__name__, "shutting down safely.")
        self.shutdown(socket.SHUT_RDWR)
        self.close()


class ModBusReader(Reader):
    def __init__(self):
        IP = "192.168.111.2"
        PORT = 502
        super(ModBusReader, self).__init__(IP, PORT)

    def read(self):
        data = self.recv(self.BufferLength)


class RobotInfoReader(Reader):
    def __init__(self):
        IP = "192.168.1.17"
        PORT = 30003
        super(RobotInfoReader, self).__init__(IP, PORT)

        # These parameters correspond to the 'Meaning'field in the UR excel sheet 'Client_Interface_V3'.
        self.MessageSize = ParameterInfo(4, 0, '!i', "Total message length in bytes")
        self.Time        = ParameterInfo(8, 4, '!d', "Time elapsed since the controller was started")

        # Set the robot joint angles as attributes that are continuously updated:
        self.BaseAngle     = ParameterInfo(8, 252, '!d', "Position (angle) of the base joint")
        self.ShoulderAngle = ParameterInfo(8, 260, '!d', "Position (angle) of the shoulder joint")
        self.ElbowAngle    = ParameterInfo(8, 268, '!d', "Position (angle) of the elbow joint")
        self.Wrist1Angle   = ParameterInfo(8, 276, '!d', "Position (angle) of the wrist1 joint")
        self.Wrist2Angle   = ParameterInfo(8, 284, '!d', "Position (angle) of the wrist2 joint")
        self.Wrist3Angle   = ParameterInfo(8, 292, '!d', "Position (angle) of the wrist3 joint")

        # Positions only seem to start from the 588th byte
        self.toolX  = ParameterInfo(8, 588, '!d', "Cartesian Tool Coordinate X")
        self.toolY  = ParameterInfo(8, 596, '!d', "Cartesian Tool Coordinate Y")
        self.toolZ  = ParameterInfo(8, 604, '!d', "Cartesian Tool Coordinate Z")
        self.toolRX = ParameterInfo(8, 612, '!d', "Cartesian Tool Orientation RX")
        self.toolRY = ParameterInfo(8, 620, '!d', "Cartesian Tool Orientation RY")
        self.toolRZ = ParameterInfo(8, 628, '!d', "Cartesian Tool Orientation RZ")

    def read(self):
        variables = vars(self)

        for obj in ParameterInfo.getInstances():
            print(obj.Note)
        # data = self.recv(self.BufferLength)
        # data = self.interpretData(data)


class Robot:
    def __init__(self):
        super(Robot, self).__init__()
        self.robotInfoReader = RobotInfoReader()
        self.robotInfoReader.read()


if __name__ == '__main__':
    robot = Robot()
