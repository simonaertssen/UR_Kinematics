import socket
import errno

from weakref import ref
from threading import Thread, Event


class ParameterInfo(object):
    r"""
    Class used to represent a parameter of the UR5 robot, with all the necessary
    information to retrieve it from the UR5 modbus. This makes reading joint
    angles, positions, voltages and velocities easier.
    A list of instances is saved to quickly yield all instances of this class.

    Attributes:
    -------
    Instances : list
        The list of existing instances for quick retrieval.
    Decimal : int
        The decimal register address of the parameter. Added for reference to
        the documentation.
    Address : bytes
        The hexadecimal register address of the parameter in \x.. format.
        Required to request information from this address.
    Note : str
        The description of the parameter and it's unit.
    Method : function handle
        The method or function that should be applied on the parameter value.
    Value : float
        The umerical value of the parameter.

    Example:
    -------
    -> ToolCurrent = ParameterInfo(770, b'\x03\x02', "Tool current (mA).", None)
    The decimal address 770 is 302 in hexadecimal. This yields the tool current.
    No method to apply on this value.
    """

    # Declare __slots__ method for faster attribute access, add __weakref__
    __slots__ = ('__weakref__', 'Decimal', 'Address', 'Note', 'Method', 'Value')
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
        r"""
        Yield a generator of instances of this class for quick recovery. This is
        cleaner than to refer to all instances by name.

        Returns:
        -------
        A generator of existing instances.
        """
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
    r"""
    Class used to extend the python socket with custom methods and attributes.

    Attributes:
    -------
    Address : tuple
        The IP address and port we wich to connect to.
    BufferLength : int
        The length of the buffer when receiving messages from the socket.
    ThreadLock : Lock
        The threadlock used for atomic access of queues of the child classes.
    """

    def __init__(self, ip, port):
        super(Reader, self).__init__(socket.AF_INET, socket.SOCK_STREAM)
        self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.settimeout(3)  # Timeout after one second
        self.Address = (ip, port)
        self.BufferLength = 1116
        self.tryConnect()

    def tryConnect(self):
        r"""
        Shutdown the socket to avoid errors when connecting. Raises an Exception
        when the socket does not simply raise a NotConnectedError. Finally
        connected to the fresh socket.
        """
        try:
            self.shutdownSafely(verbose=False)
        except OSError as e:
            if e.errno != errno.ENOTCONN:  # Catch OSError 57: socket not connected
                raise
        finally:
            self.connectSafely()

    def renewSocket(self):
        r"""
        Renew connection to the socket by calling the parent __init__ function
        again through super(). This resets the socket entirely.
        """
        super(Reader, self).__init__(socket.AF_INET, socket.SOCK_STREAM)

    def connectSafely(self):
        r"""
        Connect to the socket via the provided Address. Catch a timeout error
        and shutdown the socket before passing a ConnectionError to the parent,
        to inform that a connection was not established.
        """
        try:
            self.connect(self.Address)
            print(self.Address, "is safely connected")
        except socket.timeout:
            self.shutdownSafely()
            raise ConnectionError('{} connection timed out.'.format(self.Address)) from None

    def isClosed(self):
        r""" Use private methods from the socket to test if it's alive."""
        return self._closed

    def shutdownSafely(self, verbose):
        r""" Class method to override by any instance. """
        raise NotImplementedError("shutdownSafely() method not implemented")


class ModBusReader(Reader):
    r"""
    Class used to communicate with the UR modbus in real-time. This is safer and
    more robust than listening to the robot parameters through URscript, which
    is now reserved only for sending commands through the RobotCCO.

    Attributes:
    -------
    ToolBitChanged : bool
        The bool that signifies whether the ToolBit changed or not.
    SpikeOccurred : bool
        The bool that signifies whether a spike occured in the ToolCurrent.
    ListOfCurrents: list
        The list of electrical currents read from the toolbit to determine when
        a spike has occured.

    ToolBit : ParameterInfo(bit)
        The bit that signifies whether the gripper is cirrently open or closed.
    ToolCurrent : ParameterInfo(float)
        The electrical current through the gripper motor. Necessary to decide
        exactly when the gripper is closed after commanding it to close, as the
        ToolBit is changed by URscript before the event actually happens. The
        event is caught by detecting a spike in current.
    BaseAngle : ParameterInfo(float)

    BaseAngle : ParameterInfo(float)
        The angle at which the base of the robot is rotated.
    ShoulderAngle : ParameterInfo(float)
        The angle at which the shoulder of the robot is rotated.
    ElbowAngle : ParameterInfo(float)
        The angle at which the elbow of the robot is rotated.
    Wrist1Angle : ParameterInfo(float)
        The angle at which the first wrist of the robot is rotated.
    Wrist2Angle : ParameterInfo(float)
        The angle at which the second wrist of the robot is rotated.
    Wrist3Angle : ParameterInfo(float)
        The angle at which the third wrist of the robot is rotated.
    toolX : ParameterInfo(float)
        The x-location of the tool in space with respect to the origin.
    toolY : ParameterInfo(float)
        The y-location of the tool in space with respect to the origin.
    toolZ : ParameterInfo(float)
        The z-location of the tool in space with respect to the origin.
    toolRX : ParameterInfo(float)
        The angle at which the tool is rotated about the x-axis.
    toolRY : ParameterInfo(float)
        The angle at which the tool is rotated about the y-axis.
    toolRZ : ParameterInfo(float)
        The angle at which the tool is rotated about the z-axis.

    StopCommunicating: Event
        The event that signals that communication with the modbus should halt.
    CommunicationThread : Thread
        The thread that updates all ParameterInfo instances.
    """

    def __init__(self):
        IP = "192.168.1.17"
        PORT = 502

        self.ToolBitChanged = False
        self.SpikeOccurred = False
        self.ListOfCurrents = [0]*60
        # Read these parameters from the modbus:
        self.ToolBit       = ParameterInfo(1,   b'\x00\x01', "Vector of output bits. Only interested in number eight.", self.extractToolBit)
        self.ToolCurrent   = ParameterInfo(770, b'\x03\x02', "Current applied to the gripper (mA).", self.extractToolCurrent)
        # Set the robot joint angles as attributes that are continuously updated:
        self.BaseAngle     = ParameterInfo(270, b'\x01\x0E', "Position (angle) of the base joint (milli rad).", self.extractAngle)
        self.ShoulderAngle = ParameterInfo(271, b'\x01\x0F', "Position (angle) of the shoulder joint (milli rad).", self.extractAngle)
        self.ElbowAngle    = ParameterInfo(272, b'\x01\x10', "Position (angle) of the elbow joint (milli rad).", self.extractAngle)
        self.Wrist1Angle   = ParameterInfo(273, b'\x01\x11', "Position (angle) of the wrist1 joint (milli rad).", self.extractAngle)
        self.Wrist2Angle   = ParameterInfo(274, b'\x01\x12', "Position (angle) of the wrist2 joint (milli rad).", self.extractAngle)
        self.Wrist3Angle   = ParameterInfo(275, b'\x01\x13', "Position (angle) of the wrist3 joint (milli rad).", self.extractAngle)
        # Positions only seem to start from the 588th byte
        self.toolX         = ParameterInfo(400, b'\x01\x90', "Cartesian Tool Coordinate X (tenth of mm in base frame).", self.extractToolInfo)
        self.toolY         = ParameterInfo(401, b'\x01\x91', "Cartesian Tool Coordinate Y (tenth of mm in base frame).", self.extractToolInfo)
        self.toolZ         = ParameterInfo(402, b'\x01\x92', "Cartesian Tool Coordinate Z (tenth of mm in base frame).", self.extractToolInfo)
        self.toolRX        = ParameterInfo(403, b'\x01\x93', "Cartesian Tool Orientation RX (milli rad in base frame).", self.extractAngle)
        self.toolRY        = ParameterInfo(404, b'\x01\x94', "Cartesian Tool Orientation RY (milli rad in base frame).", self.extractAngle)
        self.toolRZ        = ParameterInfo(405, b'\x01\x95', "Cartesian Tool Orientation RZ (milli rad in base frame).", self.extractAngle)

        CommunicatingStarted = Event()
        self.StopCommunicating = Event()
        self.CommunicationThread = Thread(target=self.readContinuously, args=[CommunicatingStarted, self.StopCommunicating], daemon=True, name='ModBusReaderThread')

        # Startup parent after creation of all attributes:
        super(ModBusReader, self).__init__(IP, PORT)
        # At this moment the stop event is set due to unknown reasons. Clear the event.
        self.StopCommunicating.clear()
        self.CommunicationThread.start()
        # Wait for the first value to be read:
        CommunicatingStarted.wait()

    def readContinuously(self, communicating_started_event, stop_communicating_event):
        r"""
        Continuously communicate with the modbus through a loop that is only
        halted if the StopCommunicating is raised. Catch errors here to
        ensure the CommunicationThread keeps communicating.
        """
        while not stop_communicating_event.is_set():
            try:
                self.read(communicating_started_event)
            except OSError as e:
                print("Error reading. {}".format(e))
                self.renewSocket()

    def isConnected(self):
        r"""
        Test whether the socket is communicating or not. This is better than the
        fileno() or _closed approaches, as those only measure if the socket is
        closed or not.
        """
        return not self.StopCommunicating.isSet()

    def extractToolBit(self, data):
        r"""
        Convert the hexadecimal data to a list of bits. Read the bit that
        corresponds to the gripper state (bit 8) and see if it differs from the
        ToolBit value, to check if the state has changed so we can look for a spike.
        """
        allBits = [int(x) for x in bin(int(data))[2:]][:9:-1]
        gripperBit = 8
        ToolBitValue = allBits[gripperBit]
        if self.ToolBit.Value != ToolBitValue:
            self.ToolBitChanged = True
        return ToolBitValue

    def extractToolCurrent(self, data):
        r"""
        The last two digits of the hexadecimal data contain the value of the
        electrical current. Check if a spike occurred and signal that to other methods.
        """
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
        # Encode output as: until ToolBitChanged == True we haven't settled the current yet!
        return not self.ToolBitChanged

    @staticmethod
    def extractAngle(data):
        r"""
        Convert the hexadecimal data of a joint angle to a floating point value.
        Check for negative values by using 2's complement.
        """
        value = int(data[-4:], 16)
        if value > 6284:  # If integer value > 6284 we need to take the complement
            value = -(65535 + 1 - value)
        return value * 1.0e-3

    @staticmethod
    def extractToolInfo(data):
        r"""
        Convert the hexadecimal data of a tool position to a floating point value.
        Check for negative values by using 2's complement.
        """
        value = int(data[-4:], 16)
        if value > 32768:  # If integer value > 32767 we need to take the complement
            value = -(65535 + 1 - value)
        return value * 1.0e-4

    def read(self, communicating_started_event):
        r"""
        For every instance of ParameterInfo, send a request for more information
        to the modbus given the hexadecimal address of the value we wich to
        update. Process the values and store them in the appropriate queues for
        concurrent access to the latest value.

        The sent message is a bytes string. Example:
        b'\x00\x04\x00\x00\x00\x06\x00\x03\x00\x81\x00\x01'
        00 04 : the transaction identifier the Modbus TCP follows
        00 00 : a protocol indentifier
        00 06 : messagelength – 6 bytes will follow
        00    : unit identifier (or slave address)
        03    : the function code for 'reading'
        00 81 : the data address of the first register requested (81hex = 129 dec).
        00 01 : the total number of requested registers
        """
        parameters = list(ParameterInfo.getInstances())
        for parameter in parameters:
            self.send(b'\x00\x04\x00\x00\x00\x06\x00\x03' + parameter.Address + b'\x00\x01')
            data = self.recv(self.BufferLength).hex()
            if len(data) == 0:
                continue
            if callable(parameter.Method):  # Call custom methods
                parameter.Value = parameter.Method(data)
        communicating_started_event.set()

    def getToolBitInfo(self):
        return self.ToolBit.Value, not self.ToolBitChanged

    def getToolPosition(self):
        return [self.toolX.Value, self.toolY.Value, self.toolZ.Value, self.toolRX.Value, self.toolRY.Value, self.toolRZ.Value]

    def getJointAngles(self):
        return [self.BaseAngle.Value, self.ShoulderAngle.Value, self.ElbowAngle.Value, self.Wrist1Angle.Value, self.Wrist2Angle.Value, self.Wrist3Angle.Value]

    def shutdownSafely(self, verbose=True):
        r"""
        Shutdown the socket and the running processes one by one.
        """
        if verbose:
            print(self.Address, "shutting down safely.")
        if not self.StopCommunicating.isSet():
            self.StopCommunicating.set()
        if self.CommunicationThread.is_alive():
            self.CommunicationThread.join()
            self.StopCommunicating.clear()
        if not self.isClosed():
            self.shutdown(socket.SHUT_RDWR)
            self.close()


class RobotCCO(Reader):  # RobotChiefCommunicationOfficer
    r"""
    Class used to represent the UR5 robot arm, with which we can communicate
    through URscript. This makes it possible to send commands like moving or
    gripper operations, while the listening is handled by the ModBusReader.
    """

    def __init__(self):
        IP = "192.168.1.17"
        PORT = 30003
        super(RobotCCO, self).__init__(IP, PORT)

    def shutdownSafely(self, verbose=True):
        if verbose:
            print(self.Address, "shutting down safely.")
        if not self.isClosed():  # Use private methods from the socket to test if it's alive
            self.shutdown(socket.SHUT_RDWR)
            self.close()


if __name__ == '__main__':
    mb = ModBusReader()
    rcc = RobotCCO()
