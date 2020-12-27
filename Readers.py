import socket
import errno

from sys import exit
from weakref import ref
from queue import Queue

from threading import Thread, Event, Lock


class ParameterInfo:
    """
    Class used to represent a parameter of the UR5 robot, with all the necessary
    information to retrieve it from the UR5 modbus. This makes reading joint
    angles, positions, voltages and velocities easier.
    A list of instances is saved to quickly yield all instances of this class.

    Attributes:
    -------
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
        Numerical value of the parameter.

    Example:
    -------
    -> ToolCurrent = ParameterInfo(770, b'\x03\x02', "Tool current (mA).", None)
    The decimal address 770 is 302 in hexadecimal. This yields the tool current.
    No method to apply on this value.
    """

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
        """
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
    """
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
        self.settimeout(1) # Timeout after one second
        self.Address = (ip, port)
        self.BufferLength = 1116
        self.ThreadLock = Lock()
        self.tryConnect()

    def testHostIP(self):
        """
        Test whether the given IP is on the right subnet. Connection seems to
        work on the JLI net with a host IP of 192.168.111.6.
        My personal computer 'MacBook-Pro-van-Simon.local' was added to make
        testing at home easier. Delete this condition when using the code.
        """
        HOST_NAME = socket.gethostname()
        HOST_IP   = socket.gethostbyname(HOST_NAME)
        if HOST_NAME != 'MacBook-Pro-van-Simon.local' and HOST_IP != '192.168.111.6':
            raise ConnectionError("Verify IP of robotarm and cameras are on the same subnet.")

    def tryConnect(self):
        """
        Shutdown the socket to avoid errors when connecting. Raises an Exception
        when the socket does not simply raise a NotConnectedError. Finally
        connected to the fresh socket.
        """
        self.testHostIP()
        try:
            self.shutdownSafely(verbose=False)
        except OSError as e:
            if e.errno != errno.ENOTCONN:  # Catch OSError 57: socket not connected
                raise
        finally:
            self.connectSafely()

    def renewSocket(self):
        """
        Renew connection to the socket by calling the parent __init__ function
        again through super(). This resets the socket entirely.
        """
        super(Reader, self).__init__(socket.AF_INET, socket.SOCK_STREAM)

    def connectSafely(self):
        """
        Connect to the socket via the provided Address. Catch a timeout error
        and shutdown the socket before passing a ConnectionError to the parent,
        to inform that a connection was not established.
        """
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
    """
    Class used to communicate with the UR modbus in real-time. This is safer and
    more robust than listening to the robot parameters through URscript, which
    is now reserved only for sending commands through the RobotCCO.

    Attributes:
    -------
    ToolBitQueue : Queue containing a tuple
        The queue that contains the newest info about the state of the gripper,
        which is read directly from the appropriate modbus address.
    ToolPositionQueue : Queue containing a list
        The queue that contains the newest info about the tool position, which
        is read directly from the appropriate modbus address.
    JointAngleQueue : Queue containing a list
        The queue that contains the newest info about the robot joint angles,
        which are read directly from the appropriate modbus addresses.

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

    StopCommunicatingEvent: Event
        The event that signals that communication with the modbus should halt.
    CommunicationThread : Thread
        The thread that updates all ParameterInfo instances.
    """

    def __init__(self):
        self.ToolBitQueue = Queue()
        self.ToolPositionQueue = Queue()
        self.JointAngleQueue = Queue()

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

        self.StopCommunicatingEvent = Event()
        self.CommunicationThread = Thread(target=self.readContinuously, args=[self.ToolBitQueue, self.ToolPositionQueue, self.JointAngleQueue, self.StopCommunicatingEvent], daemon=True, name='ModBusReaderThread')

        # Startup parent after creation of all attributes:
        IP = "192.168.1.17"
        PORT = 502
        super(ModBusReader, self).__init__(IP, PORT)
        self.CommunicationThread.start()

    def readContinuously(self, tool_bit_queue, tool_position_queue, joint_angle_queue, stop_communicating_event):
        """
        Continuously communicate with the modbus through a loop that is only
        halted if the StopCommunicatingEvent is raised. Catch errors here to
        ensure the CommunicationThread keeps communicating.
        """
        while not stop_communicating_event.is_set():
            try:
                self.read(tool_bit_queue, tool_position_queue, joint_angle_queue)
            except OSError as e:
                print("Error reading. {}".format(e))
                self.renewSocket()

    def isConnected(self):
        """
        Test whether the socket is communicating or not. This is better than the
        fileno() or _closed approaches, as those only measure if the socket is
        closed or not.
        """
        return not self.StopCommunicatingEvent.isSet()

    @staticmethod
    def clearQueue(queue):
        """
        Clear the given queue until so that it is empty.
        """
        while not queue.empty():
            queue.get()

    def storeSafelyInQueue(self, value, queue):
        """
        Atomically add the given value to the given queue, which is also emptied.
        """
        if value is not None:
            with self.ThreadLock:
                self.clearQueue(queue)
                queue.put(value)

    def extractToolBit(self, data):
        """
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

    def extractToolCurrent(self, data, tool_bit_queue):
        """
        The last two digits of the hexadecimal data contain the value of the
        electricalcurrent. Check if a spike occured and signal that to other methods.
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
        # Encode output as: as long as ToolBitChanged == True we haven't settled the current yet!
        # So as long as the output is False we haven't settled.
        self.storeSafelyInQueue((self.ToolBit.Value, not self.ToolBitChanged), tool_bit_queue)
        return not self.ToolBitChanged

    @staticmethod
    def extractAngle(data):
        """
        Convert the hexadecimal data of a joint angle to a floating point value.
        Check for negative values by using 2's complement.
        """
        value = int(data[-4:], 16)
        if value > 6284:  # If integer value > 6284 we need to take the complement
            value = -(65535 + 1 - value)
        return value * 1.0e-3

    @staticmethod
    def extractToolInfo(data):
        """
        Convert the hexadecimal data of a tool position to a floating point value.
        Check for negative values by using 2's complement.
        """
        value = int(data[-4:], 16)
        if value > 32768:  # If integer value > 32767 we need to take the complement
            value = -(65535 + 1 - value)
        return value * 1.0e-4

    def read(self, tool_bit_queue, tool_position_queue, joint_angle_queue):
        """
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
        """
        Shutdown the socket and the running processes one by one.
        """
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
    """
    Class used to represent the UR5 robot arm, with which we can communicate
    through URscript. This makes it possible to send commands like moving or
    gripper operations, while the listening is handled by the ModbusReader.
    """

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
