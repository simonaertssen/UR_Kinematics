import time
import winsound

from queue import Queue
from threading import Thread, Event

from Readers import ModBusReader, RobotCCO

from KinematicsModule.Kinematics import RPY2RotVec  # Slow Python implementation
from KinematicsLib.cKinematics import ForwardKinematics, detectCollision  # Fast C and Cython implementation


class Robot:
    r"""
    Class used to represent the UR5 robot, which consists of the ModBusReader to
    listen to and aqcuire all the robot information, and the RobotCCO, to which
    we can send commands. This class implements the move commands as a blocking
    thread, so that we can wait for the target position to be reached while
    using full concurrency.

    Attributes:
    -------
    ModBusReader : ModBusReader
        The instance of the ModBusReader class to listen to parameters like the
        current joint angles or tool position.
    RobotCCO : RobotCCO
        The instance of the RobotCCO class to send commands via URscript.

    pidiv180 : float
        The value of pi/180 for conversion between angles in degrees to radians.
    ToolHoverHeight : float
        The height of the tool before picking up an object.
    ToolPickUpHeight : float
        The height of the tool when picking up an object.
    JointAngleInit : list of angles in radians
        The joint angles required for the initial position.
    JointAngleBrickDrop: list of angles in radians
        The joint angles required for an item to be dropped in the bucket.
    ToolPositionBrickDrop : list of tool positions in mm and angles in radians
        The tool position required for an item to be dropped in the bucket.
    ToolPositionLightBox : list of tool positions in mm and angles in radians
        The tool position of the tool positioned at the lower left corner of the
        light box, with the tool aligned vertically, facing down.
    """
    StopEvent = Event()

    def __init__(self):
        super(Robot, self).__init__()
        self.ModBusReader = None
        self.RobotCCO = None
        self.tryConnect()

        # Save some important positions as attributes:
        self.pidiv180 = 3.14159265359/180
        self.ToolHoverHeight = 0.06
        self.ToolPickUpHeight = 0.009

        self.JointAngleInit = [i * self.pidiv180 for i in [61.42, -93.00, 94.65, -91.59, -90.0, 0.0]]
        self.JointAngleBrickDrop = [i * self.pidiv180 for i in [87.28, -74.56, 113.86, -129.29, -89.91, -2.73]]

        self.ToolPositionBrickDrop = [0.08511, -0.51591, 0.04105, 0.00000, 0.00000, 0.00000]
        self.ToolPositionLightBox = [0.14912, -0.30970, 0.05, 0.000, 3.14159, 0.000]

        self.waitForParallelTask(function_handle=self.initialise, arguments=None, information="Initialising")

    def tryConnect(self):
        r"""
        Try to connect to the ModBusReader and the RobotCCO asynchronously. This
        is faster than starting sequentially. Any errors that were raised are
        caught, and the RobotClass is destructed before passing on the error.
        """
        ReturnErrorMessageQueue = Queue()

        def startAsync(error_queue, constructor):
            try:
                setattr(self, constructor.__name__, constructor())
            except Exception as e:
                # Startup of this part has failed and we need to shutdown all parts
                error_queue.put(e)

        startThreads = [Thread(target=startAsync, args=[ReturnErrorMessageQueue, partname], name='{} startAsync'.format(partname)) for partname in [ModBusReader, RobotCCO]]
        [x.start() for x in startThreads]
        [x.join() for x in startThreads]

        # Get first error, if any, and raise it to warn the instance and the parent
        if not ReturnErrorMessageQueue.empty():
            self.shutdownSafely()
            raise ReturnErrorMessageQueue.get()

    def shutdownSafely(self):
        r"""
        Safely shutdown the ModBusReader and the RobotCCO asynchronously. This
        is faster than starting sequentially.
        """
        # self.initialise()  # Only initialise if we want to reset the robot entirely
        if self.RobotCCO is not None and not self.RobotCCO.isClosed():
            self.halt()

        def shutdownAsync(part):
            if part is None:
                return
            try:
                part.shutdownSafely()
            except Exception as e:
                raise SystemExit("Safe shutdown failed due to {}. Aborting".format(e))

        shutdownThreads = [Thread(target=shutdownAsync, args=[part], name='{} shutdownSafely'.format(part)) for part in [self.ModBusReader, self.RobotCCO]]
        [x.start() for x in shutdownThreads]
        [x.join() for x in shutdownThreads]

    def isConnected(self):
        return self.ModBusReader.isConnected() and not (self.ModBusReader.isClosed() or self.RobotCCO.isClosed())

    def send(self, message):
        r"""
        Send a given bytestring through the socket.

        Parameters:
        ----------
        message : bytes object
            The message that is sent to URscript.
        """
        try:
            self.RobotCCO.send(message)
        except Exception as e:
            print("Sending failed due to {}".format(e))

    def stop(self):
        r"""
        Signal URscript to stop the robot with an accelleration of 5 m/s^2.
        """
        self.send(b'stop(5)')

    def halt(self):
        r"""
        Stop the current concurrent command and stop the robot.
        """
        if not self.StopEvent.isSet():
            self.StopEvent.set()
        self.stop()
        self.StopEvent.clear()

    def receive(self):
        r"""
        Receive a message from the RobotCCO.
        """
        try:
            return self.RobotCCO.recv(self.RobotCCO.BufferLength)
        except Exception as e:
            print("Receiving failed due to {}".format(e))
            return b''  # An empty bytes object

    def getToolBitInfo(self):
        return self.ModBusReader.getToolBitInfo()

    def getToolPosition(self):
        return self.ModBusReader.getToolPosition()

    def getJointAngles(self):
        return self.ModBusReader.getJointAngles()

    def getJointPositions(self):
        X, Y, Z, _, _, _ = self.getToolPosition()
        return ForwardKinematics(tuple(self.getJointAngles()), (X, Y, Z))

    def set_IO_PORT(self, port_number, on):
        r"""
        Set a given IO port of the ModBus to on or off status. This is much
        simpler through URscript, as the ModBus is only used for listening and
        the syntax is complicated.

        Parameters:
        ----------
        port_number : int
            The ModBus port number we wish to set.
        on: boolean
            The value of the port. "True" = 1, "False" = 0.
        """
        if not (0 <= port_number <= 8) or not isinstance(port_number, int):
            print("set_IO_PORT: port number value error")
            return
        if not isinstance(on, bool):
            print("set_IO_PORT: boolean value error")
            return
        command = str.encode('set_digital_out({},{}) \n'.format(port_number, on))
        self.send(command)

    def turnWhiteLampON(self, stop_event):
        r"""
        Preset configuration to set the 0th IO port. Wait for 2 seconds for the
        light to come on. Continue only if stop_event is not set.
        """
        if stop_event.isSet():
            return
        self.set_IO_PORT(0, True)
        time.sleep(2)

    def turnWhiteLampOFF(self, stop_event):
        if stop_event.isSet():
            return
        self.set_IO_PORT(0, False)

    def isGripperOpen(self):
        r"""
        Test if the gripper is open.
        """
        tool_bit, _ = self.getToolBitInfo()
        return tool_bit == 0

    def openGripper(self, stop_event):  # Equals a tool bit of 0
        if stop_event.isSet():
            return
        if self.isGripperOpen():
            print('Gripper is already open')
            return
        self.send(b'set_digital_out(8, False)' + b"\n")
        self.waitForGripperToRead(0)

    def closeGripper(self, stop_event):  # Equals a tool bit of 1
        if stop_event.isSet():
            return
        if not self.isGripperOpen():
            print('Gripper is already closed')
            return
        self.send(b'set_digital_out(8, True)' + b"\n")
        self.waitForGripperToRead(1)

    def waitForGripperToRead(self, bit_value):
        while True:
            tool_bit, settled = self.getToolBitInfo()
            if tool_bit is bit_value and settled:
                break

    def detectCollision(self):
        return detectCollision(self.getJointPositions())

    def moveTo(self, stop_event, target_position, move, p=True, wait=True, check_collisions=True):
        r"""
        Moves the robot to the target, while blocking the thread which calls this
        function until either the target position is reached or until the
        stop event is set or until a collision is detected.

        Parameters:
        ----------
        stop_event: Event
            The threading event designed to halt the execution of a thread.
        target_position : list
            The target position, given by either joint angles or a tool position.
        move: str
            The motion: movej (find best move) or movel (move in a line).
        p : bool
            The boolean that defines wether the target is a set of joint angles
            (p=False) or a tool position (p=True).
        wait: bool
            Wait for the program to reach the required position (blocking or not).
        check_collisions: boole
            Check whether a collision occurs during the move.
        """
        if stop_event.isSet():
            return

        if p:
            current_position = self.getToolPosition
        else:
            current_position = self.getJointAngles

        command = str.encode("{}({}{}) \n".format(move, "p" if p is True else "", target_position))
        self.send(command)

        start_position = current_position()
        if wait:
            try:
                self.waitUntilTargetReached(current_position, target_position, check_collisions, stop_event)
            except TimeoutError as e:  # Time ran out to test for object position
                print(e)
            except RuntimeError as e:  # Collision raises RuntimeError, so move to startpoition
                self.stop()
                time.sleep(0.1)
                self.moveTo(stop_event, start_position, "movel", wait=True, p=p, check_collisions=False)
            finally:
                time.sleep(0.075)  # To let momentum fade away

    @staticmethod
    def spatialDifference(current_position, target_position):
        r"""
        Compute the L2 spatial distance between two tool positions.

        Parameters:
        ----------
        current_position : list
            The current position, given by a tool position.
        target_position : list
            The target position, given by a tool position.
        """
        x1, y1, z1, _, _, _ = current_position
        x2, y2, z2, _, _, _ = target_position
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) ** 0.5

    def waitUntilTargetReached(self, current_position, target_position, check_collisions, stop_event):
        r"""
        Block the moveTo command until either the target position is reached or
        until the stop event is set or until a collision is detected.
        """
        difference = [1000.0 for _ in target_position]
        start_time = time.time()

        totalDifferenceTolerance = 5e-3
        while not stop_event.isSet() and sum(difference) >= totalDifferenceTolerance:
            difference = [abs(joint - pos) for joint, pos in zip(current_position(), target_position)]
            if time.time() - start_time > 15.0:
                raise TimeoutError('Movement took too long')
            if check_collisions and self.detectCollision():
                raise RuntimeError('Bumping in to stuff!')

    def waitForParallelTask(self, function_handle, arguments=None, information=None):
        r"""
        Start a command within a thread and wait for its completion to achieve concurrency.

        Parameters:
        ----------
        function_handle : function handle
            The function we wish to run concurrently. Should accept a single
            argument, the stop event.
        arguments : list
            The additional arguments a function could need. Included if such
            situations would emerge in future development.
        information : str
            The information we wish to display as the name of the thread.
        """
        if information:
            print('Task received:', information)
        thread_args = (self.StopEvent,)
        if arguments is not None:
            thread_args = [thread_args]
            thread_args.extend(arguments)
        thread = Thread(target=function_handle, args=thread_args, daemon=True, name=information)
        thread.start()
        thread.join()
        if self.StopEvent.isSet():
            self.StopEvent.clear()

    def moveToolTo(self, stop_event, target_position, move, wait=True, check_collisions=True):
        r"""
        Wrapper for the moveTo command to distinguish clearly between tool and
        joint commands.
        If the stop_event is given, then a thread is calling this function. If
        it is not given, then we want to start a thread. This avoids having two
        different funcions with two different but similar names.
        """
        self.moveTo(stop_event, target_position, move, wait=wait, p=True, check_collisions=check_collisions)

    def moveJointsTo(self, stop_event, target_position, move, wait=True, check_collisions=True):
        r"""
        Wrapper for the moveTo command to distinguish clearly between tool and
        joint commands.
        If the stop_event is given, then a thread is calling this function. If
        it is not given, then we want to start a thread. This avoids having two
        different funcions with two different but similar names.
        """
        self.moveTo(stop_event, target_position, move, wait=wait, p=False, check_collisions=check_collisions)

    def goHome(self, stop_event):
        self.moveJointsTo(stop_event, self.JointAngleInit.copy(), "movej")

    def dropObject(self, stop_event):
        if stop_event.isSet():
            return
        self.moveJointsTo(stop_event, self.JointAngleBrickDrop.copy(), "movej")
        self.openGripper(stop_event)

    def pickUpObject(self, stop_event, object_position):
        r"""
        Sequence of moves that are required to pick up an object that was
        detected by the topCamera.
        """
        if stop_event.isSet():
            return

        LIGHTBOX_LENGTH = 0.250  # m
        LIGHTBOX_WIDTH = 0.176  # m
        print("object_position: ", object_position)
        if len(object_position) < 1:
            return
        X, Y, angle = object_position

        # Adjust position to the object
        target_position = self.ToolPositionLightBox.copy()
        target_position[0] += X * LIGHTBOX_WIDTH   # adjust X position
        target_position[1] -= Y * LIGHTBOX_LENGTH  # adjust Y position
        target_position[2] = self.ToolHoverHeight
        # Get right orientation from Rodrigues conversion
        a, b, c = RPY2RotVec(0, 3.1415926, -angle)
        target_position[3] = a
        target_position[4] = b
        target_position[5] = c
        self.moveToolTo(stop_event, target_position, 'movel')
        # Go down and pickup the object
        target_position[2] = self.ToolPickUpHeight
        self.moveToolTo(stop_event, target_position, 'movel')
        self.closeGripper(stop_event)
        # Go back up
        target_position[2] = self.ToolPickUpHeight
        self.moveToolTo(stop_event, target_position, 'movel')

    def initialise(self, stop_event):
        r"""
        Sequence of moves that are required to initialise the robot safely, like
        dropping any objects the gripper is still holding onto.
        """
        if stop_event.isSet():
            return

        currentJointPosition = self.getJointAngles()
        distanceFromAngleInit = sum([abs(i - j) for i, j in zip(currentJointPosition, self.JointAngleInit.copy())])
        currentToolPosition = self.getToolPosition()
        if self.isGripperOpen():
            if currentToolPosition[2] < 0.300:
                targetToolPosition = currentToolPosition.copy()
                targetToolPosition[2] = 0.300
                # Move towards first location, don't check collisions
                # because we might start from a bad position.
                self.moveToolTo(stop_event, targetToolPosition, "movel", check_collisions=False)
        else:
            if self.spatialDifference(currentToolPosition, self.ToolPositionBrickDrop) < 0.5:
                if currentToolPosition[2] < 0.07:
                    targetToolPosition = currentToolPosition.copy()
                    targetToolPosition[2] = 0.07
                    self.moveToolTo(stop_event, targetToolPosition, "movel")
            else:
                if distanceFromAngleInit > 0.05:
                    self.goHome(stop_event)

            self.dropObject(stop_event)
        if distanceFromAngleInit > 0.05:
            self.goHome(stop_event)
        print("Initialisation Done")

    def test(self):
        print('Testing the gripper')
        self.closeGripper(self.StopEvent)
        self.openGripper(self.StopEvent)

    @staticmethod
    def beep():
        r"""
        Play a sound as confirmation.
        """
        winsound.PlaySound("SystemHand", winsound.SND_NOSTOP)


if __name__ == '__main__':
    robot = Robot()
    # robot.moveToolTo(robot.StopEvent, robot.ToolPositionLightBox.copy(), "movel", wait=False)
    robot.beep()
