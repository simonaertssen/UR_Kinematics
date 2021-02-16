import time
import winsound

from queue import SimpleQueue, LifoQueue, Empty, Full
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
    JointAngleDropObject: list of angles in radians
        The joint angles required for an item to be dropped in the bucket.
    ToolPositionDropObject : list of tool positions in mm and angles in radians
        The tool position required for an item to be dropped in the bucket.
    ToolPositionLightBox : list of tool positions in mm and angles in radians
        The tool position of the tool positioned at the lower left corner of the
        light box, with the tool aligned vertically, facing down.
    """

    ModBusReader = ModBusReader
    RobotCCO = RobotCCO

    # Save some important positions as attributes:
    ToolHoverHeight = 0.06
    ToolPickUpHeight = 0.009

    JointAngleInit = [i * 3.141593/180 for i in [61.42, -93.00, 94.65, -91.59, -90.0, 0.0]]
    JointAngleDropObject = [i * 3.141593/180 for i in [87.28, -74.56, 113.86, -129.29, -89.91, -2.73]]
    # self.JointAngleReadObject = [i * self.pidiv180 for i in [-0.068, -91.45, 94.01, -92.59, 87, 180]]
    # JointAngleReadObjectOld = [i * 3.141593/180 for i in [-0.053, -91.12, 94.04, -94.04, 87.04, 188.24]]
    JointAngleReadObject = [i * 3.141593 / 180 for i in [0.00, -90.00, 90.00, -90.00, 90.00, 180.00]]

    ToolPositionDropObject = [0.08511, -0.51591, 0.04105, 0.00000, 0.00000, 0.00000]
    ToolPositionLightBox = [0.14912, -0.31000, 0.05, 0.000, 3.14159, 0.000]
    ToolPositionReadObject = [-0.48117, -0.10529, 0.62605, 0.0007, 0.0208, 0.0134]
    ToolPositionTestCollision = [0.04860, -0.73475, 0.30999, 0.7750, 3.044, 0.002]

    StopEvent = Event()  # Stop the robot class from running
    # print(StopEvent)
    StopTaskEvent = Event()  # Stop the current task from running
    # print(StopTaskEvent)
    TaskFinishedEvent = Event()  # Signal the current task is finished
    TaskQueue = LifoQueue()
    TaskThread = Thread(args=[TaskQueue, StopTaskEvent, StopEvent, TaskFinishedEvent], daemon=True, name='Async Robot tasks')

    def __init__(self):
        super(Robot, self).__init__()
        self.tryConnect()
        self.TaskThread._target = self.runTasks
        self.giveTask(self.initialise)
        self.TaskThread.start()

    def tryConnect(self):
        r"""
        Try to connect to the ModBusReader and the RobotCCO asynchronously. This
        is faster than starting sequentially. Any errors that were raised are
        caught, and the RobotClass is destructed before passing on the error.
        """
        ReturnErrorMessageQueue = SimpleQueue()

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

    def stop(self):
        r"""
        Signal URscript to stop the robot with an accelleration of 0.001 m/s^2.
        For the robot to receive this message immediately and for the robot to respond to future
        commands immediately, we need to send an IO command. Don't kow why but it works.
        """
        self.send(b'set_digital_out(7, False)')  # Necessary to give the robot another command first
        self.send(b'stop(0.001)')
        self.send(b'set_digital_out(7, False)')

    def halt(self):
        r"""
        Stop the current concurrent command and stop the robot.
        """
        self.stop()
        if not self.StopTaskEvent.isSet():
            self.StopTaskEvent.set()
        self.clearTasks()
        self.TaskFinishedEvent.wait()  # Wait for current task to finish before going home

    def shutdownSafely(self):
        r"""
        Safely shutdown the ModBusReader and the RobotCCO asynchronously. This is faster than
        starting sequentially. Only initialise again if we wish to reset the robot entirely.
        The current Robot task will be halted with the StopTaskEvent, upon which
        self.runTasks will signal that the task is finished and will clear StopTaskEvent.
        """
        self.halt()
        self.giveTask(self.initialise)
        self.TaskFinishedEvent.wait()
        if self.TaskThread.is_alive():
            self.StopEvent.set()
            self.TaskThread.join()

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

    @staticmethod
    def runTasks(robot_task_queue, task_stop_event, robot_stop_event, robot_task_finished_event):
        while not robot_stop_event.is_set():  # Continue for as long as the robot is running
            try:  # See if there is a new task
                task_handle = robot_task_queue.get(timeout=0.01)
            except Empty as e:
                # Yes, you know that emptying the queue raises an error
                continue  # To the next iteration of the loop

            try:  # See if we can execute the function
                task_handle(task_stop_event)
            except TypeError as e:
                print('An uncallable function was encountered: {}'.format(e))
            finally:  # Signal that the task was performed in some way
                robot_task_finished_event.set()
                if task_stop_event.isSet():
                    # If the event was raised, clear it and signal that the task was stopped.
                    # This yields the robot ready for the coming task.
                    task_stop_event.clear()

    def clearTasks(self):
        while True:
            try:
                self.TaskQueue.get_nowait()
            except Empty:
                break

    def giveTask(self, function_handle):
        # Signal that the given task is not finished yet
        self.TaskFinishedEvent.clear()
        self.TaskQueue.put(function_handle)

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
            self.RobotCCO.send(message + b'\n')
        except Exception as e:
            print("Sending failed due to {}".format(e))

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
        command = str.encode('set_digital_out({},{})'.format(port_number, on))
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
        self.send(b'set_digital_out(8, False)')
        self.waitForGripperToRead(0, stop_event)

    def closeGripper(self, stop_event):  # Equals a tool bit of 1
        if stop_event.isSet():
            return
        if not self.isGripperOpen():
            print('Gripper is already closed')
            return
        self.send(b'set_digital_out(8, True)')
        self.waitForGripperToRead(1, stop_event)

    def waitForGripperToRead(self, bit_value, stop_event):
        MAX_TIME = 2.0  # seconds
        start_time = time.time()
        while not stop_event.isSet():
            tool_bit, settled = self.getToolBitInfo()
            if tool_bit is bit_value and settled:
                break
            if time.time() - start_time > MAX_TIME:
                break
            time.sleep(0.01)  # Sometimes this loop is too fast and the value is read wrongly.

    def testGripper(self):
        print('Testing the gripper')
        if self.isGripperOpen():
            self.closeGripper(self.StopEvent)
            self.openGripper(self.StopEvent)
        else:
            self.openGripper(self.StopEvent)
            self.closeGripper(self.StopEvent)

    def detectCollision(self):
        return detectCollision(self.getJointPositions())

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
        check_collisions: bool
            Check whether a collision occurs during the move.
        """
        if stop_event.isSet():
            return

        if p:
            current_position = self.getToolPosition
        else:
            current_position = self.getJointAngles

        MAX_SPEED = 0.1  # m/s
        command = str.encode("{}({}{}, v={})".format(move, "p" if p is True else "", target_position, MAX_SPEED))
        self.send(command)

        start_position = current_position()
        if wait:
            try:
                self.waitUntilTargetReached(current_position, target_position, p, check_collisions, stop_event)
            except TimeoutError as e:  # Time ran out to test for object position
                print(e)
            except InterruptedError as e:  # StopEvent is raised
                print(e)
            except RuntimeError as e:  # Collision raises RuntimeError, so move to startposition
                self.halt()
                time.sleep(0.1)
                self.moveTo(stop_event, start_position, "movel", wait=True, p=p, check_collisions=False)
            finally:
                time.sleep(0.1)  # To let momentum fade away

    def waitUntilTargetReached(self, current_position, target_position, p, check_collisions, stop_event):
        r"""
        Block the moveTo command until either the target position is reached or
        until the stop event is set or until a collision is detected.
        """
        difference = [1000.0 for _ in target_position]
        start_time = time.time()
        MAX_TIME = 5.0

        # if p:  # Constrain on position
        RELATIVE_TOLERANCE = 1e-3  # Robot arm should be accurate up to 1mm
        ABSOLUTE_TOLERANCE = 6e-3  # Total difference should not exceed 6*tolerance on each joint
        # else:
        #     RELATIVE_TOLERANCE = 2e-3
        #     ABSOLUTE_TOLERANCE = 8e-3  # Robot arm should be accurate up to 1 mrad
        while sum(difference) > ABSOLUTE_TOLERANCE or all(d > RELATIVE_TOLERANCE for d in difference):
            if stop_event.isSet() is True:
                raise InterruptedError("Stop event has been raised.")
            if check_collisions and self.detectCollision():
                raise RuntimeError('Bumping in to stuff!')
            if time.time() - start_time > MAX_TIME:
                raise TimeoutError('Movement took longer than {} s. Assuming robot is in position and continue.'.format(MAX_TIME))
            difference = [abs((joint - pos + 3.14159) % 6.2831 - 3.14159) for joint, pos in zip(current_position(), target_position)]

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

    def goHome(self, stop_event, wait=True):
        self.moveJointsTo(stop_event, self.JointAngleInit.copy(), "movej", wait=wait)

    def dropObject(self, stop_event):
        if stop_event.isSet():
            return
        self.moveJointsTo(stop_event, self.JointAngleDropObject.copy(), "movej")
        self.openGripper(stop_event)

    def presentObject(self, stop_event):
        # self.moveToolTo(stop_event, self.ToolPositionLightBox.copy(), 'movej')
        # Adjust position to the object
        target_position = self.ToolPositionReadObject.copy()
        # Get right orientation from Rodrigues conversion
        a, b, c = RPY2RotVec(0, 0, 0)
        target_position[3] = a
        target_position[4] = b
        target_position[5] = c
        print(target_position)
        self.moveToolTo(stop_event, target_position, 'movej')

    def pickUpObject(self, stop_event, object_position):
        r"""
        Sequence of moves that are required to pick up an object that was
        detected by the topCamera.
        """
        if stop_event.isSet():
            return

        LIGHTBOX_LENGTH = 0.250  # m
        LIGHTBOX_WIDTH = 0.176  # m
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
        target_position[2] = self.ToolHoverHeight
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
            if currentToolPosition[2] <= 0.300:
                targetToolPosition = currentToolPosition.copy()
                targetToolPosition[2] = 0.310
                # Move towards first location, don't check collisions
                # because we might start from a bad position.
                self.moveToolTo(stop_event, targetToolPosition, "movel", check_collisions=False)
        else:
            if self.spatialDifference(currentToolPosition, self.ToolPositionDropObject) < 0.5:
                if currentToolPosition[2] < 0.07:
                    targetToolPosition = currentToolPosition.copy()
                    targetToolPosition[2] = 0.07
                    self.moveToolTo(stop_event, targetToolPosition, "movel")
            else:
                if distanceFromAngleInit > 0.05:
                    self.goHome(stop_event)

            self.dropObject(stop_event)
        if distanceFromAngleInit > 0.1:
            self.goHome(stop_event)

    @staticmethod
    def beep():
        r"""
        Play a sound as confirmation.
        """
        winsound.PlaySound("SystemHand", winsound.SND_NOSTOP)


if __name__ == '__main__':
    robot = Robot()
