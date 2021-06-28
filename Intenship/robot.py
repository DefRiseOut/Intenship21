import math
import time
import copy
import minimalmodbus as mod
import serial


# Function for finding difference between 2 positions on the motors with looping in mind.
def diff(now, old):
    if abs(now - old) < 3500:
        return now - old
    elif now > old + 3500:
        return now - old - 4096
    else:
        return now - old + 4096


# Config for motor.
def config(motor):
    motor.serial.baudrate = 256000  # Baud
    motor.serial.bytesize = 8
    motor.serial.parity = serial.PARITY_NONE
    motor.serial.stopbits = 1
    motor.serial.timeout = 0.1  # seconds
    motor.mode = mod.MODE_RTU  # rtu or ascii mode
    motor.clear_buffers_before_each_transaction = True
    motor.close_port_after_each_call = False


# Class for synchronization of legs.
class MyRobot:
    def __init__(self, left_legs, right_legs, V, coord, d, R, max_L, port):
        # V is speed. left is a list of all legs, right is a list of right legs.
        # stop is a lock for all the legs. If stop is 1, none of the legs are allowed to move.
        # keys is a list of locks for each leg.
        # speeds is a list of target speeds for legs.
        self.port = port
        self.V = V
        self.left = []
        self.right = []
        self.legs = []
        self.time_step = 0.4
        self.stop = 1
        self.keys = []
        self.speeds = []
        # Connect and config all left legs.
        for leg in left_legs:
            temp_leg = LeftLeg(leg[0], leg[1], coord, d, V, R, self.time_step, max_L, port)
            self.legs.append(temp_leg)
            self.keys.append(False)
            self.left.append(temp_leg)
            self.speeds.append(0)

        # Connect and config all right legs.
        for leg in right_legs:
            temp_leg = RightLeg(leg[0], leg[1], coord, d, V, R, self.time_step, max_L, port)
            self.right.append(temp_leg)
            self.legs.append(temp_leg)
            self.keys.append(False)
            self.speeds.append(0)

        # Number of legs
        self.n = len(self.keys)

    # Function only used by the UI thread to stop the robot.
    def force_stop(self):
        self.stop = 1

    # Infinite loop through the cycle. Only unlocked legs move.
    def inf_loop(self, finish, error):
        try:
            # When called, automatically unlocks the robot.
            self.stop = 0
            t = time.time()
            t0 = t
            t_log = t
            speed_t = t

            # Turns on all legs, but sets their speeds to 0 to gradually increase the speed.
            for i in range(self.n):
                self.turn_on_leg(i, 1)
                if self.speeds[i] == 0:
                    self.speeds[i] = self.legs[i].V
                self.legs[i].change_speed(0)

            # Variable for saving the difference in time between each step.
            step = self.time_step

            # Cycle until robot is stopped
            while True:
                # If robot is stopped, stop all legs and exit the cycle.
                if self.stop == 1:
                    self.stop_all_legs()
                    break

                # For each leg.
                for i in range(self.n):
                    # If the leg is unlocked.
                    leg = self.legs[i]
                    if leg.V != 0:
                        # If there is a collision, stop the robot.
                        if leg.detect_collision():
                            self.stop = 1
                            self.stop_all_legs()
                            error("A collision was detected on leg [%d, %d]." % (leg.front_id, leg.back_id))
                            break
                        # Otherwise, advance the leg.
                        leg.set_new_speed(step)

                        # If wanted speed is different from current, change it by 0.01m/s. Check this every 0.2 seconds.
                        if t > speed_t + 0.3:
                            if abs(leg.V - self.speeds[i]) >= 0.006:
                                if leg.V < self.speeds[i]:
                                    if leg.V + 0.01 == 0:
                                        self.stop_leg(i)
                                    leg.change_speed(leg.V + 0.01)
                                else:
                                    if leg.V - 0.01 == 0:
                                        self.stop_leg(i)
                                    leg.change_speed(leg.V - 0.01)
                    else:
                        # If the leg is stopped, but wanted speed is non-0, we start up the leg.
                        if t > speed_t + 0.3 and abs(self.speeds[i]) >= 0.006:
                            self.turn_on_leg(i, 1)
                            if self.speeds[i] > 0:
                                leg.change_speed(0.01)
                            else:
                                leg.change_speed(-0.01)
                        else:
                            self.stop_leg(i)

                # Increase the time used for speed recalculations, if needed.
                if t > speed_t + 0.3:
                    speed_t += 0.3

                # Write information onto the log.
                if t > t_log + 1:
                    self.log_file(t - t0)
                    t_log += 1

                #  Sleep until next time_step. If time spent exceeded the time step, just
                tim = time.time()
                if t + self.time_step > tim:
                    step = self.time_step
                    t += self.time_step
                    time.sleep(t - tim)
                else:
                    step = tim - t
                    t = tim
            finish()
        except Exception as e:
            print(str(e))
            finish()
            error(str(e))

    # Function that stops all legs at the same time.
    def stop_all_legs(self):
        # Bring speeds to 0, wait 0.1 seconds.
        print("Stopping all legs...")
        for leg in self.legs:
            leg.change_speed(0)
            leg.set_speed_0()
        time.sleep(0.1)

        # Change mode to 0, that stops the motors in place. Wait 0.5 seconds for everything to stop.
        for leg in self.legs:
            leg.change_mode_0()
        time.sleep(0.5)

    # Recalculates the phases for all left legs based on the new speed.
    def recalculate_speed_left(self, speed):
        for j in range(len(self.left)):
            self.speeds[j] = speed

    # Same function but for right legs.
    def recalculate_speed_right(self, speed):
        for j in range(len(self.left), self.n):
            self.speeds[j] = speed

    # Function that sets up movement for 1 leg. It sets the target for that leg on the specified phase. Changes the mode of the leg to 2.
    def set_target_1_leg(self, side, num, phase, alpha, finish, error):
        if self.stop == 1:
            try:
                if side:
                    leg = self.right[num-1]
                else:
                    leg = self.left[num-1]
                leg.set_target_on_cycle(phase, alpha)
                leg.mode = 2
                finish()
                return 0

            except Exception as e:
                print("Error! " + str(e))
                error("Error! " + str(e))
                return -1

        else:
            print("Error, cannot setup phases while robot is running.")
            error("Error, cannot setup phases while robot is running.")
            return -1

    # A simple function to clear the log.txt file.
    @staticmethod
    def clear_log():
        f = open("log.txt", "r+")
        f.truncate(0)
        f.close()

    # Function that writes currents from each leg and their position into the log file. Is called every 0.3 seconds in main cycle.
    # The log file has a structure of: time + \t + current_front;current_back;pos_x;pos_y (for every leg with \t in between them)
    def log_file(self, t):
        with open("log.txt", "a") as f:
            f.write("%s\t%0.2f" % (self.port, t))
            for i in range(self.n):
                f.write("\t")
                leg = self.legs[i]
                curr1 = leg.front.read_register(263, 0, 3, signed=False)
                curr2 = leg.back.read_register(263, 0, 3, signed=False)
                f.write("%0.2f;%0.2f;%0.2f;%0.2f;%d;%d;%0.2f;%0.2f" % (leg.A[0], leg.A[1], leg.B[0], leg.B[1], curr1, curr2, leg.now_coord[0], leg.now_coord[1]))
            f.write("\n")

    # A simple loop that is activated to move all the legs into their specified positions. Only moves legs which have mode 2.
    # finish and error are functions that are called to notify the UI about the finishing of the function, or an error.
    def before_start_init(self, finish, error):
        try:
            # Unlock the robot.
            self.stop = 0

            # count is a counter of legs which have not reached the goal.
            count = 0

            # Unlock individually all legs with mode 2.
            for i in range(self.n):
                if self.legs[i].mode == 2:
                    count += 1
                    self.turn_on_leg(i, 2)
                else:
                    self.stop_leg(i)

            # Time setup.
            t = time.time()
            t0 = t
            t_log = t

            # Variable for difference ion time between steps.
            step = self.time_step

            while count > 0.5:
                # Stop the robot if stop is 1.
                if self.stop == 1:
                    self.stop_all_legs()
                    break

                # For each leg.
                for i in range(self.n):

                    # If the leg is unlocked.
                    if self.keys[i]:
                        leg = self.legs[i]
                        # If there is a collision, stop the robot.
                        if leg.detect_collision():
                            self.stop = 1
                            self.stop_all_legs()
                            error("A collision was detected on leg [%d, %d]." % (leg.front_id, leg.back_id))
                            break

                        # Otherwise, advance the leg.
                        leg.set_new_speed(step)

                        # Give the leg 1 second extra time to stabilize.
                        if leg.t > leg.total+1:

                            # Once we reach the goal, decrease count, stop the leg, and setup its phase and time.
                            count -= 1
                            self.stop_leg(i)
                            leg.setup_cycle()

                t += self.time_step

                # Log every 0.3 seconds.
                if t > t_log + 0.3:
                    self.log_file(t - t0)
                    t_log += 0.3

                # Sleep until next time step.
                tim = time.time()
                if t + self.time_step > tim:
                    step = self.time_step
                    t += self.time_step
                    time.sleep(t - tim)
                else:
                    step = tim - t
                    t = tim
            self.stop = 1
            finish()
            time.sleep(0.3)
        except Exception as e:
            print(str(e))
            finish()
            error(str(e))

    # A function that stops one leg.
    def stop_leg(self, i):
        self.keys[i] = False
        self.legs[i].mode = 0
        self.legs[i].fast_stop()

    # A function that turns on 1 leg into a specified mode.
    def turn_on_leg(self, i, mode):
        self.keys[i] = True
        self.legs[i].mode = mode
        self.legs[i].change_mode_1()

    def clear_save(self):
        with open("save.txt", "w") as f:
            f.truncate()

    # Saves the state of all legs into save.txt. Angles, positions of motors, coordinates, phase, current time, A and B.
    def save_state(self):
        with open("save.txt", "a") as f:
            for leg in self.legs:
                leg.get_current_angle()
                f.write(str(leg.front_id) + " ")
                f.write(str(leg.angle_front) + " ")
                f.write(str(leg.prev_front) + " ")
                f.write(str(leg.angle_back) + " ")
                f.write(str(leg.prev_back) + " ")
                f.write(str(leg.now_coord[0]) + " ")
                f.write(str(leg.now_coord[1]) + " ")
                f.write(str(leg.phase) + " ")
                f.write(str(leg.t) + " ")
                f.write(str(leg.A[0]) + " ")
                f.write(str(leg.A[1]) + " ")
                f.write(str(leg.B[0]) + " ")
                f.write(str(leg.B[1]) + " ")
                f.write(str(leg.V) + "\n")

    # Recovers the information from the save.txt. Afterwards, recalculates the phases for each leg.
    def recover_state(self):
        dic = {}
        with open("save.txt", "r") as f:
            for _ in f:
                motor_id, angleF, posF, angleB, posB, x, y, phase, t, ax, ay, bx, by, d = map(float, (f.readline()).split())
                dic[motor_id] = [angleF, posF, angleB, posB, x, y, phase, t, ax, ay, bx, by, d]

            for leg in self.legs:
                idF = leg.front_id
                leg.angle_front, leg.prev_front, leg.angle_back, = dic[idF][0], dic[idF][1], dic[idF][2]
                leg.prev_back, leg.phase, leg.t = dic[idF][3], dic[idF][6], dic[idF][7]
                leg.now_coord[0], leg.now_coord[1] = dic[idF][4], dic[idF][5]
                leg.A, leg.B = [dic[idF][8], dic[idF][9]], [dic[idF][10], dic[idF][11]]
                leg.V = dic[idF][12]

                if leg.V == 0:
                    leg.V = abs(self.V) * leg.dir

                # Saves A and B into coordinates list. leg.A and leg.B themselves are only used by the leg during DA or CB.
                leg.coordinates[0] = copy.deepcopy(leg.A)
                leg.coordinates[1] = copy.deepcopy(leg.B)

                # Recalculate phases.
                leg.calculate_phases()
                leg.set_phase_cycle()

    # A simple loop that calibrates the location of each leg. It is assumed that every leg is exactly between 2 motors, at any height.
    # After calibration, all legs will move to D.
    # finish and error are functions that are called to notify the UI about the finishing of the function, or an error.
    def calibrate(self, finish, error):
        try:
            # Unlock the robot.
            self.stop = 0

            # Turn all legs into mode 10 - calibration mode.
            for i in range(self.n):
                self.turn_on_leg(i, 10)

            # Double duty counter that counts the number of legs calibrating + number of legs that haven't reached D yet.
            count = self.n * 2

            # Time setup.
            t = time.time()
            t0 = t
            t_log = t

            step = self.time_step

            # while there are legs that haven't reached D yet.
            while count > 0.5:  # More than 0.5 just in case count = 0.0000000001 doesn't cause an infinite while loop.
                # Stop the robot is stop is 1.
                if self.stop == 1:
                    self.stop_all_legs()
                    break

                # For each leg.
                for i in range(self.n):
                    leg = self.legs[i]

                    # If the leg is calibrating:
                    if leg.mode == 10:
                        # Start the calibration.
                        # Calibration is a function that assigns minimum speeds to motors and returns True on a collision.
                        if leg.calibration():
                            count -= 1

                    # Once collided, the mode is changed from calibration to 'going to target' which is 2.
                    elif leg.mode == 2:
                        # At this point, no collision checks are made, because it is assumed that the leg will reach D without problems.
                        # In case something happens, it is up to the human to stop the robot (not the program).

                        # Advance the leg.
                        leg.set_new_speed(step)

                        # 1 second after total to stabilize.
                        if leg.t > leg.total+1:
                            print("Leg " + str(i + 1) + " reached D")

                            # Once D is reached, stop the leg, setup for phase 4, time 0.
                            self.stop_leg(i)
                            leg.setup_cycle()
                            count -= 1

                t += self.time_step

                # Log every 0.3 seconds.
                if t > t_log + 0.3:
                    self.log_file(t - t0)
                    t_log += 0.3

                # Sleep until next time step.
                tim = time.time()
                if t + self.time_step > tim:
                    step = self.time_step
                    t += self.time_step
                    time.sleep(t - tim)
                else:
                    step = tim - t
                    t = tim
            self.stop = 1
            finish()
            time.sleep(0.3)

        # If something happens during calibration, we want the user to know.
        except Exception as e:
            print(str(e))
            finish()
            error(str(e))


# Essential function that return a point on a line between A and B based on time passed and total time of travel.
# Operates on the principle that A*0.5 + B*0.5 gives the middle of the section.
def A_to_B(A, B, total, t):
    alpha = t / total
    x = B[0] * alpha + A[0] * (1 - alpha)
    y = B[1] * alpha + A[1] * (1 - alpha)
    return [x, y]


# Distance between 2 functions.
def dist(A, B):
    return ((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2) ** 0.5


# Class for the leg. Contains absolutely everything about the leg and much more.
class Leg:
    def __init__(self, id1, id2, coord, distance, V, R, time_step, max_length, port):

        # length_max - the max length that the points A and B can be from the motors.
        self.length_max = max_length

        # Current coordinates of the weight.
        self.now_coord = [0, 0]

        # Distance between the motors.
        self.b = distance

        # List of previous currents that is used for finding collisions.
        self.currents = []

        # Current phase the robot is in. 1-AB, 2-BC, 3-CD, 4-DA.
        self.phase = 1

        # Current time spent in this phase, or on the current section.
        self.t = 0

        # Coordinates of the ABCD
        self.coordinates = copy.deepcopy(coord)

        # Coordinates of the front motor and back motor.
        self.front_coord = [distance, 0]
        self.back_coord = [0, 0]

        # Time step.
        self.time_step = time_step

        # Direction of movement. 1 - forward, -1 - backwards.
        self.dir = 1

        # t_next is used after moving to a certain point on the cycle. Is used to remember the time of that point of the cycle.
        self.t_next = 0

        # Time for moving between calibration point and D.
        self.tod = 0

        # Time for AB, BC, CD, DA.
        self.tab = 0
        self.tbc = 0
        self.tcd = 0
        self.tda = 0

        # Speed of the whole robot.
        self.V = V

        # Speed limit of a motor.
        self.speed_limit = self.speed_limit = min(50, round(abs(self.V*500)))

        # Previous speeds for checking if we do not need to change speed on the motor.
        self.speed_front = 0
        self.speed_back = 0

        # Coefficients for BC line. Used to determine the B point during DA. (when finding bottom during forward movement)
        self.bc = 0
        self.x2 = 0
        self.y2 = 0

        # Coefficients for DA line. Used to determine the A point during CB. (when finding bottom during backwards movement)
        self.da = 0
        self.x1 = 0
        self.y1 = 0

        # Radius of the wheel.
        self.R = R

        # Setup all the phase times, coefficients and such.
        self.calculate_phases()

        # Angles of the wheels
        self.angle_front = 0
        self.angle_back = 0

        # Positions of motors.
        self.prev_front = 0
        self.prev_back = 0

        # The coordinates of the start of the section and the end. The leg will move from start to target.
        self.start = [0, 0]
        self.target = [0, 0]

        # Total is the time in which we move from start to target.
        self.total = 1

        # Mode of the leg. Mode 1 is cycle. Mode 2 is moving to some point on cycle. Mode 10 is calibration.
        self.mode = 0

        # Conversion coefficient for converting radian-per-second to the speed motors use.
        self.conversion_k = 60 / 2 / math.pi / 0.732

        # A and B coordinates used after finding the bottom.
        self.A = coord[0]
        self.B = coord[1]
        try:
            # Connect to front leg.
            instrument = mod.Instrument(port, id1)  # port name, slave address (in decimal)
            config(instrument)
            self.front = instrument
            self.front_id = id1

            # Connect to back leg.
            instrument = mod.Instrument(port, id2)
            config(instrument)
            self.back = instrument
            self.back_id = id2
        except Exception as e:
            print("Error! " + str(e))

    # Function that calculates everything based on the coordinates and speed.
    def calculate_phases(self):
        V = self.V

        # Save A and B into separate coordinates for output graphs.
        self.A = copy.deepcopy(self.coordinates[0])
        self.B = copy.deepcopy(self.coordinates[1])

        # If speed is 0, don't calculate times.
        if V != 0:
            # Calculate all times.
            self.tab = abs((self.coordinates[0][0] - self.coordinates[1][0]) / V)
            self.tbc = abs((self.coordinates[1][0] - self.coordinates[2][0]) / V)
            self.tda = abs((self.coordinates[3][0] - self.coordinates[0][0]) / V)
            self.tcd = abs(self.tab + self.tbc + self.tda)
            self.tod = abs((self.R - self.coordinates[3][1]) / 0.005)
        # Setup the direction.
        if self.V > 0:
            self.dir = 1
        elif self.V < 0:
            self.dir = -1
        # Calculate the speed limit for motors. Speed limit scales linearly. 5 for 0.01, 10 for 0.02 and so on.
        self.speed_limit = min(50, round(abs(self.V*500)))

        # Calculate the coefficients for DA and CB. Used for calculating x based on y, formula: (x-x2)/(x2-x1) = (y-y2)/(y2-y1)
        self.bc = (self.coordinates[2][0] - self.coordinates[1][0]) / (self.coordinates[2][1] - self.coordinates[1][1])
        self.y2 = self.coordinates[2][1]
        self.x2 = self.coordinates[2][0]
        self.da = (self.coordinates[3][0] - self.coordinates[0][0]) / (self.coordinates[3][1] - self.coordinates[0][1])
        self.y1 = self.coordinates[3][1]
        self.x1 = self.coordinates[3][0]

    # Change the speed of the leg.
    def change_speed(self, speed):
        # Only change if speed is not 0.
        if speed != 0:
            alpha = self.t / self.total
            self.V = speed

            # Recalculate phases.
            self.calculate_phases()
            self.set_phase_cycle()

            # Recalculate the current time.
            self.t = self.total * alpha

            # If the speed is opposite of previous speed, the current time should be calculated from the other end.
            if speed * self.dir < 0 and self.mode == 1:
                self.t = self.total - self.t

            # Setup the direction.
            if self.V > 0:
                self.dir = 1
            elif self.V < 0:
                self.dir = -1

        else:
            self.V = 0

    # Setup start and target based on the direction of movement and the phase.
    def set_phase_cycle(self):

        # For forward movement. A -> B -> C -> D.
        if self.V > 0:
            if self.phase == 1:
                self.start = self.coordinates[0]
                self.target = self.coordinates[1]
                self.total = self.tab
            elif self.phase == 2:
                self.start = self.coordinates[1]
                self.target = self.coordinates[2]
                self.total = self.tbc
            elif self.phase == 3:
                self.start = self.coordinates[2]
                self.target = self.coordinates[3]
                self.total = self.tcd
            elif self.phase == 4:
                self.start = copy.deepcopy(self.coordinates[3])
                self.target = copy.deepcopy(self.coordinates[0])
                self.total = self.tda

        # Backwards movement. A -> D -> C -> B.
        elif self.V < 0:
            if self.phase == 1:
                self.start = self.coordinates[1]
                self.target = self.coordinates[0]
                self.total = self.tab
            elif self.phase == 2:
                self.start = self.coordinates[2]
                self.target = self.coordinates[1]
                self.total = self.tbc
            elif self.phase == 3:
                self.start = self.coordinates[3]
                self.target = self.coordinates[2]
                self.total = self.tcd
            elif self.phase == 4:
                self.start = copy.deepcopy(self.coordinates[0])
                self.target = copy.deepcopy(self.coordinates[3])
                self.total = self.tda

    # Returns current coordinate. Also progresses the phases and checks the bottom during downward phases.
    def get_current_coord(self):

        # If the leg is in a cycle.
        if self.mode == 1:

            # Forward movement.
            if self.V > 0:

                # When we go over limit in non-downward phases, go to the next phase immediately. DA - 4 is the downward phase.
                if self.t > self.total and self.phase != 4:
                    self.phase += 1
                    self.t -= self.total
                    self.set_phase_cycle()  # Sets up new target and start.

                # When we are in a downward phase.
                elif self.phase == 4:
                    coord = A_to_B(self.start, self.target, self.total, self.t)  # Get current coordinate.
                    # Since we do not stop incrementing time when we reach time limit, this function will keep returning coordinates over B.

                    # A' is the current coordinate, B' is on BC with B'y=A'y.
                    self.A = copy.deepcopy(coord)
                    self.B = [(self.A[1] - self.y2) * self.bc + self.x2, self.A[1]]

                    # If leg reaches max length of the thread, detects bottom or A is close to B:
                    if self.check_max_length() or self.detect_bottom() or abs(self.A[0] - self.B[0]) < 0.02:
                        print("New A and B: [%0.2f, %0.2f], [%0.2f, %0.2f]" % (self.A[0], self.A[1], self.B[0], self.B[1]))

                        # We lock A and B.
                        self.coordinates[0] = copy.deepcopy(self.A)  # A = A'
                        self.coordinates[1] = copy.deepcopy(self.B)  # B = B'

                        # Advance the phase
                        self.phase = 1
                        self.t = 0

                        # Recalculate based on new coordinates.
                        self.calculate_phases()

                        # Setup next section of movement.
                        self.set_phase_cycle()
                    self.now_coord = coord
                    return coord

            # Backwards movement.
            else:
                # When we go over limit in non-downward phases, go to the next phase immediately. CB - 2 is the downward phase.
                if self.t > self.total and self.phase != 2:
                    self.phase = self.phase - 1
                    if self.phase == 0:
                        self.phase = 4
                    self.t -= self.total
                    self.set_phase_cycle()  # Sets up new target and start.

                elif self.phase == 2:  # When we are on CB:
                    coord = A_to_B(self.start, self.target, self.total, self.t)  # Get current coordinate.

                    # B' is the current coordinate,A' is on DA with B'y=A'y.
                    self.B = copy.deepcopy(coord)
                    self.A = [(self.B[1] - self.y1) * self.da + self.x1, self.B[1]]

                    # If leg reaches max length of the thread, detects bottom or A is close to B:
                    if self.check_max_length() or self.detect_bottom() or abs(self.A[0] - self.B[0]) < 0.02:
                        print("New A and B: [%0.2f, %0.2f], [%0.2f, %0.2f]" % (self.A[0], self.A[1], self.B[0], self.B[1]))

                        # We lock A and B.
                        self.coordinates[0] = copy.deepcopy(self.A)
                        self.coordinates[1] = copy.deepcopy(self.B)

                        # Advance the phase
                        self.phase = 1
                        self.t = 0

                        self.calculate_phases()   # Recalculate based on new coordinates.
                        self.set_phase_cycle()    # Setup next section of movement.
                    self.now_coord = coord
                    return coord
        else:
            # When we are not in cycle, going over time limit will just return the goal coordinate.
            if self.t > self.total:
                self.now_coord = self.target
                return self.target

        # Generic return, when not in downward phase on cycle, or when out of cycle bellow limit - just return the intermediate coordinate.
        self.now_coord = A_to_B(self.start, self.target, self.total, self.t)
        return self.now_coord

    # Check if we go over limit length. Return True if we are over limit.
    def check_max_length(self):
        l1 = dist(self.A, self.back_coord)
        l2 = dist(self.B, self.front_coord)
        return l1 > self.length_max or l2 > self.length_max

    # A function with a filter on currents. If currents spike up a little, the leg hits the bottom. The filter is very sensitive, because even on faster speeds like 0.03m/s, the current spikes by ~1.2.
    # On slower than 0.03 m/s, it is practically impossible to detect bottom.
    def detect_bottom(self):
        if len(self.currents) == 7:
            
            old_c = sum(self.currents[0:5]) / 5  # Average of 3rd to 7th currents.

            now_c = sum(self.currents[5:7]) / 2  # Average of last 2 currents.

            # The actual condition for detecting bottom. Usually, when hitting bottom, currents spike to ~6-8 for 0.03m/s. Could be different in water.
            # ow_c >= abs(self.V*425) is a scaling filter which depends on current speed. For 0.01m/s, we look for a 5-7 spike.
            # The scaling can be changed depending on practical results.

            # self.t > 0.06/abs(self.V) is a condition to give the motor some time to stabilize during DA and CB.
            # It always gives the same amount of distance for the motor. 1 cm/s - 6 seconds. 2cm/s - 3 seconds. 3cm/s - 2 seconds.
            # Caution: Changing speeds destabilizes the motors and the current will spike. Might need to add some sort of condition based on changing speeds.

            # now_c - old_c is a simple non-absolute filter. Only detect bottom when currents actually spike.

            # Filter for 3 cm/s is different.
            if abs(self.V) < 0.025:
                if now_c - old_c > abs(self.V*110) and now_c >= abs(self.V*425) and self.t > 0.06 / abs(self.V):
                    # When bottom is detected, return True.
                    print("Leg [%d, %d] bottom detected." % (self.front_id, self.back_id))
                    return True
            else:
                if now_c - old_c > 5 and now_c >= 20 and self.t > 0.06 / abs(self.V):
                    # When bottom is detected, return True.
                    print("Leg [%d, %d] bottom detected." % (self.front_id, self.back_id))
                    return True
        # Return false everytime we don't find bottom.
        return False

    # A function to detect when everything goes bad. Also used for calibration. If you can come up with a better calibration than slamming the weight up to the motors,
    # you are free to implement that. Don't forget to save this code just in case.
    def detect_collision(self):
        # Uses the same principles as detect_bottom, except the fact that filters are a lot less sensitive.

        # Takes the average of currents on both motors.
        curr1 = self.front.read_register(263, 0, 3, signed=True)
        curr2 = self.back.read_register(263, 0, 3, signed=True)
        av = (curr1 + curr2) / 2

        # Add it to the currents list.
        self.currents.append(av)

        # When collision currents reached 8 elements:
        if len(self.currents) == 8:

            # This filter does not really need scaling, because when stuff goes bad, it goes really bad. Currents can spike to around 60-70 when motors start pulling on each other.
            # Making this filter less sensitive will result in calibration being a bit more harmful for the legs (not the motors themselves, they will be fine).
            # Also note that self.V is not setup properly during calibration, so using scaling based on speed is not good.
            if av > 55:

                # Delete the first element and return True.
                self.currents = self.currents[1:]
                print("Leg [%d, %d] collision detected." % (self.front_id, self.back_id))
                return True

            # Delete the first element and return false.
            self.currents = self.currents[1:]
        return False

    # Uses formulas to calculate absolute angles for each motor based on x, y of the weight. Don't ask me how it works, it just does.
    def calculate_angle(self, x, y):
        R = self.R
        d1 = ((x - self.back_coord[0]) ** 2 + (y - self.back_coord[1]) ** 2) ** 0.5
        d2 = ((x - self.front_coord[0]) ** 2 + (y - self.front_coord[1]) ** 2) ** 0.5
        if abs(x - self.back_coord[0]) == 0:
            b1 = math.pi / 2
        else:
            b1 = math.atan(abs(y - self.back_coord[1]) / abs(x - self.back_coord[0]))
        g1 = math.acos(R / d1)
        sig1 = math.pi - b1 - g1
        if abs(x - self.front_coord[0]) == 0:
            b2 = math.pi / 2
        else:
            b2 = math.atan(abs(y - self.front_coord[1]) / abs(x - self.front_coord[0]))
        g2 = math.acos(R / d2)
        sig2 = math.pi - b2 - g2
        angle1 = sig1 + (d1 ** 2 - R ** 2) ** 0.5 / R
        angle2 = sig2 + (d2 ** 2 - R ** 2) ** 0.5 / R
        return angle1, angle2

    # Just initializes the coordinates, angles and positions of motors into the robot based on provided coordinates.
    # I think its only used during calibration.
    def initial(self, x, y):
        self.now_coord = [x, y]                                               # Setup coordinates.
        self.angle_back, self.angle_front = self.calculate_angle(x, y)        # Setup angles.
        self.prev_front = self.front.read_register(257, 0, 3, signed=False)   # Front motor position
        self.prev_back = self.back.read_register(257, 0, 3, signed=False)     # Back motor position.

    # Set a target in specified coordinates.
    def set_new_target(self, coord):
        self.start = self.now_coord  # Start is the current coordinate.
        self.target = coord          # Target is the specified coordinate.
        self.t = 0                   # Time is 0 (we are at the start of the section).
        self.total = dist(self.start, self.target) / 0.015  # Default speed of movement - 1.5 cm/s.

    # An expansion upon above function. Sets a target on the cycle with specified phase and percentage across the section of said phase.
    def set_target_on_cycle(self, phase, alpha):

        # If specified percentage is 1, it basically means the start of next phase.
        if alpha == 1:
            phase = phase % 4 + 1  # A universal function of advancing the phase. 1 -> 2 -> 3 -> 4 -> 1
            alpha = 0

        # Get the total time of the targeted phase.
        if phase == 1:
            total = self.tab
        elif phase == 2:
            total = self.tbc
        elif phase == 3:
            total = self.tcd
        elif phase == 4:
            total = self.tda
        else:
            print("Error! incorrect phase" + str(phase))
            return 0

        # Get the coordinate of the target. The coordinate is between 2 points on the cycle (which are stored in self.coordinates).
        # And the time for that coordinate in this section is simply (total time) * (percentage).
        coord = A_to_B(self.coordinates[phase - 1], self.coordinates[phase % 4], total, total * alpha)

        # Set up the target, start, t and total. They are completely different from what will be used later in the cycle since these are only for moving to the cycle.
        self.set_new_target(coord)

        # Because of that, we need to remember the phase and time for that phase after we reach the specified point on the cycle.
        # To remember time we need to use another variable to store the cycle time, while we are moving to the cycle.
        self.t_next = total * alpha

        # Phase does not change during mode 2, so we can just keep it like this.
        self.phase = phase

        self.mode = 2

    # Used after mode 2, to retrieve the time we saved in t_next. Also sets up the start and target in the cycle after we reached it.
    def setup_cycle(self):
        self.t = self.t_next
        self.mode = 1
        self.set_phase_cycle()

    # Call the stop functions fast. Mainly used when many legs are expected to stop at difference points in time (calibration, before_start_init - mode 2)
    def fast_stop(self):
        self.set_speed_0()
        self.change_mode_0()

    # Sets the speeds to 0 on both motors.
    def set_speed_0(self):
        self.front.write_register(131, 0, 0, functioncode=6, signed=False)
        self.back.write_register(131, 0, 0, functioncode=6, signed=False)

    # Changes the work mode to 0 (different from self.mode). Meaning, it will stabilize using the position and not speed.
    def change_mode_0(self):
        self.front.write_register(16, 0, 0, functioncode=6, signed=False)
        self.back.write_register(16, 0, 0, functioncode=6, signed=False)

        # Also turns on torque to hold the motors in place. Without torque, the motors will release the weight down.
        self.front.write_register(129, 1, 0, functioncode=6, signed=False)
        self.back.write_register(129, 1, 0, functioncode=6, signed=False)

    # Changes the work mode of motors back to 1. Torque is still turned on, once again, without it, motors simply cannot hold the weight.
    def change_mode_1(self):
        self.front.write_register(16, 1, 0, functioncode=6, signed=False)
        self.back.write_register(16, 1, 0, functioncode=6, signed=False)
        self.front.write_register(129, 1, 0, functioncode=6, signed=False)
        self.back.write_register(129, 1, 0, functioncode=6, signed=False)


# Class for the left leg. Differences with RightLeg class is minimal so I will not comment the RightLeg class.
class LeftLeg(Leg):
    def __init__(self, id1, id2, coord, distance, V, R, time_step, max_l, port):
        super().__init__(id1, id2, coord, distance, V, R, time_step, max_l, port)

        # The coordinates of the front and back motors are different depending on the side.
        # When looking at the motor from the outside, the left one is always [0, 0] and the right one is always [distance, 0].
        # But front and back motors are reversed for left and right side.
        self.front_coord = [0, 0]
        self.back_coord = [distance, 0]

        # Since ABCD is expected to be given in coordinates for the right side, left side legs reverse the coordinates around the middle point between 2 motors.
        # If A was [0.20, 0.20] and B was [0.04, 0.20], then for the left side (since the front is on the left) A will be [0.04, 0.20] and B - [0.20, 0.20].
        for point in self.coordinates:
            point[0] = distance - point[0]

        # Simply initialize as if we are in A. If you do not calibrate legs or load the information onto legs, the legs will think they are in A.
        self.initial(self.coordinates[0][0], self.coordinates[0][1])

        # Calculate phases again, since we changed the coordinates.
        self.calculate_phases()

    # Calibration function for legs. Assumed that the weight is exactly between both motors before the start of calibration.
    def calibration(self):

        # Once we detect collision:
        if self.detect_collision():
            # A simple fast stop without wait. If we wait, we might break something.
            self.set_speed_0()

            # Initialize before we change the mode.
            self.initial(self.b / 2, self.R + self.b / 15)
            self.change_mode_0()
            self.change_mode_1()

            # After stopping, we set a new target to D.
            self.set_target_on_cycle(4, 0)
            return True

        # Otherwise, just set minimal speeds and wait for collision.
        self.front.write_register(131, 1, 0, functioncode=6, signed=True)
        self.back.write_register(131, -1, 0, functioncode=6, signed=True)
        return False

    # A function that detects the difference in position on motors and changes actual angles depending on this.
    def get_current_angle(self):
        # Get the position.
        positionF = self.front.read_register(257, 0, 3, signed=False)
        positionB = self.back.read_register(257, 0, 3, signed=False)

        # Update the angles of the motors depending on the change in position.
        self.angle_front = self.angle_front - diff(positionF, self.prev_front) / 4096 * 2 * math.pi
        self.angle_back = self.angle_back + diff(positionB, self.prev_back) / 4096 * 2 * math.pi

        # Remember the current position for the next call of this function.
        self.prev_front = positionF
        self.prev_back = positionB

        return self.angle_front, self.angle_back

    # A universal function for setting speed to motors. Takes the coordinate where we want to be at this moment and tries to go there.
    def set_new_speed(self, time_diff):
        # Get the coordinate before the time step.
        coord_0 = self.get_current_coord()

        # Increment time every time this function is called. Time increment differs, so the robot specifies it.
        self.t += time_diff

        # Only do something when the speed is not 0.
        if not (self.mode == 1 and self.V == 0):
            # Get the coordinate we want to be at using get_current_coord()
            coord = self.get_current_coord()

            # Get the angles of motors we want to have based on wanted coordinates.
            angleB, angleF = self.calculate_angle(coord[0], coord[1])
            angleB_0, angleF_0 = self.calculate_angle(coord_0[0], coord_0[1])

            # Get the actual angles of motors.
            angleF_now, angleB_now = self.get_current_angle()

            # Calculate speed based on the difference in angles.
            # First take the actual difference.
            speedF_a = (angleF_now - angleF) / time_diff * self.conversion_k
            speedB_a = (angleB - angleB_now) / time_diff * self.conversion_k

            # Then the take the theoretical speed of motors.
            speedF_t = (angleF_0 - angleF) / time_diff * self.conversion_k
            speedB_t = (angleB - angleB_0) / time_diff * self.conversion_k

            # Add them upp with weights making the new speed.
            speedF = round(speedF_a * 0.35 + speedF_t * 0.65)
            speedB = round(speedB_a * 0.35 + speedB_t * 0.65)

            # Limit the speed of motors, just so they don't go crazy.
            if speedF > self.speed_limit:
                speedF = self.speed_limit
            elif speedF < -self.speed_limit:
                speedF = -self.speed_limit
            if speedB > self.speed_limit:
                speedB = self.speed_limit
            elif speedB < -self.speed_limit:
                speedB = -self.speed_limit

            # Write the speed into the motors if the speed is different from previous speed.
            if speedF != self.speed_front:
                self.front.write_register(131, speedF, 0, functioncode=6, signed=True)
            if speedB != self.speed_back:
                self.back.write_register(131, speedB, 0, functioncode=6, signed=True)
            self.speed_front = speedF
            self.speed_back = speedB


class RightLeg(Leg):
    def __init__(self, id1, id2, coord, distance, V, R, time_step, max_l, port):
        super().__init__(id1, id2, coord, distance, V, R, time_step, max_l, port)
        self.front_coord = [distance, 0]
        self.back_coord = [0, 0]
        self.initial(self.coordinates[0][0], self.coordinates[0][1])

    def calibration(self):
        if self.detect_collision():
            self.front.write_register(131, 0, 0, functioncode=6, signed=False)
            self.back.write_register(131, 0, 0, functioncode=6, signed=False)
            self.change_mode_0()
            self.change_mode_1()
            self.initial(self.b / 2, self.R + self.b / 15)
            self.mode = 2
            self.set_target_on_cycle(4, 0)
            return True
        self.front.write_register(131, -1, 0, functioncode=6, signed=True)
        self.back.write_register(131, 1, 0, functioncode=6, signed=True)
        return False

    def get_current_angle(self):
        positionF = self.front.read_register(257, 0, 3, signed=False)
        positionB = self.back.read_register(257, 0, 3, signed=False)
        self.angle_front = self.angle_front + diff(positionF, self.prev_front) / 4096 * 2 * math.pi
        self.angle_back = self.angle_back - diff(positionB, self.prev_back) / 4096 * 2 * math.pi
        self.prev_front = positionF
        self.prev_back = positionB
        return self.angle_front, self.angle_back

    def set_new_speed(self, time_diff):
        coord_0 = self.get_current_coord()
        self.t += time_diff
        if not (self.mode == 1 and self.V == 0):
            coord = self.get_current_coord()
            angleB, angleF = self.calculate_angle(coord[0], coord[1])
            angleB_0, angleF_0 = self.calculate_angle(coord_0[0], coord_0[1])
            angleF_now, angleB_now = self.get_current_angle()

            speedF_a = (angleF - angleF_now) / time_diff * self.conversion_k
            speedB_a = (angleB_now - angleB) / time_diff * self.conversion_k
            speedF_t = (angleF - angleF_0) / time_diff * self.conversion_k
            speedB_t = (angleB_0 - angleB) / time_diff * self.conversion_k
            speedF = round(speedF_a * 0.35 + speedF_t * 0.65)
            speedB = round(speedB_a * 0.35 + speedB_t * 0.65)

            if speedF > self.speed_limit:
                speedF = self.speed_limit
            elif speedF < -self.speed_limit:
                speedF = -self.speed_limit
            if speedB > self.speed_limit:
                speedB = self.speed_limit
            elif speedB < -self.speed_limit:
                speedB = -self.speed_limit

            if speedF != self.speed_front:
                self.front.write_register(131, speedF, 0, functioncode=6, signed=True)
            if speedB != self.speed_back:
                self.back.write_register(131, speedB, 0, functioncode=6, signed=True)
            self.speed_front = speedF
            self.speed_back = speedB
