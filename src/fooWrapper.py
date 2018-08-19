import ctypes
import math
import numpy as np
import matplotlib.pyplot as plt
import random

lib = ctypes.cdll.LoadLibrary('./libpid.so')


# Sheesh. Probably over-egging the soufflé here, but at least
# now I know how to wire up C++ code to Python code. Yet I
# hear that SWIG produces faster code... hmm.

# For future reference, when writing C++ <-> Python it is probably
# best to have Python be in charge of owning objects, so C++ doesn't
# have to deal with deleting them afterwards. In this case, since I'm
# using a C++ object (the PID object), this is unavoidable.
class PID(object):

    def __init__(self, Kp=0.0, Kd=0.0, Ki=0.0):
        self.params_size = 3

        # new()
        lib.PID_new.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double]
        lib.PID_new.restype = ctypes.c_void_p

        # initialise()
        lib.PID_Init.argtypes = [ctypes.c_void_p, ctypes.c_double]
        lib.PID_Init.restype = ctypes.c_void_p

        # set_params()
        lib.PID_SetParams.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_double]
        lib.PID_SetParams.restype = ctypes.c_void_p

        # update_params()
        lib.PID_UpdateParams.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_double]
        lib.PID_UpdateParams.restype = ctypes.c_void_p

        # reset_params()
        lib.PID_ResetParams.argtypes = [ctypes.c_void_p]
        lib.PID_ResetParams.restype = ctypes.c_void_p

        # print_params()
        lib.PID_PrintParams.argtypes = [ctypes.c_void_p]
        lib.PID_PrintParams.restype = ctypes.c_void_p

        # reset_errors()
        lib.PID_ResetErrors.argtypes = [ctypes.c_void_p]
        lib.PID_ResetErrors.restype = ctypes.c_void_p

        # respond()
        lib.PID_Respond.argtypes = [ctypes.c_void_p, ctypes.c_double]
        lib.PID_Respond.restype = ctypes.c_double

        # record_total_error()
        lib.PID_RecordTotalError.argtypes = [ctypes.c_void_p]
        lib.PID_RecordTotalError.restype = ctypes.c_void_p

        # get_total_error()
        lib.PID_GetTotalError.argtypes = [ctypes.c_void_p]
        lib.PID_GetTotalError.restype = ctypes.c_double

        # delete_self()
        lib.PID_DeleteSelf.argtypes = [ctypes.c_void_p]
        lib.PID_DeleteSelf.restype = ctypes.c_void_p

        self.obj = lib.PID_new(Kp, Kd, Ki)

    def initialise(self, init_CTE):
        lib.PID_Init(self.obj, init_CTE)

    def set_params(self, index, value):
        lib.PID_SetParams(self.obj, index, value)

    def update_params(self, index, value):
        lib.PID_UpdateParams(self.obj, index, value)

    def reset_params(self):
        lib.PID_ResetParams(self.obj)

    def print_params(self):
        lib.PID_PrintParams(self.obj)

    def reset_errors(self):
        lib.PID_ResetErrors(self.obj)

    def respond(self, CTE):
        return lib.PID_Respond(self.obj, CTE)

    def record_total_error(self):
        lib.PID_RecordTotalError(self.obj)

    def get_total_error(self):
        return lib.PID_GetTotalError(self.obj)

    def delete_self(self):
        lib.PID_DeleteSelf(self.obj)


class Robot(object):
    def __init__(self, length=20.0):
        """
        Creates robot and initializes location/orientation to 0, 0, 0.
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0
        self.clock = 0

    def set(self, x, y, orientation):
        """
        Sets a robot coordinate.
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)  # Guessing this is in radians, then.

    def set_noise(self, steering_noise, distance_noise):
        """
        Sets the noise parameters.
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 3.0):  # Let's call the max steering pi/3, which is roughly 1 radian
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift

        # Execute motion
        turn = np.tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def get_CTE(self, update_clock=True):
        res = self.CTE_function(self.x, self.y, self.clock, 0)
        if update_clock:
            self.clock += 1
        return res

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)


def euclidean(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def make_robot(x=0, y=0, heading=0):
    """
    Resets the robot back to the initial position and drift.
    You'll want to call this after you call `run`.
    """
    robot = Robot()
    robot.set(x, y, heading)
    return robot


def make_robot_linear(x=0, y=0, heading=0):
    robot = Robot()
    robot.set(x, y, heading)
    robot.CTE_function = lambda x, y, x_prime, y_prime: y
    return robot


def make_robot_circle(x=0, y=0, heading=0, centre=(0, 0), radius=1.0):
    robot = Robot()
    robot.set(x, y, heading)
    robot.CTE_function = lambda x, y, x_prime, y_prime: -(euclidean(x, y, centre[0], centre[1]) - radius)
    return robot


def make_robot_sinusoid(x=0, y=0, heading=0, A=1, B=1, C=0, D=0):
    robot = Robot()
    robot.set(x, y, heading)
    robot.clock = 0
    robot.CTE_function = lambda x, y, x_prime, y_prime: (euclidean(x, y, x_prime, (A * np.sin(B * (x - C)) + D)))  # An approximation - euclidean distance between robot and expected position at timestep t
    return robot


# Want to make twiddle() generic to the type of scenario we are using.

# What needs to stay constant? Well we may as well hold our controller and params as constant
# since you always need those three params (Kp, Kd, Ki). Likewise dp.

# The robot should treated as a parameter - avoid the scenario logic affecting
# twiddle(). The logic of the scenario would be better placed in the robot object.
# So the robot contains the logic for acquiring CTE (which makes sense since it
# would need to acquire it via its sensors anyway).

# Pass in a constructor function to substitute for the calls to make_robot(...).
def twiddle(controller, constructor, tol=0.2, num_iters=100):
    print("Starting in twiddle()...")
    pid.print_params()
    controller.reset_errors()  # p, d, and i errors set to 0
    n = controller.params_size
    dp = [1 for i in range(n)]
    robot = constructor()
    _, _, _, _, best_err = run(robot, controller, num_iters=num_iters, optimising=True)
    controller.reset_errors()  # Avoid errors propogating
    print("best_err: ", best_err)

    it = 0
    while sum(dp) > tol or it < 1:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(n):
            controller.update_params(i, dp[i])  # controller.params[i] += dp[i]
            robot = constructor()
            _, _, _, _, err = run(robot, controller, num_iters=num_iters, optimising=True)
            controller.reset_errors()
            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                controller.update_params(i, -2 * dp[i])  # controller.params[i] -= 2 * dp[i]
                robot = constructor()
                _, _, _, _, err = run(robot, controller, num_iters=num_iters, optimising=True)
                controller.reset_errors()
                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    controller.update_params(i, dp[i])  # controller.params[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return controller, best_err


# calculating terms is done by PID controller, passed as an argument.
# So twiddle() should manipulate a controller, and implicitly its coefficients, not the coefficients directly.
def run(robot, controller, num_iters=1000, optimising=False, speed=1.0):
    x_trajectory, y_trajectory = [], []
    CTE_inputs, alpha_outputs = [], []

    controller.initialise(robot.get_CTE(update_clock=False))  # initialise using first CTE reading

    n = 2 * num_iters if optimising else num_iters

    for i in range(n):
        CTE = robot.get_CTE()
        print("CTE:", CTE)
        alpha = controller.respond(CTE)
        if not optimising:
            print("CTE:", CTE)
            pass  # print("alpha:", alpha)
        robot.move(alpha, speed)

        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        CTE_inputs.append(CTE)
        alpha_outputs.append(alpha)

        if optimising and i == int(n/2):  # Start recording from halfway point.
            controller.record_total_error()  # in this case, CTE^2, and includes the current CTE

    # print("total_error:", controller.get_total_error())

    mean_sq_err = controller.get_total_error() / n

    return x_trajectory, y_trajectory, CTE_inputs, alpha_outputs, mean_sq_err


num_iters = 100

robot_line_func = lambda: make_robot_linear(x=-30, y=30)

circle_centre = (0, 0)
circle_radius = 50  # larger circle
robot_circle_func = lambda: make_robot_circle(x=circle_radius*1.5, y=0, heading=np.pi / 1, centre=circle_centre, radius=circle_radius)

a = 1
b = 1 / 32
c = 0
d = 0
robot_sin_func = lambda: make_robot_sinusoid(x=0, y=0, heading=0, A=a, B=b, C=c, D=d)

pid = PID(0.0, 0.0, 0.0)


def train_and_run(controller, scenario="line", num_iters=100):
    if scenario == "line":
        func = robot_line_func
        reference_x_trajectory, reference_y_trajectory = [i for i in range(num_iters)], [0 for i in range(num_iters)]
    elif scenario == "circle":
        func = robot_circle_func
        t = np.linspace(0, np.pi*2, 100)
        reference_x_trajectory, reference_y_trajectory = circle_radius * np.cos(t), circle_radius * np.sin(t)
    elif scenario == "sinusoid":
        func = robot_sin_func
        reference_x_trajectory, reference_y_trajectory = [i for i in range(num_iters)], [(a * np.sin(b * (x - c)) + d) for x in range(num_iters)]

    controller, best_err = twiddle(controller, func, num_iters=num_iters)

    robot = func()
    controller.update_params(0, 1)
    controller.update_params(0, 0)
    controller.update_params(0, 0.0)

    x_trajectory, y_trajectory, CTE_inputs, alpha_outputs, mean_sq_err = run(robot, controller, num_iters=num_iters, optimising=True)

    return x_trajectory, y_trajectory, CTE_inputs, alpha_outputs, mean_sq_err, reference_x_trajectory, reference_y_trajectory


x_trajectory, y_trajectory, CTE_inputs, alpha_outputs, \
    mean_sq_err, reference_x_trajectory, reference_y_trajectory = train_and_run(pid, scenario="line", num_iters=num_iters)

print("mean_sq_err:", mean_sq_err)

## Plotting

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))

ax1.plot(x_trajectory, y_trajectory, 'go', label="Robot Trajectory")
ax1.plot(reference_x_trajectory, reference_y_trajectory, 'r', label="Reference Trajectory")

ax2.plot([x for x in range(2*num_iters)], CTE_inputs, "red", label="CTE over time")
ax2.plot([x for x in range(2*num_iters)], alpha_outputs, "green", label="Alpha over time")

plt.legend()
plt.show()

pid.print_params()
pid.delete_self()


# End of file
