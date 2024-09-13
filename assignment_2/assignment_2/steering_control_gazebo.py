import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib as mpl

mpl.rcParams['font.size'] = 12  # Set default font size for all text
mpl.rcParams['axes.labelsize'] = 12  # Set default font size for axis labels
mpl.rcParams['xtick.labelsize'] = 12  # Set default font size for x-axis tick labels
mpl.rcParams['ytick.labelsize'] = 12  # Set default font size for y-axis tick labels
mpl.rcParams['legend.fontsize'] = 16  # Set default font size for legend

class SteeringControlNode(Node):
    def __init__(self):
        super().__init__('steering_control_node')

        # Initialize robot state variables
        self.R = 5.0  # Initial distance from origin
        self.theta = np.pi / 4  # Initial orientation angle (radians)
        self.alpha = 2*np.pi / 4  # Initial angle with respect to X-axis (radians)
        self.v = 0.5  # Constant linear velocity (m/s)

        # Control gain
        self.Ks = 10.0  # Proportional gain for angular velocity (steering)

        # Simulation parameters
        self.dt = 0.01  # Time step (s)
        self.total_time = 100.0  # Total simulation time (s)

        # Stopping threshold (distance from origin)
        self.x_target = 0.0
        self.y_target = 0.0
        self.stopping_threshold = (self.x_target**2 + self.y_target**2)**0.5 # Stop when R = 0 meters

        # Data for plotting
        self.R_history = []
        self.theta_history = []
        self.alpha_history = []
        self.time_history = []

        # Track X and Y positions for trajectory plot
        self.x_history = []
        self.y_history = []
        
        # Publisher to send velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer to call control loop
        self.control_timer = self.create_timer(0.1, self.simulate)

        # Subscriber to robot state (odometry)
        self.create_subscription(Odometry, '/odom', self.update_state, 10)
        
    def update_state(self, msg):
        """
        Callback function to update the robot's state based on odometry data.
        Odometry provides the robot's position (x, y) and orientation (quaternion).
        """

        # Extract robot's position (x, y) from odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Calculate R, the distance from the origin (home position)
        self.R = math.sqrt(x**2 + y**2)

        # Extract the orientation of the robot in quaternions and convert to Euler (yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)  # This is the robot's yaw (theta)

        # For alpha, we assume the velocity vector is along the x-axis of the robot
        # We need to compute alpha based on the velocity vector direction (assumed initially along x-axis)
        # In this example, we assume the velocity direction is along the direction of motion
        velocity = msg.twist.twist.linear.x
        angular_velocity = msg.twist.twist.angular.z

        # If the robot is moving, calculate alpha based on the velocity direction
        if velocity != 0:
            self.alpha = math.atan2(y, x)  # Angle relative to the X axis

        # Print the updated state for debugging purposes
        self.get_logger().info(f"Updated state -> R: {self.R}, theta: {self.theta}, alpha: {self.alpha}")

    def simulate(self):
        t = 0.0
        while t < self.total_time:
            # Apply the proportional steering control law
            angular_error = np.sign(self.alpha - self.theta - np.pi)
            omega = -self.Ks * angular_error  # Proportional control

            # Update the states using kinematic equations
            self.R += self.v * np.cos(self.alpha - self.theta) * self.dt
            self.theta += (self.v / self.R) * np.sin(self.alpha - self.theta) * self.dt
            self.alpha += omega * self.dt

            # Calculate and store X, Y positions for trajectory
            x = self.R * np.cos(self.theta)
            y = self.R * np.sin(self.theta)
            self.x_history.append(x)
            self.y_history.append(y)
            
            # Publish velocity command
            twist = Twist()
            twist.linear.x = self.v
            twist.angular.z = omega
            self.velocity_publisher.publish(twist)

            # Print current state for debugging
            self.get_logger().info(f"R: {self.R}, theta: {self.theta}, alpha: {self.alpha}")

            # Store data for visualization
            self.R_history.append(self.R)
            self.theta_history.append(self.theta)
            self.alpha_history.append(self.alpha)
            self.time_history.append(t)

            # Time increment
            t += self.dt

            # Stop the simulation if the robot is close enough to the origin
            if self.R <= self.stopping_threshold:
                print(f"Robot reached the origin within {self.stopping_threshold} meters at time {t:.2f} seconds.")
                break

        # Plot the results after simulation
        self.plot_results()

    def plot_results(self):
        time = self.time_history

        plt.figure()

        # Plot R (distance from origin)
        plt.subplot(3, 1, 1)
        plt.plot(time, self.R_history)
        plt.title('Distance R(t)')
        plt.xlabel('Time [s]', labelpad=3)
        plt.ylabel('R [m]')

        # Plot theta (orientation angle)
        plt.subplot(3, 1, 2)
        plt.plot(time, self.theta_history)
        plt.title('Orientation Theta(t)')
        plt.xlabel('Time [s]', labelpad=3)
        plt.ylabel('Theta [rad]')

        # Plot alpha (velocity angle)
        plt.subplot(3, 1, 3)
        plt.plot(time, self.alpha_history)
        plt.title('Angle Alpha(t)')
        plt.xlabel('Time [s]', labelpad=3)
        plt.ylabel('Alpha [rad]')

        # Create a new figure for trajectory plot
        plt.figure()
        plt.plot(self.x_history, self.y_history, label='Robot Trajectory')
        plt.scatter(self.x_history[0], self.y_history[0], color='red', label='Start', zorder=5)
        plt.scatter(self.x_history[-1], self.y_history[-1], color='green', label='End', zorder=5)
        plt.title('Robot Trajectory on XY Plane')
        plt.xlabel('X Position [m]')
        plt.ylabel('Y Position [m]')
        plt.grid(True)
        plt.legend()

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = SteeringControlNode()
    rclpy.spin(node)
    node.simulate()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
