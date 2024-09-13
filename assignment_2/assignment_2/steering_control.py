import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

class SteeringControlNode(Node):
    def __init__(self):
        super().__init__('steering_control_node')

        # Initialize robot state variables
        self.R = 1.0  # Initial distance from origin
        self.theta = np.pi / 4  # Initial orientation angle (radians)
        self.alpha = np.pi / 4  # Initial angle with respect to X-axis (radians)
        self.v = 0.5  # Constant linear velocity (m/s)

        # Control gain
        self.Ks = 5.0  # Proportional gain for angular velocity (steering)

        # Simulation parameters
        self.dt = 0.001  # Time step (s)
        self.total_time = 20.0  # Total simulation time (s)

        # Stopping threshold (distance from origin)
        self.x_target = 0
        self.y_target = 0
        self.stopping_threshold = (self.x_target**2 + self.y_target**2)**0.5  # Stop when R < 0.01 meters
        
        # Data for plotting
        self.R_history = []
        self.theta_history = []
        self.alpha_history = []
        self.time_history = []

        # Track X and Y positions for trajectory plot
        self.x_history = []
        self.y_history = []

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
        plt.xlabel('Time [s]')
        plt.ylabel('R [m]')

        # Plot theta (orientation angle)
        plt.subplot(3, 1, 2)
        plt.plot(time, self.theta_history)
        plt.title('Orientation Theta(t)')
        plt.xlabel('Time [s]')
        plt.ylabel('Theta [rad]')

        # Plot alpha (velocity angle)
        plt.subplot(3, 1, 3)
        plt.plot(time, self.alpha_history)
        plt.title('Angle Alpha(t)')
        plt.xlabel('Time [s]')
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
    node.simulate()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
