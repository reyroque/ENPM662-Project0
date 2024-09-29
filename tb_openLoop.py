import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


# Scenario 1: The TurtleBot moved at a constant velocity to reach a specific coordinate along
# a straight line, following the equation x=v⋅t, where x represents the displacement, v is the constant
# velocity, and t is the time.
class OpenVelocityController(Node):
    def __init__(self):
        # Initialize the node with the name 'tb_openLoop'
        super().__init__("tb_openLoop")

        # Create a publisher for the '/cmd_vel' topic
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

        # Control loop: Publish velocity commands every 0.1 seconds
        self.timer = self.create_timer(0.1, self.control_loop)

        # Displacement
        self.x = 1  # Meters
        # Time
        self.t = 5  # Seconds

        # Define linear and angular velocity values
        self.linear_velocity = self.x / self.t  # meters/second
        self.angular_velocity = 0.0  # No angular acceleration

        # Start time to track elapsed time  and initialize elapsed time
        self.start_time = time.time()
        self.elapsed_time = 0

    def control_loop(self):
        # Calculate elapsed time
        self.elapsed_time = time.time() - self.start_time

        # Create a message
        msg = Twist()

        # Publish velocities until the specified time has passed
        if self.elapsed_time < self.t:
            # Set linear velocity (forward motion)
            msg.linear.x = self.linear_velocity

            # Set angular velocity (no rotation in this case)
            msg.angular.z = self.angular_velocity

            # Publish the Twist message with velocities
            self.get_logger().info(
                f"Publishing linear velocity: {msg.linear.x}, angular velocity: {msg.angular.z}"
            )
        else:
            # Time has passed; stop the robot by setting velocities to 0
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            # Publish the stop command
            self.get_logger().info(
                "Turtle Bot travelled: "
                + str(self.x)
                + " meters in "
                + str(self.t)
                + " seconds"
            )
            self.publisher_.publish(msg)
            rclpy.shutdown()

        # Publish the message to the /cmd_vel topic
        self.publisher_.publish(msg)


# Scenario 2: The TurtleBot begins moving with an initial acceleration (a), accelerating until
# it reaches a certain speed (v). It then continues to move at this constant velocity (v) over
# a distance of x2​. Finally, the TurtleBot decelerates with a negative acceleration (- a) until
# it comes to a complete stop.
class OpenAccelerationController(Node):
    def __init__(self):
        super().__init__("tb_openLoop")

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

        # Control loop: Publish velocity commands every 0.1 seconds
        self.timer = self.create_timer(0.1, self.control_loop)

        # Track current state
        self.state = "ACCELERATING"

        # Define parameters
        self.x = 1  # meters
        self.current_distance = 0  # meters

        self.target_velocity = 0.2  # Maximum target velocity in m/s
        self.acceleration = 0.1  # m/s^2

        # Current velocity
        self.current_velocity = 0.0  # m/s

    def control_loop(self):
        # Create a message
        msg = Twist()

        # Accelerates until desired velocity is reached
        if self.state == "ACCELERATING":
            if self.current_velocity < self.target_velocity:
                # Increase velocity based on the acceleration and publish interval
                self.current_velocity += self.acceleration * 0.1

                # Keeps from exceeding maximum velocity
                self.current_velocity = min(self.current_velocity, self.target_velocity)
                print("Accelerating...")
                msg.linear.x = self.current_velocity
                self.publisher_.publish(msg)
                self.get_logger().info(f"Publishing velocity: {msg.linear.x} m/s")

            else:
                self.state = "HOLDVELOCITY"

        # Moves at aconstant velocity until desired distance is travelled
        elif self.state == "HOLDVELOCITY":
            if self.current_distance < self.x * 2:
                self.current_distance += self.current_velocity * 0.1
                # Printing distance travelled at constant speed
                print(
                    "Travelled:",
                    round(self.current_distance, 2),
                    "meters at",
                    round(self.current_velocity, 3),
                    "m/s",
                )
                msg.linear.x = self.current_velocity
                self.publisher_.publish(msg)
            else:
                self.state = "DECELERATING"

        # Decelerates until turtlebot has stopped
        elif self.state == "DECELERATING":
            if self.current_velocity > 0:
                # decrease velocity based on the acceleration and publish interval
                self.current_velocity -= self.acceleration * 0.1
                # Considering floating point error
                if self.current_velocity < 1e-6:
                    self.current_velocity = 0.0
                    self.state = "STOPPED"

            print("Decelerating...")
            msg.linear.x = self.current_velocity
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing velocity: {msg.linear.x} m/s")

        elif self.state == "STOPPED":
            print("Stopped")
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create Scenario 1 node
    # tb_openLoop = OpenVelocityController()

    # Create Scenario 2 node
    tb_openLoop = OpenAccelerationController()

    try:
        # Keep the node running
        rclpy.spin(tb_openLoop)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    # Execute the main function when this script is run
    main()
