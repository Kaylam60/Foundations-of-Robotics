import numpy as np
import rospy

# Import necessary message type
# BEGIN QUESTION 4.3
"*** REPLACE THIS LINE ***"
from geometry_msgs.msg import PoseStamped

# END QUESTION 4.3


def norm_python(data):
    """Compute the norm for each row of a numpy array using Python for loops.

    >>> data = np.array([[3, 4],
    ...                  [5, 12]])
    >>> norm_python(data)
    array([ 7., 17.])
    """
    n, d = data.shape
    norm = np.zeros(n)
    add=0
    # You can use np.absolute
    # BEGIN QUESTION 4.1
    for i in range(n):
        for j in range(d):
            abs_value=np.absolute(data[i,j])
            add+=abs_value
        norm[i]=add
        add=0
    # END QUESTION 4.1
    return norm


def norm_numpy(data):
    """Compute the norm for each row of a numpy array using numpy functions.

    >>> data = np.array([[3, 4],
    ...                  [5, 12]])
    >>> norm_numpy(data)
    array([ 7., 17.])
    """
    n, d = data.shape
    norm = np.zeros(n)
    # You can use np.linalg.norm.
    # Hint: you may find the `axis` parameter useful.
    # BEGIN QUESTION 4.2
    norm=np.linalg.norm(data, 1, axis=1)
    # END QUESTION 4.2
    return norm


class PoseListener:
    """Collect car poses."""

    def __init__(self, size=100):
        self.size = size
        self.done = False
        self.storage = []  # a list of (x, y) tuples
        # Create a subscriber for the car pose.
        # Hint: Your subscriber should be instantiated as follows: self.subscriber = rospy.Subscriber(...)
        # Hint: once you've figured out the right message type, don't forget to
        # import it at the top! If the message type from `rostopic info` is
        # "X_msgs/Y", the Python import would be "from X_msgs.msg import Y".
        # BEGIN QUESTION 4.3
        "*** REPLACE THIS LINE ***"
        self.subscriber=rospy.Subscriber("car/car_pose",PoseStamped,self.callback)

        # END QUESTION 4.3

    def callback(self, msg):
        """Store the x and y coordinates of the car."""
        header = msg.header
        rospy.loginfo(
            "Received a new message with timestamp " + str(header.stamp.secs) + "(s)"
        )

        # Extract and store the x and y position from the message data
        # BEGIN QUESTION 4.4
        "*** REPLACE THIS LINE ***"
        x_position = msg.pose.position.x
        y_position = msg.pose.position.y
        self.storage.append((x_position, y_position))
        # END QUESTION 4.4
        if len(self.storage) == self.size:
            self.done = True
            rospy.loginfo("Received enough samples, trying to unsubscribe")
            self.subscriber.unregister()
