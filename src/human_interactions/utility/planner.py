import rospy

# Msgs
from nav_msgs.srv import GetPlan, GetPlanRequest

# Local scripts
from logger import Logger, LogLevel


class Planner:
    """
    Provides the movement trajectory for reaching a goal pose.

    Attributes:
        _plan_service (rospy.ServiceProxy): The service for getting a navigation plan.
    """

    def __init__(self):
        self._logger = Logger("planner")

        # Init the plan service.
        self._plan_service = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

        self._logger.log("Planner ready.")

    def get_plan(self, start_pose, goal_pose, tolerance=0.5):
        """
        Request a navigation plan from the MoveBase plan service to the given
        destination.

        Args:
            goal_pose (Pose): The goal in robot's tf.
            tolerance (float): The x/y goal tolerance when planning.

        Returns:
            Optional[Path]: The navigation plan. If the location is not
                reacheable return None.
        """
        if not start_pose:
            self._logger.log("Can't determine start pose, aborting.", LogLevel.ERROR)
            return

        request = GetPlanRequest()
        request.start.header.frame_id = "map"
        request.start.pose = start_pose
        request.goal.header.frame_id = "map"
        request.goal.pose = goal_pose
        request.tolerance = tolerance

        try:
            response = self._plan_service.call(request)

            plan = response.plan
            plan.header.frame_id = "map"
            return plan
        except rospy.ServiceException:
            return None
