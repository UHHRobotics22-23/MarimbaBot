import rospy
import marimbabot_msg.actions

class HitSequenceAction:
    feedback = marimbabot_msg.actions.HitSequenceFeedback
    result = marimbabot_msg.actions.HitSequenceResult

    def __init__(self, name):
        self.action_name = name
        self.planning_publisher = rospy.Publisher('marimba_move', marimbabot_msg.actions.HitSequenceGoal, queue_size=10)

    def execute_callback(self, goal):
        # helper variables
        r = rospy.Rate(1)

        # initialize feedback
        self.feedback.playing = True

        # retrieve information from the goal
        hit_sequence_goals = goal.hit_sequence_elements

        # TODO forward this information to the planning node
        # could look like this:
        self.planning_publisher.publish(hit_sequence_goals)

        # TODO wait for the planning node to finish / retrieve information from the planning node
        # the planning node could publish information to a topic
        # TODO create subscriber to listen to above topic
        notes_were_played = True # TODO
        if notes_were_played:
            self.result.playing = False
            self.result.planning_successful = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self.result)
        else:
            self.result.playing = False
            self.result.planning_successful = False
            rospy.loginfo('%s: Not succeeded' % self._action_name) 


if __name__ == "__main__":
    rospy.init_node('hitsequence')
    server = HitSequenceAction(rospy.get_name())
    rospy.spin()