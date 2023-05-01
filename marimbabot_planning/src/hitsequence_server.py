import rospy
import actionlib
import marimbabot_msgs.actions
from std_msgs.msg import String

class HitSequenceAction:
    feedback = marimbabot_msgs.actions.HitSequenceFeedback
    result = marimbabot_msgs.actions.HitSequenceResult

    def __init__(self, name):
        self.action_name = name
        self.hitpoint_publisher = rospy.Publisher('move_lilypond_to_hitpoints', String, queue_size=10)


    def execute_callback(self, goal):
        # helper variables
        r = rospy.Rate(1)
        
        # initialize feedback
        self.feedback.current_number_of_notes_played = 0
        self.feedback.played_sequence = None

        lilypond_string = goal.lilypond_string  # list of note string, e.g. c4

        self.hitpoint_publisher.publish(lilypond_string)

        # TODO create subscriber to listen to the played notes
        # TODO conditional success
        if self.feedback.current_number_of_notes_played == len(self.goal.notes_sequence):
            self.result.success = True
            self.result.played_sequence = self.feedback.played_sequence
            self.result.total_of_notes_played = self.feedback.current_number_of_notes_played
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self.result)


if __name__ == "__main__":
    rospy.init_node('hitsequence')
    server = HitSequenceAction(rospy.get_name())
    rospy.spin()