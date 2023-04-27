import rospy
import actionlib
import marimbabot_msgs.actions

class HitSequenceAction:
    feedback = marimbabot_msgs.actions.HitSequenceFeedback
    result = marimbabot_msgs.actions.HitSequenceResult

    def __init__(self, name):
        self.action_name = name

    def execute_callback(self, goal):
        # helper variables
        r = rospy.Rate(1)
        
        # initialize feedback
        self.feedback.current_number_of_notes_played = 0
        self.feedback.played_sequence = [] 
        self.feedback.played_sequence_time_hit = []


        notes_sequence = goal.notes_sequence  # list of note string, e.g. c4
        notes_loudness = goal.notes_loudness # loudness factor for each note
        beats_per_minute = goal.beats_per_minute # tempo for sequence


        # TODO execute goal
        for note in notes_sequence)
            pass

        # conditional success
        if self.feedback.current_number_of_notes_played == len(self.goal.notes_sequence):
            self.result.success = True
            self.result.played_sequence = self.feedback.played_sequence
            self.result.played_sequence_time_hit = self.feedback.played_sequence_time_hit
            self.result.total_of_notes_played = self.feedback.current_number_of_notes_played
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self.result)


if __name__ == "__main__":
    rospy.init_node('hitsequence')
    server = HitSequenceAction(rospy.get_name())
    rospy.spin()