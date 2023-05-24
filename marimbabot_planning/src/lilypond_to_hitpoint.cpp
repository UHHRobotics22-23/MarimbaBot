#include "marimbabot_planning/lilypond_to_hitpoint.h"


namespace marimbabot_planning
{

/**
 * @brief Convert lilypond sequence to cartesian poses and times
 *
 * @param tf_buffer
 * @param planning_frame
 * @param lilypond
 * @param tempo
 * @return std::vector<std::tuple<geometry_msgs::PoseStamped, double, double>> (cartesian pose, start time, duration)
 **/
std::vector<std::tuple<geometry_msgs::PoseStamped, double, double>> lilypond_to_cartesian(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string& planning_frame,
    std::string lilypond,
    double tempo)
{
    // The lilypond sequence is a string of notes and looks like this: "c'4 d''4 e4 f'4 g'16"
    // The notes are separated by spaces and the duration is given by the number after the note
    // The note is given by the letter and the octave by the ' and the number after the letter

    // Split the string into a vector of notes
    std::vector<std::string> notes;
    boost::split(notes, lilypond, boost::is_any_of(" "));

    // Initialize the vector of cartesian poses, start times and durations
    std::vector<std::tuple<geometry_msgs::PoseStamped, double, double>> hits;

    double time = 0.0;

    // Each key on the marimba has its own tf frame called something like "bar_F#5" or "bar_C4"
    // Iterate over all notes and calculate the cartesian pose for each note
    for(auto note : notes)
    {
        // Use regex to check that the note is valid and parsable
        std::regex note_regex("^[a-gr][']*[0-9]+$");
        if(!std::regex_match(note, note_regex))
        {
            ROS_WARN_STREAM("Note '" << note << "' is not valid");
            continue;
        }

        // Get the first letter
        std::string note_letter = note.substr(0, 1);

        // Check if note is a rest
        if (note_letter == "r")
        {
            // If the note is a rest, we don't need to calculate the cartesian pose and instead only increment the time
            time += 60 / std::stod(note.substr(1)) / tempo;
        }
        else
        {
            // Count the number of ' in the string to get the octave
            int octave = std::count(note.begin(), note.end(), '\'');

            // Get the duration of the note by getting the substring starting at 1 + octave
            double duration = 60 / std::stod(note.substr(1 + octave)) / tempo;

            // Capitalize the note letter
            note_letter[0] = std::toupper(note_letter[0]);

            // Map the note letter and octave to the corresponding tf frame
            std::string note_frame = "bar_" + note_letter + std::to_string(octave + 4);

            // Get the cartesian pose of the note
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = note_frame;  // The frame of the note
            pose.header.stamp = ros::Time(0);  // Use the latest available transform
            pose.pose.orientation.w = 1.0;  // The orientation of the note

            try
            {
                // Transform the pose to the planning frame
                auto transformed_pose = tf_buffer->transform(
                    pose,
                    planning_frame,
                    ros::Duration(1.0)
                    );

                // Add the cartesian pose and time to the vector
                hits.push_back(std::make_tuple(transformed_pose, time, duration));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("Failed to get the transformation from the robot base to the note frame on the marimba! Error: %s", ex.what());
                ROS_WARN("Skipping note %s", note.c_str());
                continue;
            }

            // Increment the time
            time += duration;
        }
    }

    return hits;
}

} // namespace marimbabot_planning