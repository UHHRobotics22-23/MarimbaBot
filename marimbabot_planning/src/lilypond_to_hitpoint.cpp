#include <boost/algorithm/string.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <regex>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"


/**
 * @brief Convert lilypond sequence to cartesian poses and times
 *
 * @param tf_buffer
 * @param planning_frame
 * @param lilypond
 * @param tempo (default: 60.0 bpm)
 * @return std::vector<std::tuple<geometry_msgs::PoseStamped, double, double>>
 **/
std::vector<std::tuple<geometry_msgs::PoseStamped, double, double>> lilypond_to_cartesian(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string& planning_frame,
    std::string lilypond,
    double tempo = 60.0)
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


/**
 * @brief ROS node that subscribes to the recognized lilypond sequence and publishes a marker at the correct time
 * class is needed to access the publisher inside the callback function
*/
class LilypondMotionConverter
{
public: 
    LilypondMotionConverter() {
        // start spinner
        ros::AsyncSpinner spinner(2);
        spinner.start();

        // hit marker publisher
        hit_marker_pub = nodeHandle.advertise<visualization_msgs::Marker>("hit_marker", 1);
        
        // vision node subscriber
        vision_node_subscriber = nodeHandle.subscribe("vision_node/recognized_sentence", 1, &LilypondMotionConverter::move_to_pose_callback, this);
    }

    // Callback function for the vision node subscriber
    void move_to_pose_callback(const std_msgs::String::ConstPtr& lilypond_sentence) {
        ROS_DEBUG("move_to_pose_callback heard: [%s]", lilypond_sentence->data.c_str());


        // Create tf2 listener
        std::shared_ptr<tf2_ros::Buffer> tfBuffer = std::make_shared<tf2_ros::Buffer>();
        tf2_ros::TransformListener tfListener(*tfBuffer);

        // Sleep to make sure the tf listener is ready and all publishers are connected
        ros::Duration(1.0).sleep();

        std::string lilypond_string = lilypond_sentence->data.c_str();

        // Convert lilypond sequence to cartesian poses and times
        auto hits = lilypond_to_cartesian(
            tfBuffer,
            "base_link",
            lilypond_string
            );

        ROS_DEBUG("Converted lilypond sequence to cartesian poses and times");

        // Loop over all hits and publish a marker at the correct time
        double time = 0.0;

        for(int i = 0; i < hits.size(); i++)
        {
            // Create a marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time::now();
            marker.ns = "marimba";
            marker.id = 0;
            // Show a downpointing gree arrow
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = std::get<0>(hits[i]).pose.position;
            // Offset the marker so it is not under the marimba
            marker.pose.position.z += 0.1;
            // Make it down pointing
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y =  0.707;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w =  0.707;
            marker.scale.x = 0.1;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            // Add lifetime (duration of the note)
            marker.lifetime = ros::Duration(std::get<2>(hits[i]));

            // Publish the marker
            hit_marker_pub.publish(marker);

            // Check if this is not the last one
            if(i < hits.size() - 1)
            {
                // Get the time of the next hit
                double next_time = std::get<1>(hits[i + 1]);

                // Sleep until the next hit
                ros::Duration(next_time - time).sleep();

                // Update the time
                time = next_time;
            }
        }

        ros::Duration(0.1).sleep();
    }

private:
    ros::NodeHandle nodeHandle; 
    ros::Publisher hit_marker_pub;
    ros::Subscriber vision_node_subscriber;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_lilypond_to_hitpoints");

    // Create LilypondMotionConverter object
    LilypondMotionConverter subscribe_and_publish_object;
    ros::spin();


    return 0;
}
