/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <dvo_slam/camera_keyframe_tracking.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_keyframe_tracker");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    dvo_slam::CameraKeyframeTracker dense_tracker(nh, nh_private);

    ROS_INFO("started camera_keyframe_tracker...");

    ros::spin();
    //ros::MultiThreadedSpinner spinner(4);
    //spinner.spin();

    return 0;
}
