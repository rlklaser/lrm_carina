/*
 *  Copyright (C) 2012, Laboratorio de Robotica Movel - ICMC/USP
 *  Rafael Luiz Klaser <rlklaser@gmail.com>
 *  http://lrm.icmc.usp.br
 *
 *  Apoio FAPESP: 2012/04555-4
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file sba_from_file.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jan 6, 2013
 *
 */

#include <ros/ros.h>

#include <sba/sba.h>
#include <sba/sba_file_io.h>
#include <sba/visualization.h>

#include <visualization_msgs/Marker.h>

using namespace sba;
using namespace std;

void processSBAfile(char* filename, ros::NodeHandle node)
{
    // Create publisher topics.
    ros::Publisher cam_marker_pub = node.advertise<visualization_msgs::Marker>("/sba/cameras", 1);
    ros::Publisher point_marker_pub = node.advertise<visualization_msgs::Marker>("/sba/points", 1);
    ros::spinOnce();

    ROS_INFO("Sleeping for 2 seconds to publish topics...");
    //ros::Duration(2.0).sleep();

    // Create an empty SBA system.
    SysSBA sys;

    // Read in information from the bundler file.
    readBundlerFile(filename, sys);

    // Provide some information about the data read in.
    ROS_INFO("Cameras (nodes): %d, Points: %d",
        (int)sys.nodes.size(), (int)sys.tracks.size());

    // Publish markers
    drawGraph(sys, cam_marker_pub, point_marker_pub);
    ros::spinOnce();

    ROS_INFO("Sleeping for 5 seconds to publish pre-SBA markers.");
    //ros::Duration(5.0).sleep();

    // Perform SBA with 10 iterations, an initial lambda step-size of 1e-3,
    // and using CSPARSE.
    sys.doSBA(10, 1e-3, 1);

    int npts = sys.tracks.size();

    ROS_INFO("Bad projs (> 10 pix): %d, Cost without: %f",
        (int)sys.countBad(10.0), sqrt(sys.calcCost(10.0)/npts));
    ROS_INFO("Bad projs (> 5 pix): %d, Cost without: %f",
        (int)sys.countBad(5.0), sqrt(sys.calcCost(5.0)/npts));
    ROS_INFO("Bad projs (> 2 pix): %d, Cost without: %f",
        (int)sys.countBad(2.0), sqrt(sys.calcCost(2.0)/npts));

    ROS_INFO("Cameras (nodes): %d, Points: %d",
        (int)sys.nodes.size(), (int)sys.tracks.size());

    // Publish markers
    drawGraph(sys, cam_marker_pub, point_marker_pub);
    ros::spinOnce();
    ROS_INFO("Sleeping for 5 seconds to publish post-SBA markers.");
    //ros::Duration(5.0).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sba_from_file");

    ros::NodeHandle node;

    if (argc < 2)
    {
      ROS_ERROR("Arguments are:  <input filename>");
      return -1;
    }
    char* filename = argv[1];

    processSBAfile(filename, node);
    ros::spinOnce();

    return 0;
}



