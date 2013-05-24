/**
* octomap_saver: Simple example which requests binary octomaps and stores them to a file.
*
* @author A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
 * Copyright (c) 2010, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <fstream>

#include <octomap_msgs/GetOctomap.h>
using octomap_msgs::GetOctomap;

#define USAGE "\nUSAGE: octomap_saver [-f] <mapfile.[bt|ot]>\n" \
                "  -f: Query for the full occupancy octree, instead of just the compact binary one\n" \
		"  mapfile.bt: filename of map to be saved (.bt: binary tree, .ot: general octree)\n"

using namespace std;
using namespace octomap;

/**
* @brief Map generation node.
*/
class MapSaver{
public:
  MapSaver(const std::string& mapname, bool full){
    ros::NodeHandle n;
    std::string servname = "octomap_binary";
    if (full)
      servname = "octomap_full";
    ROS_INFO("Requesting the map from %s...", n.resolveName(servname).c_str());
    GetOctomap::Request req;
    GetOctomap::Response resp;
    while(n.ok() && !ros::service::call(servname, req, resp))
    {
      ROS_WARN("Request to %s failed; trying again...", n.resolveName(servname).c_str());
      usleep(1000000);
    }

    if (n.ok()){ // skip when CTRL-C
      octomap::OcTree* octree = NULL;

      if (full){
        AbstractOcTree* tree = octomap_msgs::fullMsgToMap(resp.map);
        if (tree){
          octree = dynamic_cast<OcTree*>(tree);
        }

      } else{
        octree = octomap_msgs::binaryMsgToMap(resp.map);
      }

      if (octree){
        ROS_INFO("Map with %zu nodes received, saving to %s", octree->size(), mapname.c_str());
        
        std::string suffix = mapname.substr(mapname.length()-3, 3);
        if (suffix== ".bt"){
          if (!octree->writeBinary(mapname)){
            ROS_ERROR("Error writing to file %s", mapname.c_str());
          }
        } else if (suffix == ".ot"){
          if (!octree->write(mapname)){
            ROS_ERROR("Error writing to file %s", mapname.c_str());
          }
        } else{
          ROS_ERROR("Unknown file extension, must be either .bt or .ot");
        }


      } else{
        ROS_ERROR("Error reading OcTree from stream");
      }

      delete octree;

    }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_saver");
  std::string mapFilename("");
  bool fullmap = false;
  if (argc == 3 && strcmp(argv[1], "-f")==0){
    fullmap = true;
    mapFilename = std::string(argv[2]);
  } else if (argc == 2)
    mapFilename = std::string(argv[1]);
  else{
    ROS_ERROR("%s", USAGE);
    exit(1);
  }

  try{
    MapSaver ms(mapFilename, fullmap);
  }catch(std::runtime_error& e){
    ROS_ERROR("map_saver exception: %s", e.what());
    exit(2);
  }

  exit(0);
}


