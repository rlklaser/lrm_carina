/**
* octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2010-2012.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
 * Copyright (c) 2010-2012, A. Hornung, University of Freiburg
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

#ifndef OCTOMAP_SERVER_OCTOMAPSERVER_H
#define OCTOMAP_SERVER_OCTOMAPSERVER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>

//#include <arm_navigation_msgs/CollisionObject.h>
//#include <arm_navigation_msgs/CollisionMap.h>
// #include <moveit_msgs/CollisionObject.h>
// #include <moveit_msgs/CollisionMap.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>

#include <lrm_octomap_server/OctomapServerConfig.h>
#include <lrm_octomap_server/SensorModelConfig.h>

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeStamped.h>
#include <lrm_octomap_server/OcTreeClipped.h>

#include <angles/angles.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <queue>

//#define USE_COLOR
#define USE_CLIPPING
//#define USE_STAMPTREE
//#define USE_NORMAL

namespace octomap_server {
class OctomapServer{

public:
  //typedef pcl::PointXYZ PointT;
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PCLPointCloud;
  typedef octomap_msgs::GetOctomap OctomapSrv;
  typedef octomap_msgs::BoundingBoxQuery BBXSrv;

#ifdef USE_COLOR
  typedef octomap::ColorOcTree OcTreeT;
#endif

#ifdef USE_STAMPTREE
  typedef octomap::OcTreeStamped OcTreeT;
#endif

#ifdef USE_CLIPPING
  typedef octomap::OcTreeClipped OcTreeT;
#endif

#ifdef USE_NORMAL
  typedef octomap::OcTree OcTreeT;
#endif

  OctomapServer(ros::NodeHandle nh, ros::NodeHandle nh_priv);
  virtual ~OctomapServer();
  bool octomapBinarySrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);
  bool octomapFullSrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);
  bool clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp);
  bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
  bool pruneSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

  void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  void insertCloudGroundCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  bool openFile(const std::string& filename);

protected:

  inline void updateNode(const octomap::OcTreeKey& key, float log_odds_update) {
#ifdef USE_CLIPPING
	  m_octree->updateNodeClipping(key, log_odds_update, log_odds_update==octomap::logodds(m_probHit));
#else
	  m_octree->updateNode(key, log_odds_update);
#endif
  }

  inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min){
    for (unsigned i=0; i<3; ++i)
      min[i] = std::min(in[i], min[i]);
  };
  
  inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max){
    for (unsigned i=0; i<3; ++i)
      max[i] = std::max(in[i], max[i]);
  };
 
  /// Test if key is within update area of map (2D, ignores height)
  inline bool isInUpdateBBX(const octomap::OcTreeKey& key) const{
    return (key[0] >= m_updateBBXMin[0] && key[1] >= m_updateBBXMin[1]
         && key[0] <= m_updateBBXMax[0] && key[1] <= m_updateBBXMax[1]);
  }

  void timerCallback(const ros::TimerEvent& t);
  void reconfigureCallback(lrm_octomap_server::OctomapServerConfig& config, uint32_t level);
  void reconfigureCallbackSM(lrm_octomap_server::SensorModelConfig& config, uint32_t level);
  void publishBinaryOctoMap(const ros::Time& rostime = ros::Time::now()) const;
  void publishFullOctoMap(const ros::Time& rostime = ros::Time::now()) const;
  void publishAll(const ros::Time& rostime = ros::Time::now());

  /**
  * @brief update occupancy map with a scan labeled as ground and nonground.
  * The scans should be in the global map frame.
  *
  * @param sensorOrigin origin of the measurements for raycasting
  * @param ground scan endpoints on the ground plane (only clear space)
  * @param nonground all other endpoints (clear up to occupied endpoint)
  */
  void insertScan(const tf::StampedTransform& sensorOrigin, const PCLPointCloud& ground, const PCLPointCloud& nonground);

  /// label the input cloud "pc" into ground and nonground. Should be in the robot's fixed frame (not world!)
  //void filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground) const;

  /**
  * @brief Find speckle nodes (single occupied voxels with no neighbors). Only works on lowest resolution!
  * @param key
  * @return
  */
  bool isSpeckleNode(const octomap::OcTreeKey& key) const;

  /// hook that is called after traversing all nodes
  void handlePreNodeTraversal(const ros::Time& rostime);

  /// hook that is called when traversing all nodes of the updated Octree (does nothing here)
  //void handleNode(const OcTreeT::iterator& it) {};

  /// hook that is called when traversing all nodes of the updated Octree in the updated area (does nothing here)
  //void handleNodeInBBX(const OcTreeT::iterator& it) {};

  /// hook that is called when traversing occupied nodes of the updated Octree
  void handleOccupiedNode(const OcTreeT::iterator& it);

  /// hook that is called when traversing occupied nodes in the updated area (updates 2D map projection here)
  void handleOccupiedNodeInBBX(const OcTreeT::iterator& it);

  /// hook that is called when traversing free nodes of the updated Octree
  void handleFreeNode(const OcTreeT::iterator& it);

  /// hook that is called when traversing free nodes in the updated area (updates 2D map projection here)
  void handleFreeNodeInBBX(const OcTreeT::iterator& it);

  /// hook that is called after traversing all nodes
  void handlePostNodeTraversal(const ros::Time& rostime);

  /// updates the downprojected 2D map as either occupied or free
  void update2DMap(const OcTreeT::iterator& it, bool occupied);
  void update2DMap(const OcTreeT::iterator& it, int idx, bool occupied);
  void update2DMap(int idx, const OcTreeT::iterator& it);

  inline unsigned mapIdx(int i, int j) const{
    return m_gridmap.info.width*j + i;
  }

  inline unsigned mapIdx(const octomap::OcTreeKey& key) const{
    return mapIdx((key[0] - m_paddedMinKey[0])/m_multires2DScale,
        (key[1] - m_paddedMinKey[1])/m_multires2DScale);

  }

  void putCenterMarker(tf::Quaternion orientation, Eigen::Vector4f centroid, Eigen::Vector4f max, Eigen::Vector4f min, double distance, tf::Quaternion, double);

  /**
   * Adjust data of map due to a change in its info properties (origin or size,
   * resolution needs to stay fixed). map already contains the new map info,
   * but the data is stored according to oldMapInfo.
   */

  void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const;

  inline bool mapChanged(const nav_msgs::MapMetaData& oldMapInfo, const nav_msgs::MapMetaData& newMapInfo){
    return (    oldMapInfo.height != newMapInfo.height
             || oldMapInfo.width !=newMapInfo.width
             || oldMapInfo.origin.position.x != newMapInfo.origin.position.x
             || oldMapInfo.origin.position.y != newMapInfo.origin.position.y);
  }

  std_msgs::ColorRGBA probMapColor(double h);
  std_msgs::ColorRGBA heightMapColor(double h);
  ros::NodeHandle m_nh;
  ros::NodeHandle m_nh_priv;
  ros::Publisher m_markerPub, m_markerSinglePub, m_binaryMapPub, m_laserPub, m_clusterPosePub;
  ros::Publisher m_fullMapPub, m_pointCloudPub, m_collisionObjectPub, m_mapPub, m_cmapPub;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudGroundSub;
  tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;
  tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudGroundSub;
  ros::Subscriber m_pointCloudSubs, m_pointCloudGroundSubs;

  ros::ServiceServer m_octomapBinaryService, m_octomapFullService, m_clearBBXService, m_resetService, m_pruneService;
  tf::TransformListener m_tfListener;

  typedef dynamic_reconfigure::Server<lrm_octomap_server::OctomapServerConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> m_reconfigureServer;
  boost::recursive_mutex m_config_mutex;

  typedef dynamic_reconfigure::Server<lrm_octomap_server::SensorModelConfig> ReconfigureServerSensorModel;
  boost::shared_ptr<ReconfigureServerSensorModel> m_reconfigureServerSM;
  //boost::recursive_mutex m_config_mutex;

  OcTreeT* m_octree;
  //octomap::KeyRay m_keyRay;  // temp storage for ray casting
  octomap::OcTreeKey m_updateBBXMin;
  octomap::OcTreeKey m_updateBBXMax;

  int m_poseSeq;

  bool m_updated;

  double m_maxRange;
  double m_maxRangeOcc;
  std::string m_worldFrameId; // the map frame
  std::string m_baseFrameId; // base of the robot for ground plane filtering
  std::string m_sourceFrameId; //sensor frame (cloud message can be for any frame)
  bool m_useHeightMap;
  std_msgs::ColorRGBA m_color;
  double m_colorFactor;

  bool m_latchedTopics;

  double m_res;
  unsigned m_treeDepth;
  unsigned m_maxTreeDepth;
  double m_probHit;
  double m_probHitMid;
  double m_probHitFar;
  double m_probFarDist;
  double m_probMidDist;
  double m_probMissGnd;
  double m_probMissObs;
  double m_thresMin;
  double m_thresMax;

  double m_pointcloudMinZ;
  double m_pointcloudMaxZ;
  double m_occupancyMinZ;
  double m_occupancyMaxZ;
  double m_minSizeX;
  double m_minSizeY;
  bool m_filterSpeckles;

  bool m_filterGroundPlane;
  double m_groundFilterDistance;
  double m_groundFilterAngle;
  double m_groundFilterPlaneDistance;

  double m_updateOcclusion;
  double m_occupancyThres;
  double m_waitTransform;

  bool m_compressMap;

  // downprojected 2D map:
  bool m_incrementalUpdate;
  nav_msgs::OccupancyGrid m_gridmap;
  bool m_publish2DMap;
  bool m_mapOriginChanged;
  octomap::OcTreeKey m_paddedMinKey;
  unsigned m_multires2DScale;
  bool m_projectCompleteMap;

  int m_unknownCost;
  int m_maximumCost;
  double m_decayCost;

  int m_singleMarkerId;

  double m_degradeTime;
  bool m_fullDownProjectMap;

  bool m_useGround;

private:
  boost::mutex m_mutex;
  //boost::recursive_mutex m_mutex;

  ros::Timer m_publisherTimer;

  int m_ticksOnSec;
  double m_rate;

  std::queue<octomap::OcTreeKey> m_clearQueue;

  void _publishAll(const ros::Time& rostime = ros::Time::now());
  void updateTreeProbabilities();

};
}

#endif
