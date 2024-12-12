#include <ros/ros.h>
#include <mrs_lib/transformer.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <mrs_lib/param_loader.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <cmath>
#include <random>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>

/* TODO: it considers only the position of the robot not the orientation */


/*LaserScanCluster Class //{ */
class LaserScanCluster {
  ros::NodeHandle nh_;
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher  clusters_pub_;
  ros::Publisher  clusters_pub_1;
  ros::Publisher  fake_scan_pub_;
  ros::Publisher  obstacles_array_pub_;
  ros::Timer      timer_;
  std::string     UAV_NAME_;
  std::mt19937    rng_;  // Random number generator
  double          robot_x_ = 0.0;
  double          robot_x1_ = 0.0;
  double          robot_y_ = 0.0;
  double          robot_y1_ = 0.0;
  double roll_ = 0.0;
  double pitch_ = 0.0;
  double robot_yaw_ = 0.0;
  double          _cluster_tolerance_;
  int             _cluster_min_size_;
  int             _cluster_max_size_;
  bool            _simulation_ = true;
  ros::Subscriber robot_position_sub_;
  ros::Subscriber robot_heding_sub_;
  ros::Subscriber robot_position_sub_1;
  ros::Time       last_time_received_msg_;
  mrs_lib::Transformer transformer_;
public:
  std::vector<double> _obstacles_x, _obstacles_y;
  double              _obstacles_size;

  /*LaserScanCluster init//{ */
  LaserScanCluster(ros::NodeHandle &nh, const std::string &UAV_NAME) : nh_(nh), UAV_NAME_(UAV_NAME), rng_(std::random_device{}()) {
    clusters_pub_       = nh_.advertise<visualization_msgs::MarkerArray>("/" + UAV_NAME + "/rplidar/clusters_", 1);
    
    clusters_pub_1       = nh_.advertise<visualization_msgs::MarkerArray>("/" + UAV_NAME + "/rplidar/clusters_1", 1);

    /* fake_scan_pub_      = nh_.advertise<sensor_msgs::LaserScan>("/" + UAV_NAME + "/rplidar/scan_", 1); */
    obstacles_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/" + UAV_NAME + "/rplidar/obstacles_", 1);

    fake_scan_pub_      = nh_.advertise<sensor_msgs::LaserScan>("/" + UAV_NAME + "/rplidar/scan", 1);

    /* robot_position_sub_ = nh_.subscribe("/" + UAV_NAME + "/rbl_controller/position_vis", 1, &LaserScanCluster::robotPositionCallback1, this); */

    /* robot_position_sub_1 = nh_.subscribe("/" + UAV_NAME + "/rbl_controller/position_vis", 1, &LaserScanCluster::robotPositionCallback, this); */

    robot_position_sub_ = nh_.subscribe("/" + UAV_NAME + "/estimation_manager/odom_main", 1, &LaserScanCluster::robotPositionCallback1, this);

/*     robot_position_sub_1 = nh_.subscribe("/" + UAV_NAME + "/estimation_manager/odom_main", 1, &LaserScanCluster::robotPositionCallback, this); */

    if (_simulation_) {
      laser_scan_sub_ = nh_.subscribe("/" + UAV_NAME_ + "/scan", 1, &LaserScanCluster::laserScanCallback, this);
      timer_          = nh_.createTimer(ros::Duration(0.1), &LaserScanCluster::timerCallback, this);
    } else {
      // if you want obstacles from laserscan
      laser_scan_sub_ = nh_.subscribe("/" + UAV_NAME + "/rplidar/scan_raw", 1, &LaserScanCluster::laserScanCallback, this);
      // if you want obstacles from hector_mapping
      map_sub_ = nh_.subscribe("/" + UAV_NAME + "/hector_mapping/map", 1, &LaserScanCluster::mapCallback, this);
    }

    if ((ros::Time::now() - last_time_received_msg_).toSec() > 3.0) {
      ROS_WARN("[Lidar]: Data not received since 3 seconds");
    }

    mrs_lib::ParamLoader param_loader(nh, "LaserScanCluster");
    param_loader.loadParam("obstacles_size", _obstacles_size, _obstacles_size);
    param_loader.loadParam("obstacles_x", _obstacles_x, _obstacles_x);
    param_loader.loadParam("obstacles_y", _obstacles_y, _obstacles_y);
    param_loader.loadParam("clustering/tolerance", _cluster_tolerance_);
    param_loader.loadParam("clustering/min_size", _cluster_min_size_);
    param_loader.loadParam("clustering/max_size", _cluster_max_size_);
    param_loader.loadParam("simulation", _simulation_);

  }
  //}

  /* /1* robotPositionCallback //{ *1/ */
  /* void robotPositionCallback(const visualization_msgs::Marker::ConstPtr &msg) { */
  /*   if (_simulation_) { */
  /*     robot_x_ = msg->pose.position.x; */
  /*     robot_y_ = msg->pose.position.y; */
  /*   } */
  /* } */
  /* //} */

  /* /1* robotPositionCallback //{ *1/ */
  /* void robotPositionCallback(const nav_msgs::Odometry::ConstPtr &msg) { */
  /*   if (_simulation_) { */
  /*     robot_x_ = msg->pose.pose.position.x; */
  /*     robot_y_ = msg->pose.pose.position.y; */
  /*     tf::Quaternion quat( */
  /*       msg->pose.pose.orientation.x, */
  /*       msg->pose.pose.orientation.y, */
  /*       msg->pose.pose.orientation.z, */
  /*       msg->pose.pose.orientation.w */
  /*     ); */
  /*     // Convert quaternion to roll, pitch, and yaw */
  /*     tf::Matrix3x3(quat).getRPY(roll_, pitch_, robot_yaw_); */
  /*     } */
  /* } */
  /* //} */

  /* /1* robotPositionCallback1 //{ *1/ */
  /* void robotPositionCallback1(const visualization_msgs::Marker::ConstPtr &msg) { */
  /*     robot_x1_ = msg->pose.position.x; */
  /*     robot_y1_ = msg->pose.position.y; */
  /* } */
  /* /1* void OdomCallback( *1/ */
  /* //} */

  /* robotPositionCallback1 //{ */
  void robotPositionCallback1(const nav_msgs::Odometry::ConstPtr &msg) {
    if (_simulation_) {
      robot_x_ = msg->pose.pose.position.x;
      robot_y_ = msg->pose.pose.position.y;
      tf::Quaternion quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
      );
      // Convert quaternion to roll, pitch, and yaw
      tf::Matrix3x3(quat).getRPY(roll_, pitch_, robot_yaw_);
      }
  }
  //}

  /*void laserScanCallback //{ */
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    // Convert LaserScan to PointCloud2
    sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
    convertLaserScanToPointCloud2(scan_msg, cloud_msg);
    last_time_received_msg_ = ros::Time::now();
    // Convert PointCloud2 to pcl PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    // Apply Voxel Grid Downsampling
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(0.03, 0.03, 0.03);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vox.filter(*cloud_filtered);
    // Set the maximum allowed distance FIXME: to add in param_loader

    // Create a filtered point cloud based on distance
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_distance(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &point : cloud_filtered->points) {
      double distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      /* if (distance <= 8.0) { */
      /* pcl::PointXYZ offset_point; */
      /* offset_point.x = point.x + 10;  // Apply x offset */
      /* offset_point.y = point.y + 10;  // Apply y offset */
      /* offset_point.z = point.z;             // z remains unchanged */
      cloud_filtered_distance->points.push_back(point);
      /* } */
    }

    // Apply Euclidean Clustering to the filtered point cloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered_distance);

    std::vector<pcl::PointIndices>                 cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(_cluster_tolerance_);  // Adjust based on your environment
    ec.setMinClusterSize(_cluster_min_size_);
    ec.setMaxClusterSize(_cluster_max_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered_distance);
    ec.extract(cluster_indices);

    // Create MarkerArray
    visualization_msgs::MarkerArray clusters;

    // Iterate through clusters
    for (std::size_t i = 0; i < cluster_indices.size(); ++i) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (std::size_t j = 0; j < cluster_indices[i].indices.size(); ++j) {
        cluster->points.push_back(cloud_filtered_distance->points[cluster_indices[i].indices[j]]);
      }
      // Create Marker for the cluster
      visualization_msgs::Marker marker = createClusterMarker(cluster, scan_msg->header, i);
      // Append Marker to MarkerArray
      clusters.markers.push_back(marker);
    }
    // Publish MarkerArray
    clusters_pub_.publish(clusters);
  }
  //}

  /*void convertLaserScanToPointCloud2 //{ */
  void convertLaserScanToPointCloud2(const sensor_msgs::LaserScan::ConstPtr &scan_msg, sensor_msgs::PointCloud2::Ptr &cloud_msg) {
    // Create a PointCloudXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Reserve space for points
    cloud->points.resize(scan_msg->ranges.size());

    int counter = 0;
    // Populate PointCloudXYZ with points from LaserScan
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {

      if (scan_msg->ranges[i] < 0.01) {
        continue;
      }

      // Calculate the angle of the current point
      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;

      // Calculate the Cartesian coordinates of the point
      double x = scan_msg->ranges[i] * cos(angle);
      double y = scan_msg->ranges[i] * sin(angle);

      if (isfinite(x) && isfinite(y)) {
        // Set the point in the PointCloudXYZ
        cloud->points[counter].x = x;
        cloud->points[counter].y = y;
        cloud->points[counter].z = 0.0;  // Assuming 2D laser scan, so z is set to 0
        counter++;
      }
    }

    cloud->points.resize(counter);

    // Convert PointCloudXYZ to PointCloud2
    pcl::toROSMsg(*cloud, *cloud_msg);

    // Set the header of the PointCloud2 message
    cloud_msg->header = scan_msg->header;
    cloud_msg->height = 1;
    cloud_msg->width  = cloud->points.size();
  }
  //}

  /* createClusterMarker //{ */
  visualization_msgs::Marker createClusterMarker(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster, std::size_t cluster_id) {
    visualization_msgs::Marker marker;
    marker.id                 = cluster_id;
    marker.type               = visualization_msgs::Marker::POINTS;
    marker.action             = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.15;  // Point size

    // Fixed colors for each cluster (you can customize these)
    std::vector<std::array<double, 3>> fixed_colors = {
        {1.0, 0.0, 0.0},  // Red
        {0.0, 1.0, 0.0},  // Green
        {0.0, 0.0, 1.0},  // Blue
        {1.0, 1.0, 0.0},  // Yellow
        {1.0, 0.0, 1.0},  // Magenta
        {0.0, 1.0, 1.0},  // Cyan
                          // Add more colors as needed
    };

    // Select a color based on cluster_id
    std::array<double, 3> cluster_color;
    if (cluster_id < fixed_colors.size()) {
      cluster_color = fixed_colors[cluster_id];
    } else {
      // If there are more clusters than predefined colors, use a default color
      cluster_color = {0.5, 0.5, 0.5};  // Gray
    }

    marker.color.r = cluster_color[0];
    marker.color.g = cluster_color[1];
    marker.color.b = cluster_color[2];
    marker.color.a = 0.5;  // Alpha

    // Convert cluster points to geometry_msgs/Point
    for (const auto &point : cluster->points) {
      geometry_msgs::Point p;
      p.x = point.x + robot_x_;
      p.y = point.y + robot_y_;
      p.z = point.z;
      marker.points.push_back(p);
    }

    return marker;
  }
  //}

  /* createClusterMarker //{ */
  visualization_msgs::Marker createClusterMarker(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster, const std_msgs::Header &header, std::size_t cluster_id) {
    visualization_msgs::Marker marker;
    marker.header             = header;
    marker.id                 = cluster_id;
    marker.type               = visualization_msgs::Marker::POINTS;
    marker.action             = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.15;  // Point size

    // Fixed colors for each cluster (you can customize these)
    std::vector<std::array<double, 3>> fixed_colors = {
        {1.0, 0.0, 0.0},  // Red
        {0.0, 1.0, 0.0},  // Green
        {0.0, 0.0, 1.0},  // Blue
        {1.0, 1.0, 0.0},  // Yellow
        {1.0, 0.0, 1.0},  // Magenta
        {0.0, 1.0, 1.0},  // Cyan
                          // Add more colors as needed
    };

    // Select a color based on cluster_id
    std::array<double, 3> cluster_color;
    if (cluster_id < fixed_colors.size()) {
      cluster_color = fixed_colors[cluster_id];
    } else {
      // If there are more clusters than predefined colors, use a default color
      cluster_color = {0.5, 0.5, 0.5};  // Gray
    }

    marker.color.r = cluster_color[0];
    marker.color.g = cluster_color[1];
    marker.color.b = cluster_color[2];
    marker.color.a = 0.5;  // Alpha

    // Convert cluster points to geometry_msgs/Point
    for (const auto &point : cluster->points) {
      geometry_msgs::Point p;
      p.x = point.x + robot_x_;
      p.y = point.y + robot_y_;
      p.z = point.z;
      marker.points.push_back(p);
    }

    return marker;
  }
  //}

  /* createObstaclesMarker //{ */
 void publishObstacleMarkers(const std::vector<double>& obstacles_x, 
                                const std::vector<double>& obstacles_y, 
                                double obstacle_size) {
        visualization_msgs::MarkerArray marker_array;
        for (size_t i = 0; i < obstacles_x.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = UAV_NAME_ + "/world_origin";
            marker.header.stamp = ros::Time::now();
            marker.ns = "obstacle_cylinders";
            marker.id = i; // Unique ID for each marker
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            
            // Set position and scale
            marker.pose.position.x = obstacles_x[i];
            marker.pose.position.y = obstacles_y[i];
            marker.pose.position.z = 0.0; // Center height
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = obstacle_size*2; // Diameter in X
            marker.scale.y = obstacle_size*2; // Diameter in Y
            marker.scale.z = obstacle_size; // Height

            // Set color
            marker.color.r = 1.0; // Red
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8; // Semi-transparent

            marker_array.markers.push_back(marker);
        }

        // Publish the marker array
        obstacles_array_pub_.publish(marker_array);
    }
  //}

  /*void timerCallback //{ */
  void timerCallback(const ros::TimerEvent &) {
    // Create a fake LaserScan message
    sensor_msgs::LaserScan::Ptr fake_scan(new sensor_msgs::LaserScan);
    fake_scan->header.stamp    = ros::Time::now();
    /* fake_scan->header.frame_id = UAV_NAME_ + "/world_origin"; */
    fake_scan->header.frame_id = UAV_NAME_ + "/world_origin";

    fake_scan->angle_min       = 0.0;
    fake_scan->angle_max       = 6.28;  // 2 * pi
    fake_scan->angle_increment = 0.225 / 180 * 3.1415;
    fake_scan->time_increment  = 0.0;
    fake_scan->scan_time       = 0.1;
    fake_scan->range_min       = 0.0;
    fake_scan->range_max       = 20.0;  // Set your desired max range

    int num_readings = static_cast<int>((fake_scan->angle_max - fake_scan->angle_min) / fake_scan->angle_increment);
    fake_scan->ranges.resize(num_readings);

    double                                 obstacle_radius = _obstacles_size;
    std::vector<std::pair<double, double>> obstacles0;
    std::vector<std::pair<double, double>> obstacles;
    for (size_t i = 0; i < _obstacles_x.size(); ++i) {
      obstacles.emplace_back(_obstacles_x[i], _obstacles_y[i]);
    }
    


    /* transformObstacles(obstacles0, obstacles, robot_x_,robot_y_,0); */ 




    // Random noise generator
    std::normal_distribution<double> noise_distribution(0.0, 0.01);  // Mean 0, Stddev 0.1

    // Fill ranges with simulated data
    for (int i = 0; i < num_readings; ++i) {
      // Calculate the angle of the current reading
      double angle = fake_scan->angle_min + i * fake_scan->angle_increment;

      // Initialize the minimum distance to the maximum range
      double min_distance = fake_scan->range_max;

      // Check distance to each obstacle
      for (const auto &obstacle : obstacles) {
        double obstacle_x = obstacle.first;
        double obstacle_y = obstacle.second;

        // Calculate the direction vector
        double direction_x = cos(angle);
        double direction_y = sin(angle);

        /* double direction_x = cos(angle+robot_yaw_); */
        /* double direction_y = sin(angle+robot_yaw_); */
        // Calculate the vector from the robot to the obstacle
        double dx = obstacle_x - robot_x_;
        double dy = obstacle_y - robot_y_;
        /* double dx = (obstacle_x - robot_x_)*cos(robot_yaw_) + (obstacle_x - robot_x_)*sin(robot_yaw_) ; */
        /* double dy = -(obstacle_x - robot_x_)*sin(robot_yaw_) + (obstacle_x - robot_x_)*cos(robot_yaw_) ; */
        
        // Project the vector onto the direction of the LaserSca
        double projection = direction_x * dx + direction_y * dy;
        if (projection > 0) {
          // Calculate the point on the obstacle's perimeter closest to the laser ray
          double closest_x               = robot_x_ + projection * direction_x;
          double closest_y               = robot_y_ + projection * direction_y;
          double dist_to_obstacle_center = sqrt((closest_x - obstacle_x) * (closest_x - obstacle_x) + (closest_y - obstacle_y) * (closest_y - obstacle_y));
          if (dist_to_obstacle_center <= obstacle_radius) {
            double dist_to_surface = projection - sqrt(obstacle_radius * obstacle_radius - dist_to_obstacle_center * dist_to_obstacle_center);
            min_distance           = std::min(min_distance, dist_to_surface);
          }
        }
      }

      
      // Add noise to the distance value
      double noisy_distance = min_distance + noise_distribution(rng_);
      // Ensure the range is within valid bounds
      if (noisy_distance < fake_scan->range_min) {
        noisy_distance = fake_scan->range_min;
      } else if (noisy_distance > fake_scan->range_max) {
        noisy_distance = fake_scan->range_max;
      }

      fake_scan->ranges[i] = noisy_distance;
    }

    // Publish the fake LaserScan message
    fake_scan_pub_.publish(*fake_scan);
    // Call the laserScanCallback with the fake LaserScan message
    laserScanCallback(fake_scan);
    publishObstacleMarkers(_obstacles_x, _obstacles_y, _obstacles_size);
  }
  //}

/*/1*void timerCallback //{ *1/ */
/*void timerCallback(const ros::TimerEvent &) { */
/*    // Create a fake LaserScan message */
/*    sensor_msgs::LaserScan::Ptr fake_scan(new sensor_msgs::LaserScan); */
/*    fake_scan->header.stamp    = ros::Time::now(); */
/*    fake_scan->header.frame_id = UAV_NAME_ + "/fcu"; */

/*    fake_scan->angle_min       = 0.0; */
/*    fake_scan->angle_max       = 6.28;  // 2 * pi */
/*    fake_scan->angle_increment = 0.225 / 180 * 3.1415; */
/*    fake_scan->time_increment  = 0.0; */
/*    fake_scan->scan_time       = 0.1; */
/*    fake_scan->range_min       = 0.0; */
/*    fake_scan->range_max       = 20.0;  // Set your desired max range */

/*    int num_readings = static_cast<int>((fake_scan->angle_max - fake_scan->angle_min) / fake_scan->angle_increment); */
/*    fake_scan->ranges.resize(num_readings); */

/*    double obstacle_radius = _obstacles_size; */

/*    // Step 1: Transform obstacles from world_origin to fcu */
/*    std::vector<std::pair<double, double>> obstacles_in_fcu; */
/*    try { */
/*        // Create a TF buffer and listener */
/*        tf2_ros::Buffer tfBuffer; */
/*        tf2_ros::TransformListener tfListener(tfBuffer); */

/*        // Get the transform from world_origin to fcu */
/*        geometry_msgs::TransformStamped transformStamped; */
/*        transformStamped = tfBuffer.lookupTransform(UAV_NAME_ + "/fcu", UAV_NAME_ + "/world_origin", ros::Time(0)); */

/*        // Transform each obstacle to fcu frame */
/*        for (size_t i = 0; i < _obstacles_x.size(); ++i) { */
/*            geometry_msgs::PointStamped obstacle_world, obstacle_fcu; */
/*            obstacle_world.header.frame_id = UAV_NAME_ + "/world_origin"; */
/*            obstacle_world.point.x         = _obstacles_x[i]; */
/*            obstacle_world.point.y         = _obstacles_y[i]; */
/*            obstacle_world.point.z         = 0.0; */

/*            tf2::doTransform(obstacle_world, obstacle_fcu, transformStamped); */
/*            obstacles_in_fcu.emplace_back(obstacle_fcu.point.x, obstacle_fcu.point.y); */
/*        } */
/*    } catch (tf2::TransformException &ex) { */
/*        ROS_WARN("Failed to transform obstacles to fcu frame: %s", ex.what()); */
/*        return; // Exit if transform is not available */
/*    } */

/*    // Step 2: Simulate LaserScan */
/*    std::normal_distribution<double> noise_distribution(0.0, 0.01);  // Mean 0, Stddev 0.1 */

/*    for (int i = 0; i < num_readings; ++i) { */
/*        double angle = fake_scan->angle_min + i * fake_scan->angle_increment; */
/*        double min_distance = fake_scan->range_max; */

/*        for (const auto &obstacle : obstacles_in_fcu) { */
/*            double obstacle_x = obstacle.first; */
/*            double obstacle_y = obstacle.second; */

/*            double direction_x = cos(angle); */
/*            double direction_y = sin(angle); */

/*            double dx = obstacle_x - robot_x_; */
/*            double dy = obstacle_y - robot_y_; */

/*            double projection = direction_x * dx + direction_y * dy; */
/*            if (projection > 0) { */
/*                double closest_x               = robot_x_ + projection * direction_x; */
/*                double closest_y               = robot_y_ + projection * direction_y; */
/*                double dist_to_obstacle_center = sqrt((closest_x - obstacle_x) * (closest_x - obstacle_x) + */
/*                                                      (closest_y - obstacle_y) * (closest_y - obstacle_y)); */
/*                if (dist_to_obstacle_center <= obstacle_radius) { */
/*                    double dist_to_surface = projection - sqrt(obstacle_radius * obstacle_radius - dist_to_obstacle_center * dist_to_obstacle_center); */
/*                    min_distance           = std::min(min_distance, dist_to_surface); */
/*                } */
/*            } */
/*        } */

/*        double noisy_distance = min_distance + noise_distribution(rng_); */
/*        if (noisy_distance < fake_scan->range_min) { */
/*            noisy_distance = fake_scan->range_min; */
/*        } else if (noisy_distance > fake_scan->range_max) { */
/*            noisy_distance = fake_scan->range_max; */
/*        } */

/*        fake_scan->ranges[i] = noisy_distance; */
/*    } */

/*    // Step 3: Publish the LaserScan */
/*    fake_scan_pub_.publish(*fake_scan); */
/*    laserScanCallback(fake_scan); */
/*    publishObstacleMarkers(_obstacles_x, _obstacles_y, _obstacles_size); */
/*} */
/*//} */

/*void mapCallback//{ */
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    // Map metadata
    float               resolution = msg->info.resolution;  // Map resolution in meters/cell
    int                 width      = msg->info.width;       // Map width (in number of cells)
    int                 height     = msg->info.height;      // Map height (in number of cells)
    geometry_msgs::Pose origin     = msg->info.origin;      // Map origin (position and orientation)

    // Vector to store obstacle positions
    std::vector<std::pair<float, float>> obstacle_positions;
    obstacle_positions.clear();
    // Iterate through the occupancy grid data
    for (int i = 0; i < width * height; ++i) {
      if (msg->data[i] == 100) {  // Occupied cell (obstacle)
        // Convert 1D index to 2D grid coordinates
        int x_index = i % width;
        int y_index = i / width;

        // Convert grid coordinates to world coordinates
        float x_world = origin.position.x + x_index * resolution;
        float y_world = origin.position.y + y_index * resolution;
/* TOADD this statement to reduce the number of obstacles */
        /* // Store the obstacle's world position */
        obstacle_positions.push_back(std::make_pair(x_world, y_world));
      }
    }

    // Output the number of obstacles found
    /* ROS_INFO("Found %lu obstacles.", obstacle_positions.size()); */
    /* std::cout << "xrobot: " << robot_x_ << ", yrobot: " << robot_y_ << std::endl; */

    // Print the obstacle positions
    /* for (const auto &obstacle : obstacle_positions) { */
    /*   ROS_INFO("Obstacle at: x = %f, y = %f", obstacle.first, obstacle.second); */
    /* } */
    processObstacles(obstacle_positions);
  }
  //}

  /*void processObstacles//{ */
  void processObstacles(std::vector<std::pair<float, float>> obstacle_positions) {
    // Step 1: Convert obstacle_positions to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto &pos : obstacle_positions) {
      pcl::PointXYZ point;
      point.x = pos.first;
      point.y = pos.second;
      point.z = 0.0;  // Assuming obstacles are on a 2D plane
      cloud->points.push_back(point);
    }

    // Step 2: Apply Voxel Grid Downsampling
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(0.03f, 0.03f, 0.03f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vox.filter(*cloud_filtered);

    // Step 3: Filter points based on maximum allowed distance
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_distance(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto &point : cloud_filtered->points) {
      /* double distance = std::sqrt(point.x * point.x + point.y * point.y); */
      /* double distance = std::sqrt(pow(point.x-robot_x1_,2) + pow(point.y-robot_y1_,2)); */
      /* if (distance <= 8.0) { */
        cloud_filtered_distance->points.push_back(point);
      /* } */
    }

    // Step 4: Apply Euclidean Clustering to the filtered point cloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered_distance);

    std::vector<pcl::PointIndices>                 cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(_cluster_tolerance_);  // Adjust based on your environment
    ec.setMinClusterSize(_cluster_min_size_);
    ec.setMaxClusterSize(_cluster_max_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered_distance);
    ec.extract(cluster_indices);

    // Step 5: Create and publish MarkerArray for visualization
    //

    std_msgs::Header header;
    header.frame_id = UAV_NAME_ + "/local_origin";
    header.stamp    = ros::Time::now();

    visualization_msgs::MarkerArray clusters;

    for (std::size_t i = 0; i < cluster_indices.size(); ++i) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (std::size_t j = 0; j < cluster_indices[i].indices.size(); ++j) {
        cluster->points.push_back(cloud_filtered_distance->points[cluster_indices[i].indices[j]]);
      }

      // Create Marker for the cluster (assuming you have a function to do this)
      visualization_msgs::Marker marker = createClusterMarker(cluster, header, i);
      clusters.markers.push_back(marker);
    }

    // Publish the clusters to RViz
    clusters_pub_1.publish(clusters);
  }
  //}
 

void transformObstacles(
    const std::vector<std::pair<double, double>>& obstacles,
    std::vector<std::pair<double, double>>& transformed_obstacles,
    double tx, double ty, double theta) {
    
    // Precompute cos and sin of the rotation angle
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    // Clear the output vector
    transformed_obstacles.clear();

    // Transform each obstacle
    for (const auto& obstacle : obstacles) {
        // Original coordinates in the world frame
        double x_world = obstacle.first;
        double y_world = obstacle.second;

        // Apply rotation and translation
        double x_transformed = cos_theta * x_world - sin_theta * y_world + tx;
        double y_transformed = sin_theta * x_world + cos_theta * y_world + ty;

        // Store the transformed coordinates
        transformed_obstacles.emplace_back(x_transformed, y_transformed);
    }
}

  //
      
};
//}


/*main() //{ */
int main(int argc, char **argv) {

  // Retrieve the value of the "UAV_NAME" environment variable
  const char *UAV_NAMEEnv = std::getenv("UAV_NAME");
  if (UAV_NAMEEnv == nullptr) {
    ROS_ERROR("Environment variable 'UAV_NAME' not set.");
    return 1;  // Exit with an error code
  }

  // Convert UAV_NAME to string
  std::string UAV_NAME(UAV_NAMEEnv);

  // Initialize the ROS node with the obtained value
  ros::init(argc, argv, "laser_scan_clustering_" + UAV_NAME);
  ros::NodeHandle  nh;
  LaserScanCluster laser_scan_cluster(nh, UAV_NAME);
  ros::spin();
  return 0;
}

