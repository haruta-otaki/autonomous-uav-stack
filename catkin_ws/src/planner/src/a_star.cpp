#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <vector> 
#include <unordered_map>
#include <cmath>
#include <tuple>

// header file generated from the srv file 
#include <planner/plan_path.h>

#include <octomap_msgs/Octomap.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <nav_msgs/Path.h>

#include <dynamicEDT3D/dynamicEDTOctomap.h>

// cell coordinate in sparse hash grid
struct GridKey {
    int x, y, z; 
    // operator overloading, allowing GridKey comparisons
    // const keyword is necessary for compilation, promising the operation will not modify the objects by making them immutable 
    bool operator==(const GridKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    // operator< is used only for set<Pair> to order by (f, GridKey)(it has no spatial meaning)
    bool operator<(const GridKey& other) const {
        return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
    }
};

struct GridKeyHash {
    // custom hash function combining the cell coordinates
    std::size_t operator()(const GridKey& k) const noexcept {
        // formula from Boost's hash_combine to spread entropy well 
        std::size_t h = std::hash<int>{}(k.x);
        h ^= std::hash<int>{}(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int>{}(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h; 
    }
};

struct Node {
    octomap::point3d point;
    GridKey parent{}; 
    double g = std::numeric_limits<double>::max(); 
    double h = std::numeric_limits<double>::max(); 
    double clearance = 0.0; 
    bool has_parent = false; 
    bool closed = false; 
    bool exist = false; 

    // constructor 
    Node(octomap::point3d p) {
        point = p; 
    }

    double get_f() {
        return g + h; 
    }
};

octomap::AbstractOcTree* abstract;
octomap::OcTree* tree;  
DynamicEDTOctomap* distmap = nullptr;
ros::Publisher path_pub; 
double resolution = 0.4; 
double minimum_clearance = 0.45; // iris radius (0.25) + half_voxel (0.2)


// takes in the request and response type defined in the srv file and returns 
void build_edt()
{
    if (!tree) {
        ROS_WARN("[PLANNER] octomap message has not arrived yet");
        return;
    }
    // get minimum and maximum coordinates from octree and create OctoMap point objects
    double x,y,z;
    tree->getMetricMin(x,y,z);
    octomap::point3d min(x,y,z);
    //std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
    tree->getMetricMax(x,y,z);
    octomap::point3d max(x,y,z);
    //std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;

    // treat unknown voxels as free (aggressive approach)
    bool unknownAsOccupied = false;
    // indoor value
    float maxDist = 2.0;    
    if (distmap) {
        delete distmap;
    }
    // compute an Euclidean Distance Transform (EDT) over an OctoMap to get maximize clearance from obstacles
    // clamp distance computations and restrict the distance map to a subarea through the arguments
    distmap = new DynamicEDTOctomap(maxDist, tree, min, max, unknownAsOccupied);    
    // EDT algorithm: walks through the entire octree and computes, for every free voxel, its distance to nearest occupied voxel
    // repeated calls incrementally updates the distances to reflect the modified occupancy map
    distmap->update(); 
}

bool query_edt(octomap::point3d p, double& clearance)
{
    if (!tree || !distmap) {
        ROS_WARN("[PLANNER] edt not built, call build_edt service");
        return false;
    }

    if (tree->search(p) == nullptr) {
        ROS_WARN("[PLANNER] queried point never observed by sensor");
        return false; 
    }

    octomap::point3d closestObst;
    float distance;
    geometry_msgs::Point closest_obst_pose; 
    // return the distance of the obstacle from the query point and its location
    distmap->getDistanceAndClosestObstacle(p, distance, closestObst);
    closest_obst_pose.x = closestObst.x();
    closest_obst_pose.y = closestObst.y();
    closest_obst_pose.z = closestObst.z();
    
    clearance = distance;
    return true; 
}

class SparseHashGrid {
public:
    // map using custom key and hash to store A* Nodes
    std::unordered_map<GridKey, Node, GridKeyHash> grid;

    // convert continuous world coordinates into discrete cell coordinates
    GridKey world_to_grid(octomap::point3d p) const {
        // brace initialization
        return {static_cast<int>(std::lround(p.x() / resolution)),
            static_cast<int>(std::lround(p.y() / resolution)),
            static_cast<int>(std::lround(p.z() / resolution))};
    }

    Node& get_or_insert(const GridKey& key, octomap::point3d p) {
        // finds key, average case: O(1)
        // returns an iterator, auto lets the compiler deduce the type
        auto iterator = grid.find(key);
        // iterator points at the end beyond the last key
        if (iterator == grid.end()) {
            Node node(p);
            double clearance;
            node.exist = query_edt(p, clearance);
            node.clearance = clearance;
            grid[key] = node; 
            return grid[key];
        } else {
            // the second element in the iterator<GridKey, Node, GridKeyHash> is Node 
            return iterator->second; 
        }
    }
};

// global pointer
SparseHashGrid hash_grid; 
// create an alias for pair (tuple)
typedef std::pair<double, GridKey> Pair;

// determine when to build_edt() again
void octomap_cb(const octomap_msgs::Octomap::ConstPtr &msg){
    if (abstract) {
        delete abstract;
    }
    if (tree) {
        delete tree;
    }
    // msgToMap() returns AbstractOcTree* as the message could theoretically contain any tree type. 
    abstract = octomap_msgs::msgToMap(*msg);
    // get concrete OcTree type
    tree = dynamic_cast<octomap::OcTree*>(abstract);
    if (distmap) {
        distmap->update(); 
    }
}

bool plan_path(planner::plan_path::Request  &req,
        planner::plan_path::Response &res)
{
    if (!distmap) {
       build_edt();
        // check resolution looks right and confirm coordinate frames are aligned: $ octovis simple_tree.bt
        tree->writeBinary("/tmp/tree.bt");
    }
    octomap::point3d start(req.start.pose.position.x, req.start.pose.position.y, req.start.pose.position.z);
    octomap::point3d goal(req.goal.pose.position.x, req.goal.pose.position.y, req.goal.pose.position.z);
    GridKey start_key = hash_grid.world_to_grid(start);
    GridKey goal_key = hash_grid.world_to_grid(goal);
    Node source = hash_grid.get_or_insert(start_key, start);
    Node destination = hash_grid.get_or_insert(goal_key, goal);

    a_star(start_key, goal_key);
    return true;
}

bool is_not_valid(Node node) {
    return !node.exist; 
}

bool is_obstacle(double clearance, double threshold)
{
    return clearance <= threshold; 
}

bool is_destination(const GridKey& current, const GridKey& destination)
{
    return current == destination; 
}

// calculate 3D Euclidean distance
double calculate_heuristic(octomap::point3d current, octomap::point3d destination)
{
    double dx = current.x()-destination.x();
    double dy = current.y()-destination.y();
    double dz = current.z()-destination.z();

    return ((double)sqrt(dx * dx + dy * dy + dz * dz));
}

void generate_path(const GridKey& destination_key)
{
    std::vector<octomap::point3d> path; 
    GridKey key = destination_key; 

    // source does not have parent 
    while (true) {
        Node current_node = hash_grid.grid[key];
        path.push_back(current_node.point);
        if (!current_node.has_parent) {
            break;
        }
        key = current_node.parent; 
    }

    std::reverse(path.begin(), path.end());

    // instantiate Path message
    nav_msgs::Path path_msg;

    // populate main header
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    // create geomerty_msgs waypoints into the path's poses vector
    for (auto& p : path) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;

        pose.pose.position.x = p.x();
        pose.pose.position.y = p.y();
        pose.pose.position.z = p.z();
        pose.pose.orientation.w = 1.0;

        path_msg.poses.push_back(pose);
    }

    path_pub.publish(path_msg);
}

// A* Search Algorithm
void a_star(GridKey source_key, GridKey destination_key)
{
    // source or destination is an obstacle (collision gurantee)
    if (is_obstacle(hash_grid.grid[source_key].clearance, minimum_clearance) || is_obstacle(hash_grid.grid[destination_key].clearance, minimum_clearance)) {
        ROS_WARN("[PLANNER] source or destination is aligned with an obstacle");
        return;
    }

    if (is_destination(source_key, destination_key)) {
        return;
    }

    // initialize source 
    hash_grid.grid[source_key].g = 0.0;
    hash_grid.grid[source_key].h = 0.0;

    //  create open list (set) with <f, <i, j>>
    std::set<Pair> open_list;
    open_list.insert(std::make_pair(hash_grid.grid[source_key].get_f(), source_key));

    bool found_destination = false;

    while (!open_list.empty()) {
        Pair p = *open_list.begin();

        // remove vertex from open list
        open_list.erase(open_list.begin());

        hash_grid.grid[p.second].closed = true;
        
        // generate 26 (3 * 3 * 3 - 1) neighbors / successors of p 

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                for (int k = -1; k <= 1; k++) {
                    if (i == 0 && j == 0 && k == 0) {
                        continue; 
                    }
                    else { 
                        double di = i * resolution; 
                        double dj = j * resolution; 
                        double dk = k * resolution; 

                        // is valid check 
                        GridKey neighbor_key {p.second.x + i, p.second.y + j, p.second.z + k};
                        octomap::point3d neighbor_point(hash_grid.grid[p.second].point.x() + di,
                         hash_grid.grid[p.second].point.y() + dj, hash_grid.grid[p.second].point.z() + dk); 

                        Node &neighbor = hash_grid.get_or_insert(neighbor_key, neighbor_point); 

                        // skip if neighbor is out of bounds, already visited, or an obstacle
                        if (is_not_valid(neighbor) || neighbor.closed || is_obstacle(neighbor.clearance, minimum_clearance)) {
                            continue; 
                        }

                        // neighbor is destination
                        if (is_destination(neighbor_key, destination_key)) {
                            ROS_INFO("[PLANNER] destination found");
                            generate_path(neighbor_key); 
                            found_destination = true;
                            return;
                        }

                        // new g, h, f values
                        double g = hash_grid.grid[p.second].g + sqrt(di * di + dj * dj + dk * dk); 
                        double h = calculate_heuristic(neighbor_point, hash_grid.grid[destination_key].point); 
                        double f = g + h; 

                        // if neighbor is not in open list or has a new minimum f value, insert / update the node
                        if (neighbor.get_f() == std::numeric_limits<double>::max() || neighbor.get_f() > f) {
                            open_list.insert(std::make_pair(f, neighbor_key));
                            neighbor.g = g; 
                            neighbor.h = h; 
                            neighbor.has_parent = true; 
                            neighbor.parent = p.second;
                        }
                    }
                }
            }
        }
    }

    // when the destination cell is not found and the open list is empty, 
    // A* failed to reach the destination cell which may be due to blockages
    if (found_destination == false)
        ROS_WARN("[PLANNER] failed to find the Destination Cell");
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_node");

    ros::NodeHandle nh;

    // ros::Subscriber ground_truth_pose_sub = nh.subscribe<nav_msgs::Odometry>
    //     ("ground_truth/pose", 10, ground_truth_pose_cb);

    // check resolution looks right and confirm coordinate frames are aligned: $ octovis simple_tree.bt
    // tree.writeBinary("tree.bt");
   
    ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>
    ("octomap_binary", 10, octomap_cb);

    // service is created and advertised over ROS 
    ros::ServiceServer plan_service = nh.advertiseService("plan_path", plan_path);

    path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
    // service is created and advertised over ROS 
    // ros::ServiceServer build_service = nh.advertiseService("build_edt", build_edt);

    ros::spin();
    return (0);
}