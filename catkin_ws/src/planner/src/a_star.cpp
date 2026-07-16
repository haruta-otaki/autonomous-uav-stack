#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <vector> 
#include <unordered_map>
#include <cmath>

#include <mapping/build_edt.h>
#include <mapping/query_edt.h>
#include <planner/plan_path.h>

#include <octomap_msgs/Octomap.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <bits/stdc++.h>
using namespace std;

// void octomap_cb(const octomap_msgs::Octomap::ConstPtr &msg){
// }

octomap::AbstractOcTree* abstract;
octomap::OcTree* tree; 
// global pointer 
DynamicEDTOctomap* distmap = nullptr;
double resolution = 0.4; 
double minimum_clearance = 0.45; // iris radius (0.25) + half_voxel (0.2)


void octomap_cb(const octomap_msgs::Octomap::ConstPtr &msg){
    // msgToMap() returns AbstractOcTree* as the message could theoretically contain any tree type. 
    abstract = octomap_msgs::msgToMap(*msg);
    // get concrete OcTree type
    tree = dynamic_cast<octomap::OcTree*>(abstract);
}

// takes in the request and response type defined in the srv file and returns 
void build_edt()
{
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

double query_edt(octomap::point3d p)
{
    if (!distmap) {
        ROS_WARN("edt not built, call build_edt service");
    }

    distmap->update(); 

    octomap::point3d closestObst;
    float distance;
    geometry_msgs::Point closest_obst_pose; 
    // return the distance of the obstacle from the query point and its location
    distmap->getDistanceAndClosestObstacle(p, distance, closestObst);
    closest_obst_pose.x = closestObst.x();
    closest_obst_pose.y = closestObst.y();
    closest_obst_pose.z = closestObst.z();
    
    return distance;
}

// cell coordinate in sparse hash grid
struct GridKey {
    int x, y, z; 
    // operator overloading, allowing GridKey comparisons
    // const keyword is necessary for compilation, promising the operation will not modify the objects by making them immutable 
    bool operator==(const GridKey& other) const {
        return x == other.x && y == other.y && z == other.z;
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

    // constructor 
    Node(octomap::point3d p) {
        point = p; 
    }

    double get_f() {
        return g + h; 
    }
};

class SparseHashGrid {
public:
    float cellSize;
    // map using custom key and hash to store A* Nodes
    std::unordered_map<GridKey, Node, GridKeyHash> grid;

    // convert continuous world coordinates into discrete cell coordinates
    GridKey world_to_grid(octomap::point3d p) const {
        // brace initialization
        return {static_cast<int>(std::lround(p.x() / resolution)),
            static_cast<int>(std::lround(p.y() / resolution)),
            static_cast<int>(std::lround(p.z() / resolution))};
    }

    // when to call world_to_grid
    Node& get_or_insert(const GridKey& key, octomap::point3d p) {
        // finds key, average case: O(1)
        // returns an iterator, auto lets the compiler deduce the type
        auto iterator = grid.find(key);
        // iterator points at the end beyond the last key
        if (iterator == grid.end()) {
            Node node(p);
            node.clearance = query_edt(p);
            grid[key] = node; 
            return grid[key];
        } else {
            // the second element in the iterator<GridKey, Node, GridKeyHash> is Node 
            return iterator->second; 
        }
    }
};

SparseHashGrid hash_grid; 


bool plan_path(planner::plan_path::Request  &req,
        planner::plan_path::Response &res)
{
    if (!distmap) {
       build_edt();
    }
    octomap::point3d start(req.start.pose.position.x, req.start.pose.position.y, req.start.pose.position.z);
    octomap::point3d goal(req.goal.pose.position.x, req.goal.pose.position.y, req.goal.pose.position.z);
    GridKey start_key = hash_grid.world_to_grid(start);
    GridKey goal_key = hash_grid.world_to_grid(goal);
    Node source = hash_grid.get_or_insert(start_key, start);
    Node destination = hash_grid.get_or_insert(goal_key, goal);

    a_star(source, destination);
    return true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_node");

    ros::NodeHandle nh;

    // ros::Subscriber ground_truth_pose_sub = nh.subscribe<nav_msgs::Odometry>
    //     ("ground_truth/pose", 10, ground_truth_pose_cb);

    // check resolution looks right and confirm coordinate frames are aligned: $ octovis simple_tree.bt
    // tree.writeBinary("tree.bt");
    
    // service is created and advertised over ROS 
    ros::ServiceServer plan_service = nh.advertiseService("plan_path", plan_path);

    ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>
        ("octomap_binary", 10, octomap_cb);

    return (0);
}

bool is_valid(octomap::point3d source, octomap::point3d destination, octomap::point3d current)
{
    double min_x = std::min(source.x(), destination.x());
    double max_x = std::max(source.x(), destination.x());

    double min_y = std::min(source.y(), destination.y());
    double max_y = std::max(source.y(), destination.y());

    double min_z = std::min(source.z(), destination.z());
    double max_z = std::max(source.z(), destination.z());

    return current.x() >= min_x && current.x() <= max_x &&
           current.y() >= min_y && current.y() <= max_y &&
           current.z() >= min_z && current.z() <= max_z;
}

bool is_obstacle(double clearance, double threshold)
{
    return clearance <= threshold; 
}

bool is_destination(octomap::point3d destination, octomap::point3d current)
{
    return current == destination; 
}

// calculate 3D Euclidean distance
double calculate_heuristic(octomap::point3d destination, octomap::point3d current)
{
    double dx = current.x()-destination.x();
    double dy = current.y()-destination.y();
    double dz = current.z()-destination.z();

    return ((double)sqrt(dx * dx + dy * dy + dz * dz));
}

std::vector<octomap::point3d> generate_path(const GridKey& destination_key)
{
    std::vector<octomap::point3d> path; 
    GridKey key = destination_key; 

    std::printf("Path: \n");

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
    return path;
}
// create an alias for pair (tuple)
typedef pair<double, Node> Pair;

// A* Search Algorithm
void a_star(Node source, Node destination)
{
    // source or destination is an obstacle (collision gurantee)
    if (is_obstacle(source.clearance, minimum_clearance) || is_obstacle(destination.clearance, minimum_clearance)) {
        std::printf("source or destination is aligned with an obstacle \n");
        return;
    }

    if (is_destination(source.point, destination.point)) {
        return;
    }

    // create a closed list, initialise it to false (no cell has been included) 
    bool closedList[ROW][COL];
    memset(closedList, false, sizeof(closedList));


    cell cellDetails[ROW][COL];

    // initialize source 
    source.g = 0.0;
    source.h = 0.0;

    
    //  create open list (set) with <f, <i, j>>
    set<Pair> open_list;
    open_list.insert(make_pair(source.get_f(), source));

    bool found_destination = false;

    while (!open_list.empty()) {
        Pair p = *open_list.begin();

        // Remove this vertex from the open list
        open_list.erase(open_list.begin());

        // Add this vertex to the closed list
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;

        /*
         Generating all the 8 successor of this cell

             N.W   N   N.E
               \   |   /
                \  |  /
             W----Cell----E
                  / | \
                /   |  \
             S.W    S   S.E

         Cell-->Popped Cell (i, j)
         N -->  North       (i-1, j)
         S -->  South       (i+1, j)
         E -->  East        (i, j+1)
         W -->  West           (i, j-1)
         N.E--> North-East  (i-1, j+1)
         N.W--> North-West  (i-1, j-1)
         S.E--> South-East  (i+1, j+1)
         S.W--> South-West  (i+1, j-1)*/

        // To store the 'g', 'h' and 'f' of the 8 successors
        double gNew, hNew, fNew;

        //----------- 1st Successor (North) ------------

        // Only process this cell if this is a valid one
        if (is_valid(i - 1, j) == true) {
            // If the destination cell is the same as the
            // current successor
            if (is_destination(i - 1, j, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i - 1][j].parent_i = i;
                cellDetails[i - 1][j].parent_j = j;
                printf("The destination cell is found\n");
                generate_path(cellDetails, dest);
                found_destination = true;
                return;
            }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i - 1][j] == false
                     && is_obstacle(grid, i - 1, j)
                            == true) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i - 1, j, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i - 1][j].f == FLT_MAX
                    || cellDetails[i - 1][j].f > fNew) {
                    open_list.insert(make_pair(
                        fNew, make_pair(i - 1, j)));

                    // Update the details of this cell
                    cellDetails[i - 1][j].f = fNew;
                    cellDetails[i - 1][j].g = gNew;
                    cellDetails[i - 1][j].h = hNew;
                    cellDetails[i - 1][j].parent_i = i;
                    cellDetails[i - 1][j].parent_j = j;
                }
            }
        }

        //----------- 2nd Successor (South) ------------

        // Only process this cell if this is a valid one
        if (is_valid(i + 1, j) == true) {
            // If the destination cell is the same as the
            // current successor
            if (is_destination(i + 1, j, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i + 1][j].parent_i = i;
                cellDetails[i + 1][j].parent_j = j;
                printf("The destination cell is found\n");
                generate_path(cellDetails, dest);
                found_destination = true;
                return;
            }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i + 1][j] == false
                     && is_obstacle(grid, i + 1, j)
                            == true) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i + 1, j, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i + 1][j].f == FLT_MAX
                    || cellDetails[i + 1][j].f > fNew) {
                    open_list.insert(make_pair(
                        fNew, make_pair(i + 1, j)));
                    // Update the details of this cell
                    cellDetails[i + 1][j].f = fNew;
                    cellDetails[i + 1][j].g = gNew;
                    cellDetails[i + 1][j].h = hNew;
                    cellDetails[i + 1][j].parent_i = i;
                    cellDetails[i + 1][j].parent_j = j;
                }
            }
        }

        //----------- 3rd Successor (East) ------------

        // Only process this cell if this is a valid one
        if (is_valid(i, j + 1) == true) {
            // If the destination cell is the same as the
            // current successor
            if (is_destination(i, j + 1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i][j + 1].parent_i = i;
                cellDetails[i][j + 1].parent_j = j;
                printf("The destination cell is found\n");
                generate_path(cellDetails, dest);
                found_destination = true;
                return;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i][j + 1] == false
                     && is_obstacle(grid, i, j + 1)
                            == true) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i, j + 1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i][j + 1].f == FLT_MAX
                    || cellDetails[i][j + 1].f > fNew) {
                    open_list.insert(make_pair(
                        fNew, make_pair(i, j + 1)));

                    // Update the details of this cell
                    cellDetails[i][j + 1].f = fNew;
                    cellDetails[i][j + 1].g = gNew;
                    cellDetails[i][j + 1].h = hNew;
                    cellDetails[i][j + 1].parent_i = i;
                    cellDetails[i][j + 1].parent_j = j;
                }
            }
        }

        //----------- 4th Successor (West) ------------

        // Only process this cell if this is a valid one
        if (is_valid(i, j - 1) == true) {
            // If the destination cell is the same as the
            // current successor
            if (is_destination(i, j - 1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i][j - 1].parent_i = i;
                cellDetails[i][j - 1].parent_j = j;
                printf("The destination cell is found\n");
                generate_path(cellDetails, dest);
                found_destination = true;
                return;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i][j - 1] == false
                     && is_obstacle(grid, i, j - 1)
                            == true) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i, j - 1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i][j - 1].f == FLT_MAX
                    || cellDetails[i][j - 1].f > fNew) {
                    open_list.insert(make_pair(
                        fNew, make_pair(i, j - 1)));

                    // Update the details of this cell
                    cellDetails[i][j - 1].f = fNew;
                    cellDetails[i][j - 1].g = gNew;
                    cellDetails[i][j - 1].h = hNew;
                    cellDetails[i][j - 1].parent_i = i;
                    cellDetails[i][j - 1].parent_j = j;
                }
            }
        }

        //----------- 5th Successor (North-East)
        //------------

        // Only process this cell if this is a valid one
        if (is_valid(i - 1, j + 1) == true) {
            // If the destination cell is the same as the
            // current successor
            if (is_destination(i - 1, j + 1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i - 1][j + 1].parent_i = i;
                cellDetails[i - 1][j + 1].parent_j = j;
                printf("The destination cell is found\n");
                generate_path(cellDetails, dest);
                found_destination = true;
                return;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i - 1][j + 1] == false
                     && is_obstacle(grid, i - 1, j + 1)
                            == true) {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i - 1, j + 1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i - 1][j + 1].f == FLT_MAX
                    || cellDetails[i - 1][j + 1].f > fNew) {
                    open_list.insert(make_pair(
                        fNew, make_pair(i - 1, j + 1)));

                    // Update the details of this cell
                    cellDetails[i - 1][j + 1].f = fNew;
                    cellDetails[i - 1][j + 1].g = gNew;
                    cellDetails[i - 1][j + 1].h = hNew;
                    cellDetails[i - 1][j + 1].parent_i = i;
                    cellDetails[i - 1][j + 1].parent_j = j;
                }
            }
        }

        //----------- 6th Successor (North-West)
        //------------

        // Only process this cell if this is a valid one
        if (is_valid(i - 1, j - 1) == true) {
            // If the destination cell is the same as the
            // current successor
            if (is_destination(i - 1, j - 1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i - 1][j - 1].parent_i = i;
                cellDetails[i - 1][j - 1].parent_j = j;
                printf("The destination cell is found\n");
                generate_path(cellDetails, dest);
                found_destination = true;
                return;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i - 1][j - 1] == false
                     && is_obstacle(grid, i - 1, j - 1)
                            == true) {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i - 1, j - 1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i - 1][j - 1].f == FLT_MAX
                    || cellDetails[i - 1][j - 1].f > fNew) {
                    open_list.insert(make_pair(
                        fNew, make_pair(i - 1, j - 1)));
                    // Update the details of this cell
                    cellDetails[i - 1][j - 1].f = fNew;
                    cellDetails[i - 1][j - 1].g = gNew;
                    cellDetails[i - 1][j - 1].h = hNew;
                    cellDetails[i - 1][j - 1].parent_i = i;
                    cellDetails[i - 1][j - 1].parent_j = j;
                }
            }
        }

        //----------- 7th Successor (South-East)
        //------------

        // Only process this cell if this is a valid one
        if (is_valid(i + 1, j + 1) == true) {
            // If the destination cell is the same as the
            // current successor
            if (is_destination(i + 1, j + 1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i + 1][j + 1].parent_i = i;
                cellDetails[i + 1][j + 1].parent_j = j;
                printf("The destination cell is found\n");
                generate_path(cellDetails, dest);
                found_destination = true;
                return;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i + 1][j + 1] == false
                     && is_obstacle(grid, i + 1, j + 1)
                            == true) {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i + 1, j + 1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i + 1][j + 1].f == FLT_MAX
                    || cellDetails[i + 1][j + 1].f > fNew) {
                    open_list.insert(make_pair(
                        fNew, make_pair(i + 1, j + 1)));

                    // Update the details of this cell
                    cellDetails[i + 1][j + 1].f = fNew;
                    cellDetails[i + 1][j + 1].g = gNew;
                    cellDetails[i + 1][j + 1].h = hNew;
                    cellDetails[i + 1][j + 1].parent_i = i;
                    cellDetails[i + 1][j + 1].parent_j = j;
                }
            }
        }

        //----------- 8th Successor (South-West)
        //------------

        // Only process this cell if this is a valid one
        if (is_valid(i + 1, j - 1) == true) {
            // If the destination cell is the same as the
            // current successor
            if (is_destination(i + 1, j - 1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i + 1][j - 1].parent_i = i;
                cellDetails[i + 1][j - 1].parent_j = j;
                printf("The destination cell is found\n");
                generate_path(cellDetails, dest);
                found_destination = true;
                return;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i + 1][j - 1] == false
                     && is_obstacle(grid, i + 1, j - 1)
                            == true) {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i + 1, j - 1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is
                // better, using 'f' cost as the measure.
                if (cellDetails[i + 1][j - 1].f == FLT_MAX
                    || cellDetails[i + 1][j - 1].f > fNew) {
                    open_list.insert(make_pair(
                        fNew, make_pair(i + 1, j - 1)));

                    // Update the details of this cell
                    cellDetails[i + 1][j - 1].f = fNew;
                    cellDetails[i + 1][j - 1].g = gNew;
                    cellDetails[i + 1][j - 1].h = hNew;
                    cellDetails[i + 1][j - 1].parent_i = i;
                    cellDetails[i + 1][j - 1].parent_j = j;
                }
            }
        }
    }

    // When the destination cell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destination cell. This may happen when the
    // there is no way to destination cell (due to
    // blockages)
    if (found_destination == false)
        printf("Failed to find the Destination Cell\n");

    return;
}

