struct AStarNode {
    octomap::point3d pos;
    float g_cost;      // cost from start
    float h_cost;      // heuristic to goal
    float f_cost() const { return g_cost + h_cost; }
    std::shared_ptr<AStarNode> parent;
    
    bool operator>(const AStarNode& other) const {
        return f_cost() > other.f_cost();
    }
};

float traversal_cost(octomap::point3d p, DynamicEDTOctomap& edt, 
                     float min_clearance) {
    float clearance = edt.getDistance(p);
    if (clearance < 0) return std::numeric_limits<float>::infinity(); // unknown
    if (clearance < min_clearance) return std::numeric_limits<float>::infinity(); // blocked
    
    // cost inversely proportional to clearance
    // encourages paths far from obstacles
    return 1.0f + (min_clearance / clearance);
}

float heuristic(octomap::point3d a, octomap::point3d b) {
    return (a - b).norm();  // Euclidean distance
}

nav_msgs::Path astar(octomap::point3d start, octomap::point3d goal,
                     octomap::OcTree& tree, DynamicEDTOctomap& edt,
                     float resolution, float min_clearance) {
    
    using NodePtr = std::shared_ptr<AStarNode>;
    
    // min heap ordered by f_cost
    std::priority_queue<NodePtr, std::vector<NodePtr>, 
                        [](NodePtr a, NodePtr b){ return *a > *b; }> open;
    std::unordered_map<std::string, float> closed;  // visited nodes
    
    auto start_node = std::make_shared<AStarNode>();
    start_node->pos = start;
    start_node->g_cost = 0;
    start_node->h_cost = heuristic(start, goal);
    open.push(start_node);
    
    // 26-connected 3D grid neighbors
    std::vector<octomap::point3d> offsets;
    for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
            for (int dz = -1; dz <= 1; dz++)
                if (dx || dy || dz)
                    offsets.push_back({dx*resolution, dy*resolution, dz*resolution});
    
    while (!open.empty()) {
        NodePtr current = open.top(); open.pop();
        
        // reached goal
        if ((current->pos - goal).norm() < resolution) {
            return reconstruct_path(current);
        }
        
        std::string key = to_key(current->pos, resolution);
        if (closed.count(key)) continue;
        closed[key] = current->g_cost;
        
        for (auto& offset : offsets) {
            octomap::point3d neighbor_pos = current->pos + offset;
            float cost = traversal_cost(neighbor_pos, edt, min_clearance);
            if (cost == std::numeric_limits<float>::infinity()) continue;
            
            std::string nkey = to_key(neighbor_pos, resolution);
            float new_g = current->g_cost + cost * offset.norm();
            if (closed.count(nkey) && closed[nkey] <= new_g) continue;
            
            auto neighbor = std::make_shared<AStarNode>();
            neighbor->pos = neighbor_pos;
            neighbor->g_cost = new_g;
            neighbor->h_cost = heuristic(neighbor_pos, goal);
            neighbor->parent = current;
            open.push(neighbor);
        }
    }
    
    // no path found
    nav_msgs::Path empty;
    return empty;
}

nav_msgs::Path prune_path(nav_msgs::Path& raw_path, 
                          DynamicEDTOctomap& edt,
                          float min_clearance) {
    nav_msgs::Path pruned;
    if (raw_path.poses.empty()) return pruned;
    
    pruned.poses.push_back(raw_path.poses.front());
    int anchor = 0;
    
    for (int i = 2; i < raw_path.poses.size(); i++) {
        // check if we can go directly from anchor to i
        // by sampling the line between them
        bool direct_clear = true;
        auto& a = raw_path.poses[anchor].pose.position;
        auto& b = raw_path.poses[i].pose.position;
        
        int steps = 10;
        for (int s = 1; s <= steps; s++) {
            float t = (float)s / steps;
            octomap::point3d p(
                a.x + t*(b.x - a.x),
                a.y + t*(b.y - a.y),
                a.z + t*(b.z - a.z)
            );
            if (edt.getDistance(p) < min_clearance) {
                direct_clear = false;
                break;
            }
        }
        
        if (!direct_clear) {
            // cannot go direct, keep waypoint i-1
            pruned.poses.push_back(raw_path.poses[i-1]);
            anchor = i-1;
        }
    }
    pruned.poses.push_back(raw_path.poses.back());
    return pruned;
}
