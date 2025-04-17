#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <chrono>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <map>
#include <functional>
#include <cmath>
#include <mutex>

namespace fs = std::filesystem;

// Portal structure for sector boundaries
struct Portal {
    int x, y;
    Portal() : x(0), y(0) {} // Default constructor
    Portal(int x_, int y_) : x(x_), y(y_) {}
    bool operator==(const Portal& other) const { return x == other.x && y == other.y; }
    bool operator!=(const Portal& other) const { return !(*this == other); }
    bool operator<(const Portal& other) const { return x < other.x || (x == other.x && y < other.y); }
};

struct PortalHash {
    size_t operator()(const Portal& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};

// Edge structure for intra- and inter-sector connections
struct Edge {
    Portal src, dst;
    float cost;
    Edge(const Portal& s, const Portal& d, float c) : src(s), dst(d), cost(c) {}
};

// Sector structure
struct Sector {
    int x, y, width, height;
    std::vector<Portal> portals;
    std::vector<Edge> intra_edges;
    std::vector<Edge> inter_edges;
};

// Level structure for hierarchy
struct Level {
    int default_sector_size;
    std::vector<Sector> sectors;
};

// Node structure (grid-level)
struct Node {
    int x, y;
    bool walkable;
    int g, h, f;
    Node* parent;
    enum Direction { NONE, NORTH, SOUTH, EAST, WEST, NE, NW, SE, SW };
    Direction cameFrom;
    int obstacle_proximity;

    Node(int x_, int y_, bool walkable_) : x(x_), y(y_), walkable(walkable_),
        g(0), h(0), f(0), parent(nullptr), cameFrom(NONE), obstacle_proximity(0) {}
    
    bool operator>(const Node& other) const { 
        return f > other.f || (f == other.f && g > other.g); // Lower g-score tiebreaker
    }
    bool operator==(const Node& other) const { return x == other.x && y == other.y; }
};

namespace std {
    template<> struct hash<Node> {
        size_t operator()(const Node& n) const {
            return hash<int>()(n.x) ^ (hash<int>()(n.y) << 1);
        }
    };
    template<> struct hash<Node*> {
        size_t operator()(const Node* n) const {
            return hash<int>()(n->x) ^ (hash<int>()(n->y) << 1);
        }
    };
}

struct PerformanceMetrics {
    double time_ms;
    int expanded;
    int generated;
    int open_size;
    int path_length;
    double optimal_length;
    double quality;
    std::string scenario;

    PerformanceMetrics() : time_ms(0), expanded(0), generated(0), open_size(0), 
                          path_length(0), optimal_length(1.0), quality(0), scenario("") {}
};

struct FailedScenario {
    std::string map_name;
    std::string scenario_str;
    std::string algorithm;
    int start_x, start_y, goal_x, goal_y;
};

template<typename T, typename Priority = int>
class PriorityQueue {
private:
    struct Element {
        Priority priority;
        T item;
        Element(Priority p, T i) : priority(p), item(i) {}
        bool operator<(const Element& other) const {
            return priority > other.priority || (priority == other.priority && item->g > other.item->g);
        }
    };
    std::priority_queue<Element> heap;
    std::unordered_map<T, Priority> priorities;
public:
    void push_or_update(T item, Priority priority) {
        auto it = priorities.find(item);
        if (it == priorities.end() || priority < it->second) {
            heap.emplace(priority, item);
            priorities[item] = priority;
        }
    }
    T extract_min() {
        while (!heap.empty()) {
            Element elem = heap.top();
            heap.pop();
            auto it = priorities.find(elem.item);
            if (it != priorities.end() && it->second == elem.priority) {
                priorities.erase(it);
                return elem.item;
            }
        }
        throw std::runtime_error("Priority queue is empty");
    }
    bool empty() const { return priorities.empty(); }
    size_t size() const { return priorities.size(); }
};

class Grid {
public:
    Grid(const std::string& mapFile) : width_(0), height_(0), generations_(0) {
        std::ifstream file(mapFile);
        if (!file) {
            std::cerr << "Failed to open map file: " << mapFile << "\n";
            return;
        }
        
        std::string line;
        if (!std::getline(file, line) || line != "type octile") {
            std::cerr << "Error: Invalid map format (expected 'type octile') in " << mapFile << "\n";
            return;
        }
        
        try {
            if (!std::getline(file, line)) throw std::runtime_error("Error reading height");
            height_ = std::stoi(line.substr(7));
            if (!std::getline(file, line)) throw std::runtime_error("Error reading width");
            width_ = std::stoi(line.substr(6));
        } catch (const std::exception& e) {
            std::cerr << "Error parsing dimensions in " << mapFile << ": " << e.what() << "\n";
            return;
        }
        
        if (!std::getline(file, line) || line != "map") {
            std::cerr << "Error: Expected 'map' line in " << mapFile << "\n";
            return;
        }
        
        std::cout << "Grid size: " << width_ << "x" << height_ << "\n";
        
        grid_.resize(height_, std::vector<Node*>(width_, nullptr));
        bits_.resize(width_ * height_, false);
        for (int y = 0; y < height_; ++y) {
            if (!std::getline(file, line)) {
                std::cerr << "Error reading map data at line " << y << " in " << mapFile << "\n";
                return;
            }
            for (int x = 0; x < width_ && x < line.size(); ++x) {
                bool walkable = (line[x] == '.'); // Only '.' is walkable
                grid_[y][x] = new Node(x, y, walkable);
                bits_[y * width_ + x] = walkable;
            }
        }
        
        // Compute obstacle proximity
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                Node* node = grid_[y][x];
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        Node* neighbor = getNode(x + dx, y + dy);
                        if (neighbor && !neighbor->walkable) node->obstacle_proximity++;
                    }
                }
            }
        }
        
        // Build hierarchy
        hierarchy_ = BuildHierarchy();
        std::cout << "Grid initialized for " << mapFile << "\n";
    }

    ~Grid() {
        for (auto& row : grid_) {
            for (Node* node : row) delete node;
        }
    }

    Node* getNode(int x, int y) {
        if (x < 0 || x >= width_ || y < 0 || y >= height_) return nullptr;
        return grid_[y][x];
    }

    bool IsTraversable(int x, int y) const {
        if (x < 0 || y < 0 || x >= width_ || y >= height_) return false;
        return bits_[y * width_ + x];
    }

    bool ValidMove(const Portal& from, const Portal& to) const {
        int dx = to.x - from.x;
        int dy = to.y - from.y;
        if (abs(dx) == 1 && abs(dy) == 1) {
            return IsTraversable(from.x + dx, from.y) && IsTraversable(from.x, from.y + dy);
        }
        return IsTraversable(to.x, to.y);
    }

    Node* findNearestWalkable(int x, int y) {
        if (Node* n = getNode(x, y); n && n->walkable) return n;
        for (int r = 1; r < std::max(width_, height_); ++r) {
            for (int dy = -r; dy <= r; ++dy) {
                for (int dx = -r; dx <= r; ++dx) {
                    if (Node* n = getNode(x + dx, y + dy); n && n->walkable) return n;
                }
            }
        }
        return nullptr;
    }

    void resetNodes() {
        for (auto& row : grid_) {
            for (Node* node : row) {
                node->g = node->h = node->f = 0;
                node->parent = nullptr;
                node->cameFrom = Node::NONE;
            }
        }
        generations_ = 0;
    }

    int heuristic(Node* a, Node* b, double weight = 1.0) {
        int dx = abs(a->x - b->x);
        int dy = abs(a->y - b->y);
        int base_h = static_cast<int>(1.0 * (dx + dy) + (1.41421 - 2.0) * std::min(dx, dy));
        weight = std::min(weight, 1.0); // Admissible heuristic
        int penalty = a->obstacle_proximity * 5;
        return static_cast<int>(weight * base_h + penalty);
    }

    std::vector<Node*> getAStarSuccessors(Node* current) {
        std::vector<Node*> successors;
        const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        for (int i = 0; i < 8; ++i) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];
            if (Node* n = getNode(nx, ny); n && n->walkable && ValidMove({current->x, current->y}, {nx, ny})) {
                successors.push_back(n);
            }
        }
        return successors;
    }

    std::vector<Node*> getCanonicalSuccessors(Node* current, Node* goal) {
        std::vector<Node*> successors;
        const int dx_goal = (goal->x > current->x) ? 1 : (goal->x < current->x) ? -1 : 0;
        const int dy_goal = (goal->y > current->y) ? 1 : (goal->y < current->y) ? -1 : 0;

        auto addSuccessor = [&](int dx, int dy) {
            if (Node* n = getNode(current->x + dx, current->y + dy); 
                n && n->walkable && ValidMove({current->x, current->y}, {n->x, n->y})) {
                if (std::find(successors.begin(), successors.end(), n) == successors.end()) {
                    successors.push_back(n);
                }
            }
        };

        addSuccessor(dx_goal, dy_goal);
        addSuccessor(dx_goal, 0);
        addSuccessor(0, dy_goal);

        if (current->cameFrom == Node::NONE) return successors;

        int dx_prev = 0, dy_prev = 0;
        switch (current->cameFrom) {
            case Node::NORTH: dy_prev = -1; break;
            case Node::SOUTH: dy_prev = 1; break;
            case Node::EAST: dx_prev = 1; break;
            case Node::WEST: dx_prev = -1; break;
            case Node::NE: dx_prev = 1; dy_prev = -1; break;
            case Node::NW: dx_prev = -1; dy_prev = -1; break;
            case Node::SE: dx_prev = 1; dy_prev = 1; break;
            case Node::SW: dx_prev = -1; dy_prev = 1; break;
            default: break;
        }

        addSuccessor(dx_prev, dy_prev);
        if (dx_prev != 0 && dy_prev == 0) {
            addSuccessor(dx_prev, -1);
            addSuccessor(dx_prev, 1);
        } else if (dx_prev == 0 && dy_prev != 0) {
            addSuccessor(-1, dy_prev);
            addSuccessor(1, dy_prev);
        } else if (dx_prev != 0 && dy_prev != 0) {
            addSuccessor(dx_prev, 0);
            addSuccessor(0, dy_prev);
        }

        return successors;
    }

    std::vector<Level> BuildHierarchy() {
        std::vector<Level> hierarchy;
        int current_size = 8; // Base sector size
        const Level* prev_level = nullptr;

        while (true) {
            Level level;
            level.default_sector_size = current_size;

            std::vector<int> x_divs, y_divs;
            for (int x = 0; x < width_; x += current_size) x_divs.push_back(x);
            for (int y = 0; y < height_; y += current_size) y_divs.push_back(y);

            std::vector<Sector> sectors(x_divs.size() * y_divs.size());
            std::mutex sector_mutex;

            for (size_t i = 0; i < sectors.size(); ++i) {
                Sector& sector = sectors[i];
                size_t index = i;
                int x = x_divs[index / y_divs.size()];
                int y = y_divs[index % y_divs.size()];
                
                sector.x = x;
                sector.y = y;
                sector.width = std::min(current_size, width_ - x);
                sector.height = std::min(current_size, height_ - y);

                // Compute portals
                std::vector<Portal> portals;
                int x_start = sector.x, x_end = sector.x + sector.width;
                int y_start = sector.y, y_end = sector.y + sector.height;

                if (y_start > 0) {
                    for (int x = x_start; x < x_end; ++x) {
                        if (IsTraversable(x, y_start) && IsTraversable(x, y_start - 1)) {
                            portals.emplace_back(x, y_start);
                        }
                    }
                }
                if (y_end < height_) {
                    for (int x = x_start; x < x_end; ++x) {
                        if (IsTraversable(x, y_end - 1) && IsTraversable(x, y_end)) {
                            portals.emplace_back(x, y_end - 1);
                        }
                    }
                }
                if (x_start > 0) {
                    for (int y = y_start; y < y_end; ++y) {
                        if (IsTraversable(x_start, y) && IsTraversable(x_start - 1, y)) {
                            portals.emplace_back(x_start, y);
                        }
                    }
                }
                if (x_end < width_) {
                    for (int y = y_start; y < y_end; ++y) {
                        if (IsTraversable(x_end - 1, y) && IsTraversable(x_end, y)) {
                            portals.emplace_back(x_end - 1, y);
                        }
                    }
                }
                sector.portals = std::move(portals);

                // Compute intra-edges
                sector.intra_edges = ComputeIntraEdges(sector);
            }

            // Compute inter-edges
            for (auto& sector : sectors) {
                sector.inter_edges = ComputeInterEdges(sector, sectors, current_size);
            }

            level.sectors = std::move(sectors);
            {
                std::lock_guard<std::mutex> lock(sector_mutex);
                hierarchy.push_back(std::move(level));
            }

            if (x_divs.size() <= 1 && y_divs.size() <= 1) break;
            prev_level = &hierarchy.back();
            current_size *= 2;
        }

        return hierarchy;
    }

    std::vector<Edge> ComputeIntraEdges(const Sector& sector) {
        std::vector<Edge> edges;
        for (size_t i = 0; i < sector.portals.size(); ++i) {
            for (size_t j = i + 1; j < sector.portals.size(); ++j) {
                auto result = Dijkstra(sector.portals[i], sector.portals[j], sector);
                if (std::isfinite(result.cost)) {
                    edges.emplace_back(sector.portals[i], sector.portals[j], result.cost);
                    edges.emplace_back(sector.portals[j], sector.portals[i], result.cost);
                }
            }
        }
        return edges;
    }

    std::vector<Edge> ComputeInterEdges(const Sector& sector, const std::vector<Sector>& sectors, int current_size) {
        std::vector<Edge> edges;
        const int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
        const int dy[] = {0, 0, 1, -1, -1, 1, 1, -1};
        const float costs[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.41421f, 1.41421f, 1.41421f, 1.41421f};

        for (int i = 0; i < 8; ++i) {
            int nx = sector.x + dx[i] * current_size;
            int ny = sector.y + dy[i] * current_size;
            if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

            auto neighbor_it = std::find_if(sectors.begin(), sectors.end(), 
                [nx, ny](const Sector& s) { return s.x == nx && s.y == ny; });
            if (neighbor_it == sectors.end()) continue;

            const Sector& neighbor = *neighbor_it;
            for (const auto& p1 : sector.portals) {
                for (const auto& p2 : neighbor.portals) {
                    if (ValidMove(p1, p2)) {
                        edges.emplace_back(p1, p2, costs[i]);
                        edges.emplace_back(p2, p1, costs[i]);
                    }
                }
            }
        }
        return edges;
    }

    struct DijkstraResult {
        float cost;
        std::vector<Portal> path;
        std::unordered_map<Portal, float, PortalHash> cumulative_costs;
    };

    DijkstraResult Dijkstra(const Portal& start, const Portal& end, const Sector& sector) {
        std::priority_queue<std::pair<float, Portal>, std::vector<std::pair<float, Portal>>, std::greater<>> pq;
        std::unordered_map<Portal, float, PortalHash> costs;
        std::unordered_map<Portal, Portal, PortalHash> predecessors;

        pq.push({0.0f, start});
        costs[start] = 0.0f;

        while (!pq.empty()) {
            auto [current_cost, current] = pq.top();
            pq.pop();

            if (current == end) {
                std::vector<Portal> path;
                Portal portal_node = end;
                while (portal_node != start) {
                    path.push_back(portal_node);
                    portal_node = predecessors.at(portal_node);
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return {current_cost, path, costs};
            }

            if (current_cost > costs[current]) continue;

            const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
            const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
            const float step_costs[] = {1.41421f, 1.0f, 1.41421f, 1.0f, 1.41421f, 1.41421f, 1.0f, 1.41421f};

            for (int i = 0; i < 8; ++i) {
                Portal neighbor{current.x + dx[i], current.y + dy[i]};
                if (neighbor.x < sector.x || neighbor.x >= sector.x + sector.width ||
                    neighbor.y < sector.y || neighbor.y >= sector.y + sector.height) continue;
                if (!ValidMove(current, neighbor)) continue;

                float new_cost = current_cost + step_costs[i];
                if (!costs.count(neighbor) || new_cost < costs[neighbor]) {
                    costs[neighbor] = new_cost;
                    predecessors[neighbor] = current;
                    pq.push({new_cost, neighbor});
                }
            }
        }

        return {std::numeric_limits<float>::infinity(), {}, {}};
    }

    const Sector* findSector(int x, int y, const Level& level) const {
        for (const auto& sector : level.sectors) {
            if (x >= sector.x && x < sector.x + sector.width &&
                y >= sector.y && y < sector.y + sector.height) {
                return &sector;
            }
        }
        return nullptr;
    }

    std::vector<Portal> getCanonicalPortalSuccessors(const Portal& current, Node* goal, const Sector& sector, const Level& level) {
        std::vector<Portal> successors;
        int dx_goal = (goal->x > current.x) ? 1 : (goal->x < current.x) ? -1 : 0;
        int dy_goal = (goal->y > current.y) ? 1 : (goal->y < current.y) ? -1 : 0;

        auto addSuccessor = [&](const Portal& p) {
            if (std::find(successors.begin(), successors.end(), p) == successors.end()) {
                successors.push_back(p);
            }
        };

        // Check intra- and inter-edges
        for (const auto& edge : sector.intra_edges) {
            if (edge.src == current) {
                int dx = edge.dst.x - current.x;
                int dy = edge.dst.y - current.y;
                if ((dx == dx_goal && dy == dy_goal) || (dx == dx_goal && dy == 0) || (dx == 0 && dy == dy_goal)) {
                    addSuccessor(edge.dst);
                }
            }
        }
        for (const auto& edge : sector.inter_edges) {
            if (edge.src == current) {
                int dx = edge.dst.x - current.x;
                int dy = edge.dst.y - current.y;
                if ((dx == dx_goal && dy == dy_goal) || (dx == dx_goal && dy == 0) || (dx == 0 && dy == dy_goal)) {
                    addSuccessor(edge.dst);
                }
            }
        }

        // Apply cameFrom restrictions
        if (portal_cameFrom_.find(current) != portal_cameFrom_.end() && portal_cameFrom_[current] != Node::NONE) {
            int dx_prev = 0, dy_prev = 0;
            switch (portal_cameFrom_[current]) {
                case Node::NORTH: dy_prev = -1; break;
                case Node::SOUTH: dy_prev = 1; break;
                case Node::EAST: dx_prev = 1; break;
                case Node::WEST: dx_prev = -1; break;
                case Node::NE: dx_prev = 1; dy_prev = -1; break;
                case Node::NW: dx_prev = -1; dy_prev = -1; break;
                case Node::SE: dx_prev = 1; dy_prev = 1; break;
                case Node::SW: dx_prev = -1; dy_prev = 1; break;
                default: break;
            }
            for (const auto& edge : sector.intra_edges) {
                if (edge.src == current) {
                    int dx = edge.dst.x - current.x;
                    int dy = edge.dst.y - current.y;
                    if ((dx == dx_prev && dy == dy_prev) ||
                        (dx_prev != 0 && dy_prev == 0 && dx == dx_prev && (dy == -1 || dy == 1)) ||
                        (dx_prev == 0 && dy_prev != 0 && dy == dy_prev && (dx == -1 || dx == 1)) ||
                        (dx_prev != 0 && dy_prev != 0 && (dx == dx_prev || dy == dy_prev))) {
                        addSuccessor(edge.dst);
                    }
                }
            }
            for (const auto& edge : sector.inter_edges) {
                if (edge.src == current) {
                    int dx = edge.dst.x - current.x;
                    int dy = edge.dst.y - current.y;
                    if ((dx == dx_prev && dy == dy_prev) ||
                        (dx_prev != 0 && dy_prev == 0 && dx == dx_prev && (dy == -1 || dy == 1)) ||
                        (dx_prev == 0 && dy_prev != 0 && dy == dy_prev && (dx == -1 || dx == 1)) ||
                        (dx_prev != 0 && dy_prev != 0 && (dx == dx_prev || dy == dy_prev))) {
                        addSuccessor(edge.dst);
                    }
                }
            }
        }

        return successors;
    }

    PerformanceMetrics runSearch(Node* start, Node* goal, double optimal, double weight, int algo, 
                                std::ofstream& detailedFile, std::ofstream& scenarioFile, 
                                const std::string& scenarioStr, const std::string& mapName, 
                                std::vector<FailedScenario>& failedScenarios) {
        start = findNearestWalkable(start->x, start->y);
        goal = findNearestWalkable(goal->x, goal->y);
        if (!start || !goal) {
            std::cerr << "No valid start or goal found for scenario " << scenarioStr << "\n";
            FailedScenario failure;
            failure.map_name = mapName;
            failure.scenario_str = scenarioStr;
            failure.algorithm = (algo == 0) ? "A*" : "CA*";
            failure.start_x = start ? start->x : -1;
            failure.start_y = start ? start->y : -1;
            failure.goal_x = goal ? goal->x : -1;
            failure.goal_y = goal ? goal->y : -1;
            failedScenarios.push_back(failure);
            PerformanceMetrics metrics;
            metrics.optimal_length = optimal;
            metrics.scenario = scenarioStr;
            return metrics;
        }

        PerformanceMetrics metrics;
        metrics.scenario = scenarioStr;
        auto start_time = std::chrono::high_resolution_clock::now();

        // Use top-level hierarchy for high-level search
        const Level& top_level = hierarchy_.back();
        const Sector* start_sector = findSector(start->x, start->y, top_level);
        const Sector* goal_sector = findSector(goal->x, goal->y, top_level);

        if (!start_sector || !goal_sector) {
            std::cerr << "Start or goal sector not found\n";
            return runGridSearch(start, goal, optimal, weight, algo, detailedFile, scenarioFile, scenarioStr, mapName, failedScenarios);
        }

        // High-level CA* search
        std::unordered_map<Portal, float, PortalHash> costs;
        std::unordered_map<Portal, Portal, PortalHash> predecessors;
        portal_cameFrom_.clear();
        std::priority_queue<std::pair<float, Portal>, std::vector<std::pair<float, Portal>>, std::greater<>> pq;
        std::unordered_set<Portal, PortalHash> closed;

        // Add start portals
        for (const auto& p : start_sector->portals) {
            float dist = std::hypot(start->x - p.x, start->y - p.y);
            costs[p] = dist;
            pq.push({dist, p});
            portal_cameFrom_[p] = Node::NONE;
        }

        bool path_found = false;
        Portal end_portal(0, 0);
        while (!pq.empty()) {
            auto [current_cost, current] = pq.top();
            pq.pop();

            if (closed.count(current)) continue;
            closed.insert(current);

            if (std::find(goal_sector->portals.begin(), goal_sector->portals.end(), current) != goal_sector->portals.end()) {
                end_portal = current;
                path_found = true;
                break;
            }

            // Canonical successor generation
            std::vector<Portal> successors;
            if (algo == 1) {
                successors = getCanonicalPortalSuccessors(current, goal, *findSector(current.x, current.y, top_level), top_level);
            } else {
                for (const auto& edge : findSector(current.x, current.y, top_level)->intra_edges) {
                    if (edge.src == current) successors.push_back(edge.dst);
                }
                for (const auto& edge : findSector(current.x, current.y, top_level)->inter_edges) {
                    if (edge.src == current) successors.push_back(edge.dst);
                }
            }

            for (const auto& successor : successors) {
                float edge_cost = 0.0f;
                for (const auto& edge : findSector(current.x, current.y, top_level)->intra_edges) {
                    if (edge.src == current && edge.dst == successor) { edge_cost = edge.cost; break; }
                }
                for (const auto& edge : findSector(current.x, current.y, top_level)->inter_edges) {
                    if (edge.src == current && edge.dst == successor) { edge_cost = edge.cost; break; }
                }
                float new_cost = current_cost + edge_cost;
                if (!costs.count(successor) || new_cost < costs[successor] * 0.9) {
                    costs[successor] = new_cost;
                    predecessors[successor] = current;
                    if (algo == 1) {
                        int dx = successor.x - current.x;
                        int dy = successor.y - current.y;
                        portal_cameFrom_[successor] = 
                            dx == 1 ? (dy == -1 ? Node::NE : dy == 1 ? Node::SE : Node::EAST) :
                            dx == -1 ? (dy == -1 ? Node::NW : dy == 1 ? Node::SW : Node::WEST) :
                            dy == 1 ? Node::SOUTH : dy == -1 ? Node::NORTH : Node::NONE;
                    }
                    pq.push({new_cost, successor});
                }
            }
        }

        if (!path_found) {
            std::cout << "No high-level path found, falling back to grid search\n";
            return runGridSearch(start, goal, optimal, weight, algo, detailedFile, scenarioFile, scenarioStr, mapName, failedScenarios);
        }

        // Reconstruct high-level path
        std::vector<Portal> portal_path;
        Portal portal_node = end_portal;
        while (costs.count(portal_node)) {
            portal_path.push_back(portal_node);
            if (!predecessors.count(portal_node)) break;
            portal_node = predecessors[portal_node];
        }
        std::reverse(portal_path.begin(), portal_path.end());

        // Low-level CA* pathfinding between portals
        std::vector<Node*> final_path;
        Node* current = start;
        for (size_t i = 0; i < portal_path.size(); ++i) {
            Node* next = getNode(portal_path[i].x, portal_path[i].y);
            auto segment = runGridSearch(current, next, optimal, weight, algo, detailedFile, scenarioFile, scenarioStr, mapName, failedScenarios);
            if (segment.path_length == 0) {
                std::cout << "Failed to find segment path\n";
                return metrics;
            }
            Node* node = next;
            std::vector<Node*> segment_path;
            while (node != current) {
                segment_path.push_back(node);
                node = node->parent;
            }
            std::reverse(segment_path.begin(), segment_path.end());
            final_path.insert(final_path.end(), segment_path.begin(), segment_path.end());
            current = next;
            metrics.expanded += segment.expanded;
            metrics.generated += segment.generated;
        }
        // Final segment to goal
        auto final_segment = runGridSearch(current, goal, optimal, weight, algo, detailedFile, scenarioFile, scenarioStr, mapName, failedScenarios);
        if (final_segment.path_length == 0) {
            std::cout << "Failed to find final segment path\n";
            return metrics;
        }
        Node* node = goal;
        std::vector<Node*> final_segment_path;
        while (node != current) {
            final_segment_path.push_back(node);
            node = node->parent;
        }
        std::reverse(final_segment_path.begin(), final_segment_path.end());
        final_path.insert(final_path.end(), final_segment_path.begin(), final_segment_path.end());
        metrics.expanded += final_segment.expanded;
        metrics.generated += final_segment.generated;

        // Compute metrics
        metrics.time_ms = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - start_time).count();
        metrics.path_length = 0;
        for (size_t i = 1; i < final_path.size(); ++i) {
            int dx = final_path[i]->x - final_path[i-1]->x;
            int dy = final_path[i]->y - final_path[i-1]->y;
            metrics.path_length += (dx != 0 && dy != 0) ? 14 : 10;
        }
        metrics.optimal_length = optimal;
        metrics.quality = (metrics.optimal_length > 0.0001) ? 
            static_cast<double>(metrics.path_length) / metrics.optimal_length : 0.0;
        metrics.open_size = 0;

        std::string algoName = (algo == 0) ? "A*" : "CA*";
        std::cout << "Scenario: map = " << mapName << ".map, start = (" << start->x << "," << start->y 
                  << "), goal = (" << goal->x << "," << goal->y << ")\n";
        std::cout << "Running " << algoName << " with hierarchy...\n";
        std::cout << "Path found! Time: " << metrics.time_ms << " ms, Path length: " << metrics.path_length << "\n";

        scenarioFile << "Scenario: map = " << mapName << ".map, start = (" << start->x << "," << start->y 
                     << "), goal = (" << goal->x << "," << goal->y << ")\n";
        scenarioFile << "Algorithm: " << algoName << "\n";
        scenarioFile << "Weight: " << weight << "\n";
        scenarioFile << "Time: " << metrics.time_ms << " ms\n";
        scenarioFile << "Path length: " << metrics.path_length << "\n";
        scenarioFile << "Quality: " << metrics.quality << "\n";
        scenarioFile << "\n";

        detailedFile << scenarioStr << "," << weight << "," << algoName << ","
                     << metrics.time_ms << "," << metrics.expanded << "," << metrics.generated << ","
                     << metrics.open_size << "," << metrics.quality << "," << metrics.path_length << "\n";
        return metrics;
    }

private:
    PerformanceMetrics runGridSearch(Node* start, Node* goal, double optimal, double weight, int algo, 
                                    std::ofstream& detailedFile, std::ofstream& scenarioFile, 
                                    const std::string& scenarioStr, const std::string& mapName, 
                                    std::vector<FailedScenario>& failedScenarios) {
        PerformanceMetrics metrics;
        metrics.scenario = scenarioStr;
        auto start_time = std::chrono::high_resolution_clock::now();

        PriorityQueue<Node*> open;
        std::unordered_set<Node*> closed;

        start->g = 0;
        start->h = heuristic(start, goal, weight);
        start->f = start->g + start->h;
        open.push_or_update(start, start->f);
        metrics.generated = 1;
        bool retry_with_astar = false;

        while (!open.empty()) {
            Node* current = open.extract_min();
            metrics.expanded++;

            if (current == goal) {
                metrics.time_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::high_resolution_clock::now() - start_time).count();
                
                std::vector<std::string> directions;
                Node* node = current;
                metrics.path_length = 0;
                while (node != nullptr) {
                    if (node->parent) {
                        int dx = node->x - node->parent->x;
                        int dy = node->y - node->parent->y;
                        metrics.path_length += (dx != 0 && dy != 0) ? 14 : 10;
                    }
                    node = node->parent;
                }

                metrics.optimal_length = optimal;
                metrics.quality = (metrics.optimal_length > 0.0001) ? 
                    static_cast<double>(metrics.path_length) / metrics.optimal_length : 0.0;
                metrics.open_size = open.size();
                return metrics;
            }

            closed.insert(current);

            std::vector<Node*> successors;
            if (algo == 0 || (algo == 1 && (retry_with_astar || current->obstacle_proximity > 0)))
                successors = getAStarSuccessors(current);
            else
                successors = getCanonicalSuccessors(current, goal);

            for (Node* successor : successors) {
                int tentative_g = current->g + 
                    ((abs(successor->x - current->x) + abs(successor->y - current->y) == 2) ? 14 : 10);
                if (closed.count(successor) && tentative_g >= successor->g * 0.9) continue;

                if (!successor->parent || tentative_g < successor->g) {
                    successor->parent = current;
                    successor->g = tentative_g;
                    successor->h = heuristic(successor, goal, weight);
                    successor->f = successor->g + successor->h;

                    if (algo == 1) {
                        const int dx = successor->x - current->x;
                        const int dy = successor->y - current->y;
                        successor->cameFrom = 
                            dx == 1 ? (dy == -1 ? Node::NE : dy == 1 ? Node::SE : Node::EAST) :
                            dx == -1 ? (dy == -1 ? Node::NW : dy == 1 ? Node::SW : Node::WEST) :
                            dy == 1 ? Node::SOUTH : dy == -1 ? Node::NORTH : Node::NONE;
                    }

                    open.push_or_update(successor, successor->f);
                    if (!closed.count(successor)) metrics.generated++;
                    else closed.erase(successor);
                }
            }

            if (algo == 1 && open.empty() && !retry_with_astar) {
                retry_with_astar = true;
                open.push_or_update(start, start->f);
                closed.clear();
                resetNodes();
                start->g = 0;
                start->h = heuristic(start, goal, weight);
                start->f = start->g + start->h;
                metrics.generated = 1;
            }
        }

        metrics.time_ms = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - start_time).count();
        metrics.path_length = 0;
        metrics.quality = 0.0;
        metrics.optimal_length = optimal;
        metrics.open_size = open.size();

        std::string algoName = (algo == 0) ? "A*" : "CA*";
        FailedScenario failure;
        failure.map_name = mapName;
        failure.scenario_str = scenarioStr;
        failure.algorithm = algoName;
        failure.start_x = start->x;
        failure.start_y = start->y;
        failure.goal_x = goal->x;
        failure.goal_y = goal->y;
        failedScenarios.push_back(failure);

        return metrics;
    }

    int width_, height_;
    std::vector<std::vector<Node*>> grid_;
    std::vector<bool> bits_;
    std::vector<Level> hierarchy_;
    int generations_;
    std::unordered_map<Portal, Node::Direction, PortalHash> portal_cameFrom_;
};

struct Scenario {
    std::string map;
    int startX, startY, goalX, goalY;
    double optimalLength;
};

std::vector<Scenario> loadScenarios(const std::string& scenFile) {
    std::vector<Scenario> scenarios;
    std::ifstream file(scenFile);
    if (!file) {
        std::cerr << "Failed to open scenario file: " << scenFile << "\n";
        return scenarios;
    }

    std::string line;
    std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        Scenario s;
        int bucket, width, height;
        if (iss >> bucket >> s.map >> width >> height >> s.startX >> s.startY >> s.goalX >> s.goalY >> s.optimalLength) {
            if (s.startX == s.goalX && s.startY == s.goalY) continue;
            if (s.optimalLength < 0.0001) s.optimalLength = 1.0;
            scenarios.push_back(s);
        }
    }
    std::cout << "Loaded " << scenarios.size() << " scenarios.\n";
    return scenarios;
}

std::string findScenarioFile(const std::string& scenFolder, const std::string& mapName) {
    for (const auto& entry : fs::directory_iterator(scenFolder)) {
        std::string filename = entry.path().filename().string();
        if (filename.find(mapName) != std::string::npos && 
            filename.find(".map.scen") != std::string::npos) {
            return entry.path().string();
        }
    }
    return "";
}

int main() {
    const std::string mapFolder = "/Users/noshinnawal/Desktop/JPS_Pathfinding/starcraft/sc1-map/";
    const std::string scenFolder = "/Users/noshinnawal/Desktop/JPS_Pathfinding/starcraft/sc1-scen/";
    const std::string outputFolder = "/Users/noshinnawal/Desktop/JPS_Pathfinding/starcraft/output/";
    
    if (!fs::exists(mapFolder)) {
        std::cerr << "Map folder not found: " << mapFolder << "\n";
        return 1;
    }
    if (!fs::exists(outputFolder)) fs::create_directories(outputFolder);

    std::string detailedFilePath = outputFolder + "detailed_results.csv";
    std::ofstream detailedFile(detailedFilePath);
    if (!detailedFile.is_open()) {
        std::cerr << "Failed to open detailed results file: " << detailedFilePath << "\n";
        return 1;
    }
    detailedFile << "Scenario,Weight,Algorithm,Time (ms),Expansions,Generations,Open,Quality,Path\n";

    std::vector<FailedScenario> failedScenarios;
    std::vector<double> weights = {1.0}; // Admissible heuristic
    std::map<std::string, std::map<double, std::vector<PerformanceMetrics>>> results;

    int totalMaps = 0;
    for (const auto& entry : fs::directory_iterator(mapFolder)) {
        if (entry.path().extension() == ".map") totalMaps++;
    }
    std::cout << "Total maps found: " << totalMaps << "\n";
    int mapCount = 0;

    for (const auto& entry : fs::directory_iterator(mapFolder)) {
        if (entry.path().extension() != ".map") continue;
        mapCount++;
        const auto mapPath = entry.path();
        std::string mapName = mapPath.stem().string();
        std::cout << "Processing map " << mapCount << "/" << totalMaps << ": " << mapName << "\n";
        Grid grid(mapPath.string());
        if (!grid.getNode(0, 0)) {
            std::cerr << "Failed to initialize grid for " << mapName << "\n";
            continue;
        }

        std::string scenFile = findScenarioFile(scenFolder, mapName);
        if (scenFile.empty()) {
            std::cerr << "No scenario file found for " << mapName << "\n";
            continue;
        }

        std::string scenarioFilePath = outputFolder + mapName + ".map.scen.details.txt";
        std::ofstream scenarioFile(scenarioFilePath);
        if (!scenarioFile.is_open()) {
            std::cerr << "Failed to open scenario details file: " << scenarioFilePath << "\n";
            continue;
        }

        auto scenarios = loadScenarios(scenFile);
        if (scenarios.empty()) {
            std::cerr << "No valid scenarios for " << mapName << "\n";
            scenarioFile.close();
            continue;
        }

        // Select 10 scenarios: first 2, last 2, and 6 from the middle
        std::vector<size_t> selected_indices;
        size_t total_scenarios = scenarios.size();
        if (total_scenarios <= 10) {
            // If 10 or fewer scenarios, use all
            for (size_t i = 0; i < total_scenarios; ++i) {
                selected_indices.push_back(i);
            }
        } else {
            // First 2 scenarios
            selected_indices.push_back(0);
            selected_indices.push_back(1);
            // Last 2 scenarios
            selected_indices.push_back(total_scenarios - 1);
            selected_indices.push_back(total_scenarios - 2);
            // 6 scenarios from the middle
            size_t mid = total_scenarios / 2;
            // Select indices around the middle (e.g., mid-3 to mid+2 for 6 scenarios)
            for (size_t i = mid - 3; i <= mid + 2; ++i) {
                if (i < total_scenarios && i >= 2 && i < total_scenarios - 2) { // Avoid overlap with first/last
                    selected_indices.push_back(i);
                }
            }
            // Ensure exactly 10 scenarios if possible
            while (selected_indices.size() < 10 && mid + 3 < total_scenarios - 2) {
                selected_indices.push_back(mid + 3);
                mid++;
            }
        }

        std::cout << "Selected " << selected_indices.size() << " scenarios for processing.\n";

        std::unordered_set<std::string> processedScenarios;
        for (size_t idx : selected_indices) {
            const auto& scenario = scenarios[idx];
            std::string scenarioKey = std::to_string(scenario.startX) + "," + 
                                    std::to_string(scenario.startY) + "," + 
                                    std::to_string(scenario.goalX) + "," + 
                                    std::to_string(scenario.goalY);
            if (processedScenarios.find(scenarioKey) != processedScenarios.end()) continue;
            processedScenarios.insert(scenarioKey);

            std::ostringstream scenarioStr;
            scenarioStr << (idx + 1) << "/" << total_scenarios << " (" 
                        << scenario.startX << "," << scenario.startY << ") to (" 
                        << scenario.goalX << "," << scenario.goalY << ")";
            std::cout << "  Scenario " << scenarioStr.str() << "\n";
            Node* start = grid.getNode(scenario.startX, scenario.startY);
            Node* goal = grid.getNode(scenario.goalX, scenario.goalY);
            
            if (start && goal) {
                start->walkable = true;
                goal->walkable = true;
                for (double w : weights) {
                    grid.resetNodes();
                    auto metrics = grid.runSearch(start, goal, scenario.optimalLength, w, 1, 
                                                 detailedFile, scenarioFile, scenarioStr.str(), 
                                                 mapName, failedScenarios);
                    metrics.scenario = scenarioStr.str();
                    results["CA*"][w].push_back(metrics);
                }
            }
        }

        scenarioFile.close();
        std::cout << "Scenario details saved to " << scenarioFilePath << "\n";
    }

    detailedFile.close();
    std::cout << "Detailed results saved to " << detailedFilePath << "\n";

    std::string failedFilePath = outputFolder + "failed_scenarios.csv";
    std::ofstream failedFile(failedFilePath);
    if (!failedFile.is_open()) {
        std::cerr << "Failed to open failed scenarios file: " << failedFilePath << "\n";
        return 1;
    }
    failedFile << "Map,Scenario,Algorithm,StartX,StartY,GoalX,GoalY\n";
    for (const auto& failure : failedScenarios) {
        failedFile << failure.map_name << "," << failure.scenario_str << "," << failure.algorithm << ","
                   << failure.start_x << "," << failure.start_y << "," 
                   << failure.goal_x << "," << failure.goal_y << "\n";
    }
    failedFile.close();
    std::cout << "Failed scenarios saved to " << failedFilePath << "\n";

    std::string tableFilePath = outputFolder + "performance_table.csv";
    std::ofstream tableFile(tableFilePath);
    if (!tableFile.is_open()) {
        std::cerr << "Failed to open table file: " << tableFilePath << "\n";
        return 1;
    }

    tableFile << "Weight,CA*_Time(ms),CA*_Expansions,CA*_Generations,CA*_Open,CA*_Quality\n";

    for (double w : weights) {
        tableFile << std::fixed << std::setprecision(3) << w;
        
        auto& metrics = results["CA*"][w];
        double avg_time = 0, avg_exp = 0, avg_gen = 0, avg_open = 0, avg_quality = 0;
        int valid_count = 0;
        
        for (const auto& m : metrics) {
            if (m.path_length > 0) {
                avg_time += m.time_ms;
                avg_exp += m.expanded;
                avg_gen += m.generated;
                avg_open += m.open_size;
                avg_quality += m.quality;
                valid_count++;
            }
        }

        if (valid_count > 0) {
            avg_time /= valid_count;
            avg_exp /= valid_count;
            avg_gen /= valid_count;
            avg_open /= valid_count;
            avg_quality /= valid_count;
        }

        tableFile << "," << avg_time << "," << avg_exp << "," << avg_gen << "," 
                  << avg_open << "," << avg_quality << "\n";
    }

    tableFile.close();
    std::cout << "Performance table saved to " << tableFilePath << "\n";
    return 0;
}