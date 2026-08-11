#include "path_searching/dyn_a_star.h"
#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    adjusted_start_ = start_pt;
    adjusted_end_ = end_pt;
    initial_start_occ_ = initial_end_occ_ = 0;
    start_adjust_steps_ = end_adjust_steps_ = 0;
    init_failure_reason_ = "none";

    if (!Coord2Index(start_pt, start_idx))
    {
        init_failure_reason_ = "requested start outside A-star pool";
        return false;
    }
    if (!Coord2Index(end_pt, end_idx))
    {
        init_failure_reason_ = "requested end outside A-star pool";
        return false;
    }

    Eigen::Vector3d start_to_end = end_pt - start_pt;
    if (start_to_end.norm() < 1e-6)
    {
        init_failure_reason_ = "start and end are coincident";
        return false;
    }
    const double path_yaw = std::atan2(start_to_end(1), start_to_end(0));
    start_to_end.normalize();

    int occ = checkOccupancy(Index2Coord(start_idx), path_yaw);
    initial_start_occ_ = occ;
    if (occ)
    {
        //ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt -= start_to_end * step_size_;
            ++start_adjust_steps_;
            adjusted_start_ = start_pt;
            if (!Coord2Index(start_pt, start_idx))
            {
                init_failure_reason_ = "occupied start could not be moved to a free point before pool boundary";
                return false;
            }

            occ = checkOccupancy(Index2Coord(start_idx), path_yaw);
            if (occ == -1)
            {
                init_failure_reason_ = "start adjustment left the sliding occupancy map";
                return false;
            }
        } while (occ);
    }

    occ = checkOccupancy(Index2Coord(end_idx), path_yaw);
    initial_end_occ_ = occ;
    if (occ)
    {
        //ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt += start_to_end * step_size_;
            ++end_adjust_steps_;
            adjusted_end_ = end_pt;
            if (!Coord2Index(end_pt, end_idx))
            {
                init_failure_reason_ = "occupied end could not be moved to a free point before pool boundary";
                return false;
            }

            occ = checkOccupancy(Index2Coord(end_idx), path_yaw);
            if (occ == -1)
            {
                init_failure_reason_ = "end adjustment left the sliding occupancy map";
                return false;
            }
        } while (occ);
    }

    return true;
}

ASTAR_RET AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    const auto time_1 = std::chrono::steady_clock::now();
    last_diagnostics_ = AStarSearchDiagnostics{};
    last_diagnostics_.requested_start = start_pt;
    last_diagnostics_.requested_end = end_pt;
    ++rounds_;
    gridPath_.clear();
    requested_start_ = start_pt;
    requested_end_ = end_pt;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        last_diagnostics_.result = ASTAR_RET::INIT_ERR;
        last_diagnostics_.reason = init_failure_reason_;
        last_diagnostics_.elapsed_seconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - time_1).count();
        last_diagnostics_.adjusted_start = adjusted_start_;
        last_diagnostics_.adjusted_end = adjusted_end_;
        last_diagnostics_.initial_start_occ = initial_start_occ_;
        last_diagnostics_.initial_end_occ = initial_end_occ_;
        last_diagnostics_.start_adjust_steps = start_adjust_steps_;
        last_diagnostics_.end_adjust_steps = end_adjust_steps_;
        return ASTAR_RET::INIT_ERR;
    }

    const Eigen::Vector3d search_start = Index2Coord(start_idx);
    const Eigen::Vector3d search_end = Index2Coord(end_idx);
    last_diagnostics_.adjusted_start = search_start;
    last_diagnostics_.adjusted_end = search_end;
    last_diagnostics_.initial_start_occ = initial_start_occ_;
    last_diagnostics_.initial_end_occ = initial_end_occ_;
    last_diagnostics_.start_adjust_steps = start_adjust_steps_;
    last_diagnostics_.end_adjust_steps = end_adjust_steps_;
    const Eigen::Vector2d search_start_xy = search_start.head<2>();
    const Eigen::Vector2d search_xy_delta = search_end.head<2>() - search_start_xy;
    const double search_xy_len2 = search_xy_delta.squaredNorm();

    auto interpolateZIndexOnSearchPlane = [&](const int x_idx, const int y_idx) -> int {
        if (search_xy_len2 < 1e-8)
            return start_idx(2);

        Eigen::Vector3i sample_idx(x_idx, y_idx, start_idx(2));
        const Eigen::Vector2d sample_xy = Index2Coord(sample_idx).head<2>();
        double ratio = (sample_xy - search_start_xy).dot(search_xy_delta) / search_xy_len2;
        ratio = std::max(0.0, std::min(1.0, ratio));

        const double z = search_start(2) + ratio * (search_end(2) - search_start(2));
        return static_cast<int>((z - center_(2)) * inv_step_size_ + 0.5) + CENTER_IDX_(2);
    };

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    endPtr->index = end_idx;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    double tentative_gScore;

    int num_iter = 0;
    int boundary_rejects = 0;
    int collision_rejects = 0;
    int outside_map_rejects = 0;
    int closed_rejects = 0;
    int generated_nodes = 1;
    size_t max_open_size = openSet_.size();
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            last_diagnostics_.result = ASTAR_RET::SUCCESS;
            last_diagnostics_.reason = "success";
            last_diagnostics_.elapsed_seconds =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - time_1).count();
            last_diagnostics_.iterations = num_iter;
            last_diagnostics_.generated_nodes = generated_nodes;
            last_diagnostics_.max_open_size = max_open_size;
            last_diagnostics_.collision_rejects = collision_rejects;
            last_diagnostics_.outside_map_rejects = outside_map_rejects;
            last_diagnostics_.boundary_rejects = boundary_rejects;
            last_diagnostics_.closed_rejects = closed_rejects;
            return ASTAR_RET::SUCCESS;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        // Keep the search terrain-aware without turning it into an unrestricted
        // 3-D search.  Stair surfaces are not exactly the straight Z plane
        // between a collision segment's endpoints, so allow a narrow vertical
        // band around that reference plane.  Limit each edge to one Z voxel to
        // prevent a horizontal step from producing an unrealistic vertical
        // jump across the full band.
        constexpr int REFERENCE_Z_HALF_BAND_VOXELS = 3;
        constexpr int MAX_Z_STEP_VOXELS = 1;
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0)
                    continue;

                const int neighbor_x = current->index(0) + dx;
                const int neighbor_y = current->index(1) + dy;
                const int reference_z = interpolateZIndexOnSearchPlane(neighbor_x, neighbor_y);
                const int min_neighbor_z = std::max(
                    reference_z - REFERENCE_Z_HALF_BAND_VOXELS,
                    current->index(2) - MAX_Z_STEP_VOXELS);
                const int max_neighbor_z = std::min(
                    reference_z + REFERENCE_Z_HALF_BAND_VOXELS,
                    current->index(2) + MAX_Z_STEP_VOXELS);

                for (int neighbor_z = min_neighbor_z; neighbor_z <= max_neighbor_z; ++neighbor_z)
                {
                    Vector3i neighborIdx(neighbor_x, neighbor_y, neighbor_z);

                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                {
                    ++boundary_rejects;
                    continue;
                }

                neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                neighborPtr->index = neighborIdx;

                bool flag_explored = neighborPtr->rounds == rounds_;

                if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                {
                    ++closed_rejects;
                    continue; //in closed set.
                }

                neighborPtr->rounds = rounds_;

                const double neighbor_yaw = std::atan2(static_cast<double>(dy), static_cast<double>(dx));
                const int neighbor_occ = checkOccupancy(Index2Coord(neighborPtr->index), neighbor_yaw);
                if (neighbor_occ)
                {
                    ++collision_rejects;
                    if (neighbor_occ < 0)
                        ++outside_map_rejects;
                    continue;
                }

                const int dz = neighborIdx(2) - current->index(2);
                double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                tentative_gScore = current->gScore + static_cost;

                if (!flag_explored)
                {
                    //discover a new node
                    neighborPtr->state = GridNode::OPENSET;
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    ++generated_nodes;
                    max_open_size = std::max(max_open_size, openSet_.size());
                }
                else if (tentative_gScore < neighborPtr->gScore)
                { //in open set and need update
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                }
                }
            }
        const auto time_2 = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(time_2 - time_1).count() > 0.2)
        {
            const double elapsed = std::chrono::duration<double>(time_2 - time_1).count();
            last_diagnostics_.result = ASTAR_RET::SEARCH_ERR;
            last_diagnostics_.reason = "SEARCH_TIMEOUT";
            last_diagnostics_.elapsed_seconds = elapsed;
            last_diagnostics_.iterations = num_iter;
            last_diagnostics_.generated_nodes = generated_nodes;
            last_diagnostics_.max_open_size = max_open_size;
            last_diagnostics_.collision_rejects = collision_rejects;
            last_diagnostics_.outside_map_rejects = outside_map_rejects;
            last_diagnostics_.boundary_rejects = boundary_rejects;
            last_diagnostics_.closed_rejects = closed_rejects;
            return ASTAR_RET::SEARCH_ERR;
        }
    }

    const auto time_2 = std::chrono::steady_clock::now();

    const double elapsed = std::chrono::duration<double>(time_2 - time_1).count();
    last_diagnostics_.result = ASTAR_RET::SEARCH_ERR;
    last_diagnostics_.reason = "OPEN_SET_EMPTY";
    last_diagnostics_.elapsed_seconds = elapsed;
    last_diagnostics_.iterations = num_iter;
    last_diagnostics_.generated_nodes = generated_nodes;
    last_diagnostics_.max_open_size = max_open_size;
    last_diagnostics_.collision_rejects = collision_rejects;
    last_diagnostics_.outside_map_rejects = outside_map_rejects;
    last_diagnostics_.boundary_rejects = boundary_rejects;
    last_diagnostics_.closed_rejects = closed_rejects;

    return ASTAR_RET::SEARCH_ERR;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}
