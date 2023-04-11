#ifndef ASTART_SEARCHER_H
#define ASTART_SEARCHER_H

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <map>

#define INIT_SCORE 10000
#define OPEN_POINT 1
#define CLOSE_POINT -1

struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
    int id;        // 1--> open set, -1 --> closed set
    float f_score, g_score, h_score;

    Eigen::Vector2i index;
    GridNodePtr father_point;
    std::multimap<float, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector2i _index){  

      id = 0;
      index = _index;

      g_score = INIT_SCORE;
      f_score = INIT_SCORE;
      h_score = INIT_SCORE;
      father_point = NULL;
    }

};


class AstarPathFinder
{
private:
    /* data */
public:
    AstarPathFinder(/* args */);
    //~AstarPathFinder();

    std::vector<Eigen::Vector2i> path;

    void InitGridMap(const nav_msgs::OccupancyGrid msg);
    Eigen::Vector2i PosToGrid(const Eigen::Vector2f pos);
    bool isOccupied(const Eigen::Vector2i & index) const;
    bool isOccupied(const Eigen::Vector2f & pos);
    bool isFree(const Eigen::Vector2i & index) const;
    bool isFind(void);
    bool get_goal(const geometry_msgs::PoseStampedConstPtr & goal);
    void resetGrid(GridNodePtr ptr);
    void resetUsedGrids();
    void get_state(const geometry_msgs::PoseStamped & state);
    int  get_heu(Eigen::Vector2i start_pt, Eigen::Vector2i goal_pt);
    std::vector<Eigen::Vector2i> get_path(void); 
    void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets,  \
                        std::vector<float> & edgeCostSets);
    void AstarGraphSearch(void); 
    
protected:
    
    std::vector<int8_t> grid_data;
    float resolution;
    int map_width, map_height;
    int finding = 0;
    Eigen::Vector2f origin_pos;

    geometry_msgs::PoseStampedConstPtr  goal_pos;
    geometry_msgs::PoseStamped state_pos, start_pos;

    Eigen::Vector2i goal_index, state_index, start_index;
    
    std::multimap<float, GridNodePtr> openSet;
    std::vector<Eigen::Vector2i> closeSet;

    GridNodePtr terminatePtr;
    GridNodePtr ** GridNodeMap;

};





#endif