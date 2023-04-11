#include "Astar_searcher.h"


AstarPathFinder::AstarPathFinder() {

    state_index(0) = 96;
    state_index(1) = 54;
    start_index(0) = 96;
    start_index(1) = 54;

}

bool AstarPathFinder::isFind(void) {

    if(finding == 1)
        return true;
    else 
        return false;
}

void AstarPathFinder::InitGridMap(const nav_msgs::OccupancyGrid msg) {

    resolution = msg.info.resolution;
    origin_pos(0) = msg.info.origin.position.x;
    origin_pos(1) = msg.info.origin.position.y;
    map_width = msg.info.width;
    map_height = msg.info.height;
    grid_data = msg.data;
    
    // 分配栅格地图
    GridNodeMap = new GridNodePtr * [map_width];    //2级指针

    for(int i = 0; i < map_width; i++){

        GridNodeMap[i] = new GridNodePtr[map_height];
        for(int j = 0; j < map_height; j++){

                Eigen::Vector2i tmpIdx;
                tmpIdx(0) = i;
                tmpIdx(1) = j;  
                GridNodeMap[i][j] = new GridNode(tmpIdx); 
            
        }
    }

    ROS_INFO("init finish!");

}

Eigen::Vector2i  AstarPathFinder::PosToGrid(const Eigen::Vector2f pos) {
    
    Eigen::Vector2i grid_xy;
    grid_xy(0) = int((pos(0) - origin_pos(0)) / resolution) + 1;
    grid_xy(1) = int((pos(1) - origin_pos(1)) / resolution) + 1;
    return grid_xy;
}

bool AstarPathFinder::isOccupied(const Eigen::Vector2i & index) const{

#if 1
    std::vector<int> data;

    for(int i = -4; i <= 4; i++)
        for(int j = -4; j <= 4; j++)
            data.push_back(int(grid_data.at( (index(1)+i) * map_width + index(0) + j)));

    for(auto p : data) {
        if(p == 100 || p == -1) {
            return true;
        }
    }
    return false;

#else
    // int data = int(grid_data.at(index(1) * map_width + index(0)));
    // if(data == 100 || data == -1)
    //     return true;
    // else 
    //     return false;
#endif

}

bool AstarPathFinder::isOccupied(const Eigen::Vector2f & pos){

    Eigen::Vector2i goal_point_i;
    goal_point_i = PosToGrid(pos);   

    if(isOccupied(goal_point_i))
        return true;
    else 
        return false;     
}

bool AstarPathFinder::isFree(const Eigen::Vector2i & index) const {

    return ~isOccupied(index);

}

bool AstarPathFinder::get_goal(const geometry_msgs::PoseStampedConstPtr & goal) {

    Eigen::Vector2f goal_point_f;
    goal_point_f(0) = goal->pose.position.x;
    goal_point_f(1) = goal->pose.position.y;
    

    if(isOccupied(goal_point_f)) {
        ROS_INFO("set goal failed ! is Occupied !!!");
        return false;
    }
    else {

        ROS_INFO("set goal success ! ");
        goal_pos = goal;
        start_pos = state_pos; 

        Eigen::Vector2f start_point_f;
        start_point_f(0) = start_pos.pose.position.x;
        start_point_f(1) = start_pos.pose.position.y;
        
        goal_index = PosToGrid(goal_point_f);
        start_index = PosToGrid(start_point_f);
        finding = 1;

        ROS_INFO("Start_pose: x = %f, y = %f", start_pos.pose.position.x, start_pos.pose.position.y);
        ROS_INFO("Goal_pose:  x = %f, y = %f", goal_point_f(0), goal_point_f(1));
        return true;
    }

}

int AstarPathFinder::get_heu(Eigen::Vector2i start_pt, Eigen::Vector2i goal_pt) {

    return (std::abs(goal_pt(0) - start_pt(0)) + std::abs(goal_pt(1) - start_pt(1)));
}

void AstarPathFinder::get_state(const geometry_msgs::PoseStamped & state) {

    Eigen::Vector2f pose_f;
    pose_f(0) = state.pose.position.x;
    pose_f(0) = state.pose.position.y;

    // ROS_INFO("pose: x = %f, y = %f", pose_f(0), pose_f(1));
    state_pos = state;
    state_index = PosToGrid(pose_f);
}

void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<float> & edgeCostSets) {

    neighborPtrSets.clear();
    edgeCostSets.clear();

    Eigen::Vector2i current_index = currentPtr->index;
    Eigen::Vector2i tmp_index;
    GridNodePtr tmp_ptr = NULL;
    int current_x = current_index[0];
    int current_y = current_index[1];

    for(int i = -1 ; i <= 1 ; ++i){
        for(int j = -1 ; j<= 1 ; ++j){

                if( i == 0 && j == 0){
                    continue;
                }
                
                tmp_index(0) = current_x + i;
                tmp_index(1) = current_y + j;

                if( (tmp_index(0) < 0 )|| (tmp_index(1) < 0) || (tmp_index(0) > map_width -1) || (tmp_index(1) > map_height -1)){
                    continue;
                }

                if(isOccupied(tmp_index)){
                    continue;
                }
                

                float dist ;
                if(std::abs(i) == 1 && std::abs(j) == 1) {
                    dist = 1.4;    
                }
                else {
                    dist = 1;
                }
                
                //zuikuihuoshou
                tmp_ptr = GridNodeMap[tmp_index(0)][tmp_index(1)];
                
                neighborPtrSets.push_back(tmp_ptr);
                edgeCostSets.push_back(dist);

        }
    }

}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->father_point = NULL;
    ptr->g_score = INIT_SCORE;
    ptr->f_score = INIT_SCORE;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < map_width ; i++)
        for(int j=0; j < map_height ; j++)
                resetGrid(GridNodeMap[i][j]);

    path.clear();
}


void AstarPathFinder::AstarGraphSearch(void){

    ros::Time time_1 = ros::Time::now();  

    GridNodePtr start_grid = new GridNode(start_index);
    GridNodePtr goal_grid = new GridNode(goal_index);


    start_grid->f_score = get_heu(start_index, goal_index);
    //std::cout << "start_f = " << start_grid->f_score << std::endl;

    start_grid->g_score = 0;
    start_grid->id = 1;

    openSet.clear();

    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    openSet.insert(std::make_pair(start_grid->f_score, start_grid));

    std::vector<GridNodePtr> neighborPtrSets;
    std::vector<float> edgeCostSets;

    // //   this is the main loop
    while (!openSet.empty()){

        // 弹出最xiao f的节点
        currentPtr = openSet.begin()->second;
        currentPtr->id = -1; // 标记为闭集

        // std::cout << "current index: x = " << currentPtr->index(0) << " y = " << currentPtr->index(1) <<std::endl;
        
        // 从开集中移除
        openSet.erase(openSet.begin());

        // 获取拓展集合
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);     

        // 遍历拓展集合        
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            
            neighborPtr = neighborPtrSets[i];
            float gh = currentPtr->g_score + edgeCostSets[i];
            float fh = gh + get_heu(neighborPtr->index, goal_index);

            // std::cout << "neighborPtr index: x = " << neighborPtr->index(0) << " y = " << neighborPtr->index(1) << " id = " << neighborPtr->id << std::endl;

            // 如果为自由节点
            if(neighborPtr->id == 0){ 
                
                // 计算相应的g和f，并加入opensets
                neighborPtr->id = 1;
                neighborPtr->g_score = gh;
                neighborPtr->f_score = fh;
                neighborPtr->father_point = currentPtr;

                neighborPtr->nodeMapIt = openSet.insert(std::make_pair(neighborPtr->f_score, neighborPtr));
                // 此处注意一定要先计算和赋值完f再加入

                // 判断是否为目标节点 改到此处为了提高代码效率 不用将所有节点加入后等弹出时发现目标再退出
                if(neighborPtr->index == goal_index){
                    ros::Time time_2 = ros::Time::now();
                    terminatePtr = neighborPtr;
                    finding = 0;
                    ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->g_score * resolution );    
                    return;
                }
                else{
                    // 标记为open list
                    neighborPtr->id = 1;
                    continue;
                }
            }
            else if(neighborPtr->id == 1){ 
                // 如果已经在openlist里面
                if(neighborPtr->g_score > gh)
                {
                    // 更新对应的f值
                    neighborPtr->g_score = gh;
                    neighborPtr->f_score = fh;
                    neighborPtr->father_point = currentPtr;
                    openSet.erase(neighborPtr->nodeMapIt);
                    neighborPtr->nodeMapIt = openSet.insert(std::make_pair(neighborPtr->f_score, neighborPtr));

                }
            }
            else{
                // 如果是closelist里面的则不做处理
                continue;
            }

        }      
    }

    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );



}

std::vector<Eigen::Vector2i> AstarPathFinder::get_path() 
{   
    std::vector<Eigen::Vector2i> path;
    std::vector<GridNodePtr> gridPath;

    GridNodePtr tmp_ptr = terminatePtr;

    while(tmp_ptr->father_point != NULL )
    {
        gridPath.push_back(tmp_ptr);
        tmp_ptr = tmp_ptr->father_point;
    }

    for (auto ptr: gridPath)
        path.push_back(ptr->index);

    reverse(path.begin(),path.end());

    return path;
}

