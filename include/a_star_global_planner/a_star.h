#ifndef ASTAR_GLOBAL_PLANNER_
#define ASTAR_GLOBAL_PLANNER_

#include <math.h>
#include <stdio.h>
#include <ros/ros.h>
#include <Eigen/Eigen>

#ifndef COSTTYPE
#define COSTTYPE unsigned char
#endif

#define COST_UNKNOWN_ROS 255
#define COST_OBS 254
#define COST_OBS_ROS 253
#define COST_FREE 0

#define inf 1e10

namespace astar_global_planner{

    class AstarNode; 
    typedef AstarNode* AstarNodePtr;

    class AstarNode
    {
        public:
        /* data */
        //int judge; // 1-->open list. -1-->close list
        double gScore; // f值
        double fScore; // g值 f = g + h

        int NodeX;  //节点的栅格x值
        int NodeY;  //节点的栅格y值
        int Node_i; //节点在数组中的索引
        //Eigen::Vector2i index; //节点的栅格坐标
        //Eigen::Vector2d coord; //节点的世界坐标
        //父节点
        //AstarNodePtr ParNode; 

        //父节点
        int ParNodeX;
        int ParNodeY;
        //int ParNode_i;  //父节点
        //std::multimap<double, AstarNodePtr>::iterator nodeMapIt;
        AstarNode(int Nodex, int Nodey, int NodeIndex)
        {
            NodeX = Nodex;
            NodeY = Nodey;
            Node_i = NodeIndex;
            gScore = inf;
            fScore = inf;
        }

        AstarNode(){};
        ~AstarNode(){};
    };
    
    /*Astar算法计算*/
    class AstarGlobalPlanner
    {
        public:
        
        /*有参构造函数*/
        AstarGlobalPlanner(int nx, int ny);
        /*析构函数*/
        ~AstarGlobalPlanner();
        //AstarNode createNode(int x, int y, int Index, double g, double f, int Parx, int Pary);
        // Astar路径规划
        bool calcAstarPath(unsigned char* costs, int start_x, int start_y, int end_x, int end_y);
        
        // 初始化地图
        //void InitMap(const COSTTYPE *costmap, bool allow_unknown);

        // 拓展节点
        //void AstarGetSucc(unsigned char* costs, AstarNode tempNode, AstarNode* currentPtr, std::vector<AstarNode>& neighborNodeSets, std::vector<double>& edgeCostSets);
        void Astar_GetSucc(unsigned char* costs, AstarNode currentNode, std::vector<AstarNode>& neighborNodeSets, std::vector<double>& edgeCostSets);
        
        void Astar_GetSucc_8_dir(unsigned char* costs, AstarNode currentNode, std::vector<AstarNode>& neighborNodeSets, std::vector<double>& edgeCostSets);
        
        // 获取h值
        //double AstarGetHeu(AstarNodePtr node1, AstarNodePtr node2);
        double Astar_GetHeu(AstarNode node1, AstarNode node2);

        // 找到栅格坐标对应的地图一维数组的索引
        int CellToIndex(double x, double y);
        // 找到索引对应的栅格坐标
        void IndexToCell(int index, double& cell_x, double& cell_y);
        
        // 搜索节点在closelist中的索引
        //int closelistIndex(std::map<int, AstarNodePtr> closelist_, int index_);
        int closelist_Index(std::vector<AstarNode> closeList_, int node_x, int node_y);

        // 搜索节点在openlist中的索引
        //double openlistIndex(std::multimap<double, AstarNodePtr> openlist_, int X, int Y);
        double openlist_Index(std::multimap<double, AstarNode> openlist_, int X, int Y);
        // 获取路径
        /*  参数1:      由A*路径规划获得的closelist
            参数2:      起始节点
            参数3:      终止节点
            返回值:     返回一个由二维数组组成的vector 路径*/
        std::vector<std::pair<int,int>> getAstarPath(int xBegin, int yBegin, int xStop, int yStop);

        int nx_, ny_, ns_;
        // 一维地图地图数据
        //std::vector<int> mapData;
        // 一维地图数据，存放cost代价值
        //COSTTYPE *costarry;
        //open list 键值为fScore的值
        //std::multimap<double, AstarNodePtr> openlist;
        std::multimap<double, AstarNode> openlist_;
        //close list 键值为index索引
        std::vector<AstarNode> closelist_;
        //std::map<int, AstarNodePtr> closelist;
        //std::vector<AstarNodePtr> closelist;

        // terminated nodePtr 终止的节点 在头文件中声明的指针，在cpp文件中引用时，需要先初始化才能使用
        /*声明一个指针, 指向AstarNode类*/
        // AstarNode* terminatePtr;
        // begin nodePtr 开始的节点
        /*声明一个指针，指向AsterNode类*/
        //AstarNodePtr beginPtr;

        // current nodePtr，当前操作的节点
        /*声明一个指针，指向AsterNode类*/
        //AstarNode* currentPtr;

        // current nodePtr，当前操作的节点
        /*声明一个指针，指向AsterNode类*/
        //AstarNode* neighborPtr;   

        //创建一个vector,存储所有的拓展的邻居节点
        //std::vector<AstarNodePtr> neighborPtrSets;

        //用来记录邻点间的距离(代价)
        //std::vector<double> edgeCostSets;

        // 起点和终点的栅格坐标
        //int start_x, start_y, goal_x, goal_y;
    
    };
};

#endif