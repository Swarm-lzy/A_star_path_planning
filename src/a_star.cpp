#include "a_star.h"
#include <costmap_2d/cost_values.h>
#include <algorithm>
#include <iostream>
#include <stdio.h>

namespace astar_global_planner
{
    AstarGlobalPlanner::AstarGlobalPlanner(int nx, int ny)
    {   
        nx_ = nx;
        ny_ = ny;
        ns_ = nx * ny;
        
        //用new创建一个节点指针，调用无参构造函数
        //currentPtr = new AstarNode;
        //ROS_WARN("444444444444444444444444444444 %d 5555555555 %d", currentPtr->NodeX, currentPtr->NodeY);
        //用new创建一个节点指针，调用无参构造函数，指向邻居的节点
        //neighborPtr = new AstarNode;
        //ROS_WARN("444444444444444444444444444444 %d 5555555555 %d", neighborPtr->NodeX, neighborPtr->NodeY);
        //用new创建一个对象，调用无参构造函数，指向终点节点
        //terminatePtr = new AstarNode;
        //ROS_WARN("444444444444444444444444444444 %d use construct function %d", terminatePtr->NodeX, terminatePtr->NodeY);
        
        /*
        ROS_DEBUG("COST Array is %d x %d\n", nx, ny);
        costarry = new COSTTYPE[ns_];
        memset(costarry, 0, ns_*sizeof(COSTTYPE));
        */
        //mapData = new COSTTYPE[ns_];
        //memset(mapData, 0, ns_*sizeof(COSTTYPE));
    }
    
    AstarGlobalPlanner::~AstarGlobalPlanner()
    {
        /*
        if (currentPtr)
        {
            delete currentPtr;
        }
        if (neighborPtr)
        {
            delete neighborPtr;
        } 
        */
        /*
        if (terminatePtr)
        {
            delete terminatePtr;
            terminatePtr = NULL;
            ROS_WARN("000000000000000000000000000000000000 use delete function");
        }     
        else
        {
            ROS_WARN("000000000000000000000000000000000000 no use delete function");
        }
        */
    }
    

    bool AstarGlobalPlanner::calcAstarPath(unsigned char *costs, int start_x, int start_y, int end_x, int end_y)
    {   
        //把open list清空 节点
        openlist_.clear();
        //openlist_.clear();
        //把close list清空 节点
        closelist_.clear();
        //closelist_.clear();

        //起点是全部costarr指针数据的第几个
        int start_index = CellToIndex(start_x, start_y);
        //std::cout<< "start_index"<< start_index <<std::endl;
        //构造起点节点
        AstarNode startNode(start_x, start_y, start_index);
        //AstarNodePtr startPtr = &startNode; //不是用new创建的对象指针，在栈中创建，函数结束自动释放
        //AstarNodePtr startPtr = new AstarNode(start_x, start_y, start_index);

        // 终点是全部costarr指针数据的第几个
        int end_index = CellToIndex(end_x, end_y);
        std::cout<< "end_x"<< end_x << "end_y" << end_y << "end_index" << end_index <<std::endl;
        //构造终点节点
        AstarNode goalNode(end_x, end_y, end_index);
        //AstarNodePtr goalPtr = &goalNode;
        //AstarNode goalNode(end_x, end_y, end_index);

        
        // 把起点放入open list中
        //startPtr->judge = 1;
        /*
        startPtr->gScore = 0;
        startPtr->fScore = startPtr->gScore + AstarGetHeu(startPtr, goalPtr);
        startPtr->ParNodeX = 0;
        startPtr->ParNodeY = 0;
        openlist.insert(std::make_pair(startNode.fScore, startPtr));
        */
        //std::cout<<"number of node pointer in openlist\n"<< openlist.size()<<std::endl;
        
        startNode.gScore = 0.0;
        startNode.fScore = startNode.gScore + Astar_GetHeu(startNode, goalNode);
        startNode.ParNodeX = 0;
        startNode.ParNodeY = 0;
        openlist_.insert(std::make_pair(startNode.fScore, startNode));
        
        //std::cout<<"number of node pointer in openlist\n"<< openlist.size()<<std::endl;
        
        AstarNode currentNode;
        //AstarNodePtr currentPtr = &currentNode;

        AstarNode tempNode;
        //AstarNodePtr tempPtr = &tempNode;

        //AstarNode neighborNode;
        //AstarNodePtr neighborPtr = &neighborNode;
        //std::vector<AstarNodePtr> neighborPtrSets;
        std::vector<AstarNode> neighborNodeSets;
        std::vector<double> edgeCostSets;
        /*
        AstarNode terminateNode;
        AstarNodePtr terminatePtr = &terminateNode;
        */

        //将起始点存入开始节点
        //beginPtr = startPtr;
        // 开始循环
        /*如果openlist为空，则跳出循环*/
        //std::cout<< "if the openlist is empty" << openlist.empty() << std::endl;
        int count = 0; 
        //ROS_INFO("Start Loop");
        // 我发现while循环里面只是循环了一次，原因是把起点放入openlist后，没有往里面添加新的节点，并且将起点放入到closelist中了
        
        while (!openlist_.empty())
        {
            count++;
            //ROS_INFO("iteration times %d and number in openlist %ld", count, openlist_.size());
            //std::cout<<"iteration times"<< count << "number of openlist" << openlist_.size() << std::endl;
            //for(std::multimap<double, AstarNode>::iterator it = openlist_.begin(); it!=openlist_.end(); it++)
            //{
            //    ROS_WARN("before x %d and y %d in openlist",it->second.NodeX, it->second.NodeY);
            //}
            // 取最小的f的节点
            //currentNode = openlist_.begin()->second;
            currentNode = openlist_.begin()->second;
            //tempPtr = openlist.begin()->second; // 在未对tempNode进行操作时，tempNode并未改变
            //std::cout <<"before x of minimise f value" << currentPtr->NodeX <<"before y of minimise f value"<< currentPtr->NodeY<< std::endl;
            //std::cout <<"x of currentNode" << currentNode.NodeX <<"y of currentNode"<< currentNode.NodeY<< std::endl;
            //std::cout <<"Parx of currentNode" << currentNode.ParNodeX << "Party of currentNode" << currentNode.ParNodeY << std::endl;
            //std::cout <<"x of start node" << start_x <<"y of start node"<< start_y << std::endl;
            //std::cout<< "end_index"<< end_index <<"end_x"<< end_x <<"end_y"<< end_y <<std::endl;
            // 将该节点从open list取出，放入close list中
            //currentPtr->judge = -1;
            openlist_.erase(openlist_.begin());
            //openlist_.erase(openlist_.begin());
            //closelist.insert(std::make_pair(currentPtr->Node_i, currentPtr));
            closelist_.push_back(currentNode);
            //ROS_INFO("the number of node in openlist %ld", openlist.size());
            // 判断该节点是否是终点
            if (currentNode.Node_i == end_index)
            {
                //将找到的目标点存入终止节点中
                int final_x = currentNode.NodeX;
                int final_y = currentNode.NodeY;
                ROS_WARN("final x %d and end x%d", final_x, end_x);
                ROS_WARN("final y %d and end y%d", final_y, end_y);
                /*
                terminatePtr = &(currentNode);
                ROS_WARN("terminatPtr x%d and y%d", terminatePtr->NodeX, terminatePtr->NodeY);
                */
                return true; 
            }

            //currentPtr与tempPtr指向同一块地址，当tempPtr改变时，currentPtr也改变
            /*
            if (count == 4)
            {
                return false;
            }
            */
            //将该节点进行拓展, 出问题了，拓展的邻居节点为0
            //AstarGetSucc(costs, tempNode, currentPtr, neighborNodeSets, edgeCostSets);
            //std::cout <<"after x of minimise f value" << currentPtr->NodeX <<"after y of minimise f value"<< currentPtr->NodeY<< std::endl;
            Astar_GetSucc(costs, currentNode, neighborNodeSets, edgeCostSets);  
            //Astar_GetSucc_8_dir(costs, currentNode, neighborNodeSets, edgeCostSets);    
            //std::cout <<"after x of minimise f value" << currentNode.NodeX <<"after y of minimise f value"<< currentNode.NodeY<< std::endl;
            //std::cout<< "number of neighbour node in neighborPtrSets list" << neighborNodeSets.size() <<std::endl;
            //输出neighborPtrSets中的节点
            //判断未拓展的节点
            
            for (int i=0; i<neighborNodeSets.size(); i++)
            {   
                //ROS_WARN("The X and Y in neighborPtrSets %d and %d", neighborNodeSets[i].NodeX, neighborNodeSets[i].NodeY);
                //ROS_WARN("successfully implmented");
                tempNode = neighborNodeSets[i];
                //判断新节点是否在openlist中
                double Flag = openlist_Index(openlist_, neighborNodeSets[i].NodeX, neighborNodeSets[i].NodeY);
                //std::cout<<"x of neighborNode" << neighborNodeSets[i].NodeX <<"y of neighborNode" << neighborNodeSets[i].NodeY<< std::endl;
                //double Flag = openlist_Index(openlist_, neighborNodeSets[i].NodeX, neighborNodeSets[i].NodeY);
                //ROS_INFO("if the node is open list\n %f", Flag);
                // 问题出在: openlist每次只会把新扩展的点放进去，之前存在的那些点都删除了
                //新节点，未在openlist中
                if (Flag == -1.0)
                {
                    AstarNode Node(neighborNodeSets[i].NodeX, neighborNodeSets[i].NodeY, neighborNodeSets[i].Node_i);
                    Node.gScore = currentNode.gScore + edgeCostSets[i];
                    Node.fScore = Node.gScore + Astar_GetHeu(tempNode, goalNode);
                    Node.ParNodeX = currentNode.NodeX;
                    Node.ParNodeY = currentNode.NodeY;
                    //ROS_WARN("new node in openlist");
                    //计算新的f值，并指定父节点
                    //并将该新节点插入到openlist中
                    //double gScore = currentNode.gScore + edgeCostSets[i];
                    //double fScore = gScore + Astar_GetHeu(tempNode, goalNode);
                    //neighborNode.gScore = currentNode.gScore + edgeCostSets[i];
                    //neighborNode.fScore = neighborNode.gScore + Astar_GetHeu(neighborNode, goalNode);
                    //int ParNodeX = currentNode.NodeX;
                    //int ParNodeY = currentNode.NodeY;
                    //AstarNode Node = createNode(neighborNodeSets[i].NodeX, neighborNodeSets[i].NodeY, neighborNodeSets[i].Node_i, gScore, fScore, ParNodeX, ParNodeY);
                    openlist_.insert(std::make_pair(Node.fScore, Node));
                    //std::cout<< "Heu cost" << Astar_GetHeu(tempNode, goalNode)<<std::endl;
                    //std::cout<< "ParNodex of new node" << Node.ParNodeX << "ParNodey of new node" << Node.ParNodeY <<std::endl;
                }
                else
                {
                    //ROS_WARN("old node in openlist");
                    std::multimap<double, AstarNode>::iterator indexLow = openlist_.lower_bound(Flag);
                    std::multimap<double, AstarNode>::iterator indexUpper = openlist_.upper_bound(Flag);
                    indexUpper--;
                    if (indexLow == indexUpper)
                    {
                        //ROS_WARN("no repeat key");
                        //std::cout<< "x of old node" << indexLow->second.NodeX << "y of old node" << indexLow->second.NodeY <<std::endl;
                        //std::cout<< "g value of neighbor node" << indexLow->second.gScore <<"g value of current node add cost"<< currentNode.gScore + edgeCostSets[i]<<std::endl;
                        //表示没有重复的键值，即openlist中只有一个对应的节点
                        if (indexLow->second.gScore > currentNode.gScore + edgeCostSets[i])
                        {
                            //std::cout<< "g value of neighbor node" << indexLow->second.gScore <<"g value of current node add cost"<< currentNode.gScore + edgeCostSets[i]<<std::endl;
                            //若从currentPtr节点拓展的节点的代价更小，比之前保存在openlist中的小，则更换f，g以及父节点到currentPtr
                            indexLow->second.gScore = currentNode.gScore + edgeCostSets[i];
                            indexLow->second.fScore = indexLow->second.gScore + Astar_GetHeu(indexLow->second, goalNode);
                            indexLow->second.ParNodeX = currentNode.NodeX;
                            indexLow->second.ParNodeY = currentNode.NodeY;
                            //ROS_WARN("no repeat key, old node in openlist with change");
                            //std::cout<< "Heu cost" << Astar_GetHeu(indexLow->second, goalNode)<<std::endl;
                            //std::cout<< "ParNodex of old node" << indexLow->second.ParNodeX << "ParNodey of new node" << indexLow->second.ParNodeY <<std::endl;
                        }
                        else
                        {   

                            //ROS_WARN("no repeat key, old node in openlist without change");
                            //std::cout<< "ParNodex of old node" << indexLow->second.ParNodeX << "ParNodey of new node" << indexLow->second.ParNodeY <<std::endl;
                        }
                    }
                    else
                    {
                        //表示由重复的键值，即openlist中有若干个对应的节点
                        //则需要找到由currentPtr拓展出来的节点
                        //ROS_WARN("repeat key");
                        while (indexLow != indexUpper)
					    {
						    if (indexLow->second.NodeX == neighborNodeSets[i].NodeX && indexLow->second.NodeY == neighborNodeSets[i].NodeY)
							    break;
						    indexLow++;
					    }
                        //std::cout<< "x of old node" << indexLow->second.NodeX << "y of old node" << indexLow->second.NodeY <<std::endl;
                        //std::cout<< "g value of neighbor node" << indexLow->second.gScore <<"g value of current node add cost"<< currentNode.gScore + edgeCostSets[i]<<std::endl;
                        if (indexLow->second.gScore > currentNode.gScore + edgeCostSets[i])
                        {
                            //若从currentPtr节点拓展的节点的代价更小，比之前保存在openlist中的小，则更换f，g以及父节点到currentPtr
                            indexLow->second.gScore = currentNode.gScore + edgeCostSets[i];
                            indexLow->second.fScore = indexLow->second.gScore + Astar_GetHeu(indexLow->second, goalNode);
                            indexLow->second.ParNodeX = currentNode.NodeX;
                            indexLow->second.ParNodeY = currentNode.NodeY;
                            //ROS_WARN("repeat key, old node in openlist with change");
                            //std::cout<< "Heu cost" << Astar_GetHeu(indexLow->second, goalNode)<<std::endl;
                            //std::cout<< "ParNodex of old node" << indexLow->second.ParNodeX << "ParNodey of new node" << indexLow->second.ParNodeY <<std::endl;
                        }
                        else
                        {
                            //ROS_WARN("repeat key, old node in openlist without change");
                            //std::cout<< "ParNodex of old node" << indexLow->second.ParNodeX << "ParNodey of new node" << indexLow->second.ParNodeY <<std::endl;
                        }
                    }
                    
                    /*
                    //ROS_WARN("old node in openlist");
                    //新节点，在openlist中
                    //将openlist中对应的节点取出
                    std::multimap<double, AstarNodePtr>::iterator indexLow = openlist.lower_bound(Flag);
                    std::multimap<double, AstarNodePtr>::iterator indexUpper = openlist.upper_bound(Flag);
                    indexUpper--;
                    if (indexLow == indexUpper)
                    {
                        //表示没有重复的键值，即openlist中只有一个对应的节点
                        if (indexLow->second->gScore >= currentPtr->gScore + edgeCostSets[i])
                        {
                            //若从currentPtr节点拓展的节点的代价更小，比之前保存在openlist中的小，则更换f，g以及父节点到currentPtr
                            indexLow->second->gScore = currentPtr->gScore + edgeCostSets[i];
                            indexLow->second->fScore = indexLow->second->gScore + AstarGetHeu(indexLow->second, goalPtr);
                            indexLow->second->ParNodeX = currentPtr->NodeX;
                            indexLow->second->ParNodeY = currentPtr->NodeY;
                        }
                    }
                    else
                    {
                        //表示由重复的键值，即openlist中有若干个对应的节点
                        //则需要找到由currentPtr拓展出来的节点
                        while (indexLow != indexUpper)
					    {
						    if (indexLow->second->Node_i == neighborPtr->Node_i)
							    break;
						    indexLow++;
					    }
                        if (indexLow->second->gScore > currentPtr->gScore + edgeCostSets[i])
                        {
                            //若从currentPtr节点拓展的节点的代价更小，比之前保存在openlist中的小，则更换f，g以及父节点到currentPtr
                            indexLow->second->gScore = currentPtr->gScore + edgeCostSets[i];
                            indexLow->second->fScore = indexLow->second->gScore + AstarGetHeu(indexLow->second, goalPtr);
                            indexLow->second->ParNodeX = currentPtr->NodeX;
                            indexLow->second->ParNodeY = currentPtr->NodeY;
                        }
                    }  
                    */
                }
            }
            //将openlist中的节点打印出来
            /*
            for(std::multimap<double, AstarNode>::iterator it = openlist_.begin(); it!=openlist_.end(); it++)
            {
                ROS_WARN("after x %d and y %d in openlist",it->second.NodeX, it->second.NodeY);
            }
            */
        }
        return false;
    }
    
    //初始化地图的代价值
    /*
        costmap 指向地图的指针
        cm 指向创建的指针
    */
    /*
    void AstarGlobalPlanner::InitMap(const COSTTYPE *costmap, bool allow_unknown)
    {
        // 创建一个指针，指向代价值数组
        COSTTYPE *cm = costarry;
        for (int i=0; i<ny_; i++)
        {
            int k=i*nx_;
            for (int j=0; j<nx_; j++, k++, costmap++, cm++)
            {
                *cm = COST_OBS; // 首先对cm赋值为254
                int v = *costmap; //取costmap的代价值，从第一个开始取值
                // 当地图栅格的代价值小于253，则当做空闲栅格,当大于255，当做未知区域
                
                if (v < COST_OBS_ROS)
                {
                    v = COST_FREE;
                    *cm = v;
                }
                else if (v >= COST_UNKNOWN_ROS && allow_unknown)
                {
                    v = COST_UNKNOWN_ROS;
                    *cm = v;
                }
                else 
                {
                    v = COST_OBS;
                    *cm = v;
                }
            }
        }

    }
    */

    //由该节点进行上，下，左，右拓展四个节点，需要判断三个条件
    /*
        1. 新节点是否在地图内
        2. 新节点是否是障碍物或者未知区域
        3. 新节点是否在closelist，即是已经被拓展后的节点
    */

    /*
    void AstarGlobalPlanner::AstarGetSucc(unsigned char* costs, AstarNode tempNode, AstarNodePtr currentPtr, std::vector<AstarNode>& neighborNodeSets, std::vector<double>& edgeCostSets)
    {
        neighborNodeSets.clear();
        edgeCostSets.clear();
        AstarNode currentNode = *(currentPtr);
        
        //int cur_x = currentPtr->NodeX;
        //int cur_y = currentPtr->NodeY;
        //int cur_index = currentPtr->Node_i;
             
        //ROS_WARN("current index %d", cur_index);
        //ROS_WARN("current cost value %d", (int)costs[cur_index]); //确实是0，因为在之前将起点的cost设置为了0
        int next_x, next_y;
        int next_index;

        for (int i = -1; i<=1; i++)
        {
            for (int j = -1; j<=1; j++)
            {
                std::cout << "before current x" << currentPtr->NodeX << std::endl;
                std::cout << "before current y" << currentPtr->NodeY << std::endl;
                AstarNode nextNode;
                //AstarNodePtr nextPtr = &nextNode;
                // 取左，下，上，右四个邻居节点
                if ((i+j) == 1 || (i+j) == -1)
                {
                    next_x = currentNode.NodeX + i;
                    next_y = currentNode.NodeY + j;
                    next_index = CellToIndex(next_x, next_y);
                    nextNode.NodeX = next_x;
                    nextNode.NodeY = next_y;
                    nextNode.Node_i = next_index;
                    // 由index索引值判断是否溢出
                    if (next_index < 0 || next_index >= ns_)
                    {
                        std::cout<<"exceed"<<std::endl;
                    }
                    // 有障碍的点，或者无信息的点
                    else if ((int)costs[next_index] >= COST_OBS)
                    {
                        std::cout<< "obstacle or no information" <<std::endl;
                    }
                    // 是否在close list中
                    else if (closelist_Index(closelist_, next_x, next_y) == 1)
                    {
                        std::cout<< "in closelist" << std::endl;
                    }
                    
                    else if (closelistIndex(closelist, next_index) == 1)
                    {
                        ROS_WARN("in closelist");
                        continue;
                    }
                    
                    else 
                    {
                        std::cout<< "normal node" << std::endl;
                        neighborNodeSets.push_back(nextNode);
                        edgeCostSets.push_back(std::sqrt(abs(currentNode.NodeX - next_x) + abs(currentNode.NodeY - next_y)) * 50.0);
                    }
                }
                std::cout << "after current x" << currentPtr->NodeX << std::endl;
                std::cout << "after current y" << currentPtr->NodeY << std::endl;
                //std::cout << "after temp x" << tempNode.NodeX << std::endl;
                //std::cout << "after temp y" << tempNode.NodeY << std::endl;
                
                if (i==0 && j==0)
                {   
                    // 跳过此次循环
                    continue;
                }
                if (i==j || (i+j) == 0)
                {
                    // 跳过此次循环
                    continue;
                }
                // 获取拓展节点的地图坐标,不能这么取
                
                next_x = cur_x + i;
                next_y = cur_y + j;
                next_index = CellToIndex(next_x, next_y);
                nextNode.NodeX = next_x;
                nextNode.NodeY = next_y;
                nextNode.Node_i = next_index;
                
                
                std::cout<< "next_x" <<next_x<<std::endl;
                std::cout<< "next_y" <<next_y<<std::endl;
                std::cout<< "next_index" <<next_index<<std::endl;
                std::cout<< "cost value" << (int)costs[next_index] <<std::endl;
            
                //ROS_WARN("COST VALUE IS %d", costs[1]);   
                
                //neighborNodeSets.push_back(nextNode);
                //edgeCostSets.push_back(std::sqrt(abs(cur_x - next_x) + abs(cur_y - next_y)) * 50.0);
            }
        }
    }
    */
    void AstarGlobalPlanner::Astar_GetSucc_8_dir(unsigned char* costs, AstarNode currentNode, std::vector<AstarNode>& neighborNodeSets, std::vector<double>& edgeCostSets)
    {
        neighborNodeSets.clear();
        edgeCostSets.clear();
        int next_x, next_y, next_index;
        for (int i = -1; i<=1; i++)
        {
            for (int j = -1; j<=1; j++)
            {
                AstarNode nextNode;
                if (i != 0 || j != 0)
                {
                    next_x = currentNode.NodeX + i;
                    next_y = currentNode.NodeY + j;
                    next_index = CellToIndex(next_x, next_y);
                    nextNode.NodeX = next_x;
                    nextNode.NodeY = next_y;
                    nextNode.Node_i = next_index;
                    if (next_index < 0 || next_index >= ns_)
                    {

                    }
                    else if ((int)costs[next_index] >= COST_OBS)
                    {

                    }
                    else if (closelist_Index(closelist_, next_x, next_y) != -1)
                    {
                        
                    }
                    else
                    {
                        neighborNodeSets.push_back(nextNode);
                        edgeCostSets.push_back(std::sqrt(abs(currentNode.NodeX - next_x) + abs(currentNode.NodeY - next_y)) * 7);
                    }
                }
            }
        } 
    }

    void AstarGlobalPlanner::Astar_GetSucc(unsigned char* costs, AstarNode currentNode, std::vector<AstarNode>& neighborNodeSets, std::vector<double>& edgeCostSets)
    {

        neighborNodeSets.clear();
        edgeCostSets.clear();
        int next_x, next_y, next_index;
        for (int i = -1; i<=1; i++)
        {
            for (int j = -1; j<=1; j++)
            {
                //std::cout << "before current x" << currentNode.NodeX << std::endl;
                //std::cout << "before current y" << currentNode.NodeY << std::endl;
                AstarNode nextNode;
                //AstarNodePtr nextPtr = &nextNode;
                // 取左，下，上，右四个邻居节点
                if (i != 0 || j != 0 )
                {
                    next_x = currentNode.NodeX + i;
                    next_y = currentNode.NodeY + j;
                    //std::cout << "next x" << next_x << std::endl;
                    //std::cout << "next y" << next_y << std::endl;
                    next_index = CellToIndex(next_x, next_y);
                    nextNode.NodeX = next_x;
                    nextNode.NodeY = next_y;
                    nextNode.Node_i = next_index;
                    // 由index索引值判断是否溢出
                    if (next_index < 0 || next_index >= ns_)
                    {
                        //std::cout<<"exceed"<<std::endl;
                    }
                    // 有障碍的点，或者无信息的点
                    else if ((int)costs[next_index] >= COST_OBS)
                    {
                        //std::cout<< "obstacle or no information" <<std::endl;
                    }
                    // 是否在close list中
                    else if (closelist_Index(closelist_, next_x, next_y) != -1)
                    {
                        //std::cout<< "in closelist" << std::endl;
                    }
                    /*
                    else if (closelistIndex(closelist, next_index) == 1)
                    {
                        ROS_WARN("in closelist");
                        continue;
                    }
                    */
                    else 
                    {
                        //std::cout<< "normal node" << std::endl;
                        neighborNodeSets.push_back(nextNode);
                        edgeCostSets.push_back(std::sqrt(abs(currentNode.NodeX - next_x) + abs(currentNode.NodeY - next_y)) * 7);
                    }
                }
                //std::cout << "after current x" << currentNode.NodeX << std::endl;
                //std::cout << "after current y" << currentNode.NodeY << std::endl;
            }
        }
    }



    double AstarGlobalPlanner::Astar_GetHeu(AstarNode node1, AstarNode node2)
    {
        double x_1 = node1.NodeX;
        double y_1 = node1.NodeY;

        double x_2 = node2.NodeX;
        double y_2 = node2.NodeY;

        double dx = abs((double)(x_1 - x_2)) * 7.0;
        double dy = abs((double)(y_1 - y_2)) * 7.0;

        double h = std::sqrt(std::pow(dx,2.0) + std::pow(dy,2.0));
        return h;
    }

    /*
    double AstarGlobalPlanner::AstarGetHeu(AstarNodePtr node1, AstarNodePtr node2)
    {
        double start_x = node1->NodeX;
        double start_y = node1->NodeY;

        double end_x = node2->NodeX;
        double end_y = node2->NodeY;

        double dx = abs((double)(start_x - end_x));
        double dy = abs((double)(start_y - end_y));
        double h = std::sqrt(std::pow(dx,2.0) + std::pow(dy,2.0));
        return h;
    }
    */

    int AstarGlobalPlanner::CellToIndex(double x, double y)
    {
        return x + nx_ * y;
    }

    void AstarGlobalPlanner::IndexToCell(int index, double& cell_x, double& cell_y)
    {
        cell_x = double(index % nx_);
        cell_y = double(index / nx_);
    }

    /*判断该节点对应的索引是否在closelist中，如果在则返回1，如果不在则返回-1*/
    /*
    int AstarGlobalPlanner::closelistIndex(std::map<int, AstarNodePtr> closelist_, int index_)
    {
        std::map<int, AstarNodePtr>::iterator iter;
        iter = closelist_.find(index_);
        if (iter == closelist_.end())
        {
            return -1;
        }
        return 1;
    }
    */

    // 在closelist中返回i， 不在就返回-1
    int AstarGlobalPlanner::closelist_Index(std::vector<AstarNode> closeList_, int node_x, int node_y)
    {
        for (int i=0; i<closeList_.size(); i++)
        {
            if (node_x == closeList_[i].NodeX && node_y == closeList_[i].NodeY)
            {
                return i;
            }
        }
        return -1;
    }

    /*判断该节点对应的索引是否在openlist中
    key值索引，注意因为在multimap中，key值可能是重复的
    返回值: -1              ----未找到
            fScore         -----找到*/
    /*
    double AstarGlobalPlanner::openlistIndex(std::multimap<double, AstarNodePtr> openlist_, int X, int Y)
    {
        for(std::multimap<double, AstarNodePtr>::iterator it = openlist_.begin(); it!=openlist_.end(); it++)
        {
            if (it->second->NodeX == X && it->second->NodeY == Y)
            {
                return it->first;
            }
        }
        return -1.0;
    }
    */

    double AstarGlobalPlanner::openlist_Index(std::multimap<double, AstarNode> openlist_, int X, int Y)
    {
        for(std::multimap<double, AstarNode>::iterator it = openlist_.begin(); it!=openlist_.end(); it++)
        {
            if (it->second.NodeX == X && it->second.NodeY == Y)
            {
                return it->first;
            }
        }
        return -1.0;
    }

    /*
    AstarNode AstarGlobalPlanner::createNode(int x, int y, int Index, double g, double f, int Parx, int Pary)
    {
        AstarNode node;
        node.NodeX = x;
        node.NodeY = y;
        node.Node_i = Index;
        node.gScore = g;
        node.fScore = f;
        node.ParNodeX = Parx;
        node.ParNodeY = Pary;
        return node;
    }
    */
    std::vector<std::pair<int,int>> AstarGlobalPlanner::getAstarPath(int xStart, int yStart, int xGoal, int yGoal)
    {
        
        std::pair<int, int> path;

        //存储路径
	    std::vector<std::pair<int, int>> findPath;

	    path.first = xGoal;
	    path.second = yGoal;
	    findPath.emplace_back(path);
        int index = closelist_Index(closelist_, xGoal, yGoal);

        while (true)
        {   
            if (closelist_[index].ParNodeX == xStart && closelist_[index].ParNodeY == yStart)
            {
                path.first = xStart;
                path.second = yStart;
                findPath.emplace_back(path);   
                break; 
            }
            int nodeX = closelist_[index].ParNodeX;
            int nodeY = closelist_[index].ParNodeY;
            path.first = nodeX;
            path.second = nodeY;
            findPath.emplace_back(path); // 不需要创建一个对象实例，如果用push_back就要创建一个对象实例

            index = closelist_Index(closelist_, nodeX, nodeY);
        }
        std::reverse(findPath.begin(), findPath.end());
        return findPath;
        /*
        std::cout<< "x of termination in function" << terminatePtr->NodeX<< std::endl;
        AstarNode firstNode = closelist_[0];
        int X_first = firstNode.NodeX;
        int Y_first = firstNode.NodeY;
        ROS_WARN("x first in function %d and y first in function %d", X_first, Y_first);

        AstarNode finalNode = (closelist_).back();
        int X_final = finalNode.NodeX;
        int Y_final = finalNode.NodeY;
        ROS_WARN("x final in function %d and y final in function %d", X_final, Y_final);

        int num = (closelist_).size();
        ROS_WARN("the number of node in closelist in function %d", num);
        */




        /*
        AstarNodePtr prevPtr = terminatePtr;
        //int ParX, ParY, ParIndex;
        while (prevPtr->ParNodeX != 0 && prevPtr->ParNodeY != 0)
        {
            //code
            //ROS_INFO("");
            gridPath.push_back(prevPtr);
            int ParX = prevPtr->ParNodeX;
            int ParY = prevPtr->ParNodeY;
            int ParIndex = CellToIndex(ParX, ParY); 
            prevPtr = new AstarNode(ParX, ParY, ParIndex);
            
        }
        for (auto ptr: gridPath)
        {   
            temp.first = ptr->NodeX;
            temp.second = ptr->NodeY;
            path.push_back(temp);
        }
        std::reverse(path.begin(),path.end());
        ROS_WARN("start X %f and start Y %f", path[0].first, path[0].second);
        */
    }
}

