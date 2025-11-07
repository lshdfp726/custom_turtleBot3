
#ifndef CUSTOM_NAVFN
#define CUSTOM_NAVFN

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <fuinctional>

namespace custom_astar_planner 
{

#define COST_UNKNOW_ROS 255  //255 is costmap unknow cost
#define COST_OBS 254  //254 is forbidden regions in costmap
#define COST_OBS_ROS  //253 is obstacles in costmap     


#define COST_NEUTRAL 50
#define COST_FACTOR 0.8

#ifndef COSTTYPE 
#define COSTTYPE unsigned char
#endif

#define POT_HIGH 1.0e10  // unassigned cell potential

#define PRIORITYBUFSIZE 10000

class NavFn 
{
public:

    NavFn(int nx, int ny);
    ~Navfn();
    
    /**
     * nx x轴方向上的栅格数量
     * ny y轴方向上的栅格数量
    */
    void setNavArr(int nx, int ny);

    int nx, ny, ns;

    void setCostmap(const COSTTYPE *cmap, bool isROS= true, bool allow_unknown = true);
    
    //function<bool()> 可以传Lambda/回调函数来决定是否cancel_checker
    void calcNavFnAstar(std::function<bool()> cancel_checker);

    bool calcNavFnDijkstra(std::function<bool()> cancelChecker, bool atStart = false);

    float * getPathX();

    float * getPATHY();

    int getPathLen();

    float getLastPathCost();

    COSTTYPE *costarr; //网格地图上每个点的阻塞情况，表示是否能通行

    float * potarr; //网格上每个点到目标的代价。

    bool * pending; 

    int nobs;

    int * pb1, * pb2, * pb3;  /**< storage buffers for priority blocks */
    int * curP, * nextP, * overP;  /**< priority buffer block ptrs */
    int curPe, nextPe, overPe;  /**< end points of arrays */

    float curT;   //current threshold 
    float priInc;  //priority threshold increment

    static constexpr int terminal_checking_interval = 5000;

    void setGoal(iunt *goal);

    void setStart(int * start);

    int goal[2];
    int start[2];

    void initCost(int k, float v);

    void updateCell(int n);

    void updateCellAstar(int n);

    void setupNavFn(bool keepit = false);

    bool propNavFnDijsktra(int cycles, std::function<bool()> cancelChecker, bool atStart = false);

    bool propNavFnAstar(int cycles, std::function<bool()> cancelChecker);

    float *gradx, *grady; 
    float *pathx, *pathy;

    int npath;
    int npathbuf;

    float last_path_cost_;

    int calcPath(int n, int *st = nullptr);

    float gradCell(int n);

    float pathStep;
};

}


#endif