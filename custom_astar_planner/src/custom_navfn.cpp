#include "nav2_navfn_planner/navfn.hpp"

#include <algorithm>
#include "nav2_core/planner_exceptions.hpp"
#include "rclcpp/rclcpp.hpp"


namespace custom_astar_planner 
{

    NavFn::NavFn(int xs, int ys) 
    {
        costarr = nullptr;
        postarr = nullptr;
        pending = nullptr;
        gradx = grady = nullptr;

        setNavArr(xs, ys);

        pb1 = new int[PRIORITYBUFSIZE];
        pb2 = new int[PRIORITYBUFSIZE];
        pb3 = new int[PRIORITYBUFSIZE];

        priInc = 2 * COST_NEUTRAL;

        goal[0] = goal[1] = 0;
        start[0] = start[1] = 0;

        npathbuf = npath = 0;
        pathx = pathy = nullptr;
        pathStep = 0.5;
    }

    NavFn::~NavFn() {
        if (costarr) {
            delete[] costarr;
        }

        if (postarr) {
            delete[] postarr;
        }

        if (pending) {
            delete[] pending;
        }

        if (gradx) {
            delete[] gradx;
        }

        if (grady) {
            delete[] grady;
        }

        if (pathx) {
            delete[] pathx;
        }

        if (pathy) {
            delete[] pathy;
        }

        if (pb1) {
            delete[] pb1;
        }

        if (pb2) {
            delete[] pb2;
        }

        if (pb3) {
            delete[] pb3;
        }
    }

    NavFn::setGoal(int * g) {
        goal[0] = g[0];
        goal[1] = g[1];
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Setting goal to %d, %d\n", goal[0], goal[1]);
    }

    void NavFn::setStart(int *g) {
        start[0] = g[0];
        start[1] = g[1];
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Setting start to %d,%d\n", start[0], start[1]);
    }

    void NavFn::setNavArr(int xs, int ys) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Array is %d x %d\n", xs, ys);

        nx = xs; //横轴
        ny = ys; //纵轴
        ns = nx * ny; //总的栅格数量

        if (costarr) {
            delete[] costarr;
        }
        if (postarr) {
            delete[] postarr;
        }
        if (pending) {
            delete[] pending;
        }

        if (gradx) {
            delete[] gradx;
        }

        if (grady) {
            delete[] grady;
        }

        costarr = new COSTTYPE[ns];
        memset(costarr, 0, ns * sizeof(COSTTYPE));
        postarr = new float[ns];
        pending = new bool[ns];
        memset(pending, 0 , ns * sizeof(bool));
        gradx = new float[ns];  //x方向上的左边到右边的代价差值， < 0 往右走，>0 往左走
        grady = new float[ns];  //y方向上的左边到右边的代价差值， < 0 往下走，>0 往上走
    }
    
    /**
    * 把ros 给的实际代价[0, 50,255 ...] 经过内部 COST_NEUTRAL + COST_FACTOR * v 转换一下，
    * 目的是 最低代价不是0，避免计算除0风险
    * 生成更安全的路径，无代价的0 转为50 意味着不会贴着障碍物/区域走
    *  cmap 输入代价
    *  costarr 转换的输出代价，后续流程中使用
    */
    void NavFn::setCostmap(const COSTTYPE *cmap, bool isROS, bool allow_unknown) 
    {
        COSTTYPE *cm = costarr;
        if (isROS) {
            for (int i = 0; i < ny; i ++) {
                int k = i * nx;  //一维数组
                for (int j = 0; j < nx; j++, k++, cmap ++, cm++) {//cmap 输入指针，cm输出指针
                    *cm = COST_OBS;
                    int v = *cmap;
                    if (v < COST_OBS_ROS) { //非障碍物路径
                        v = COST_NEUTRAL + COST_FACTOR * v;
                        if (v >= COST_OBS) {
                            v = COST_OBS - 1; //确保不超过障碍物的阈值
                        }
                        *cm = v;
                    } else if (v == COST_UNKNOW_ROS && allow_unknown) { //未知区域
                        v = COST_OBS - 1;
                        *cm = v;
                    }
                }
            }
        } else { //非ROS map， 如 PGM 简单的灰度图像格式 ，会在可行范围内再创建上下左右7像素的边界缓冲区，这缓冲区表示无效数据或者噪声
            for (int i = 0; i < ny; i ++) {
                int k = i * nx;
                for (int j = 0; j < nx; j ++, k++, cmap++, cm ++) {
                    *cm = COST_OBS;
                    if (i < 7 || i > ny - 8 || j < 7 || j > nx - 8) {
                        continue;
                    }
                    int v = *cmap;
                    if (v < COST_OBS_ROS) {
                        v = COST_NEUTRAL + COST_FACTOR * v;
                        if (v >= COST_OBS) {
                            v = COST_OBS - 1;
                        }
                        *cm = v;
                    } else if (v == COST_UNKNOW_ROS) {
                        v = COST_OBS - 1;
                        *cm = v;
                    }
                }
            }
        }
    }

    bool NavFn::calcNavFnDijkstra(std::function<bool()> cancelChecker, bool atStart) 
    {
        setupNavFn(true);

        return propNavFnDijsktra(std::max(nx * ny / 20, nx + ny), cancelChecker, atStart);
    }

    bool NavFn::calcNavFnAstar(std::function<bool()> cancelChecker) {
        setupNavFn(true);

        return propNavFnAstar(std::max(nx *ny / 20, nx + ny), cancelChecker);;
    }

    float * NavFn::getPathX() { return pathx ;}
    float * NavFn::getPATHY() { return pathy ;}
    int NavFn::getPathLen() { return npath; }

    /**
     * inserting onto the priority blocks, 
     * 分级缓存，分了三个队列来处理
     * 提高缓存局部性，减少优先队列的插入/删除操作， 提高CPU缓存命中率
     * 一个队列 O(logn), 多级队列近似 O(1)操作
    */ 
    #define push_cur(n)  {if (n >= 0 && n < ns && !pending[n] && \
        costarr[n] < COST_OBS && curPe < PRIORITYBUFSIZE) \
        {curP[curPe++] = n; pending[n] = true;}}
    #define push_next(n) {if (n >= 0 && n < ns && !pending[n] && \
        costarr[n] < COST_OBS && nextPe < PRIORITYBUFSIZE) \
        {nextP[nextPe++] = n; pending[n] = true;}}
    #define push_over(n) {if (n >= 0 && n < ns && !pending[n] && \
        costarr[n] < COST_OBS && overPe < PRIORITYBUFSIZE) \
        {overP[overPe++] = n; pending[n] = true;}}


    void NavFn::setupNavFn(bool keepit) 
    {
        //reset
        for (int i = 0; i < ns; i ++) {
            potarr[i] = POT_HIGH;
            if (!keepit) {
                costarr[i] = COST_NEUTRAL;
            }
            gradx[i] = grady[i] = 0.0;
        }   

        //上边界设置为障碍物
        COSTTYPE *pc = costarr; 
        for (int i = 0; i < nx; i ++) {
            *pc++ = COST_OBS;
        }
        //下边界设置为障碍物
        pc = costarr + (ny - 1) * nx;
        for (int i = 0; i < nx; i ++) {
            *pc++ = COST_OBS;
        }
        //左边界设置为障碍物
        pc = costarr;
        for (int i = 0; i < ny; i++, pc += nx) {
            *pc = COST_OBS;
        }
        //有边界设置为障碍物
        pc = costarr + nx - 1;
        for (int i = 0; i < ny; i ++, pc += nx) {
            *pc = COST_OBS;
        }


        curT = COST_OBS;
        //配置current缓存堆
        curP = pb1;
        curPe = 0;
        //配置下一批待处理的缓存堆
        nextP = pb2;
        nextPe = 0;
        //冗余缓存堆
        overP = pb3;
        overPe = 0;

        memset(pending, 0, ns * sizeof(bool));

        int k = goal[0] + goal[1] * nx;
        initCost(k, 0);


        //find number of obstacle cells
        pc = costarr;
        int ntot = 0;
        for (int i = 0; i < ns; i++, pc++) {
            if (*pc >= COST_OBS) {
                ntot ++;
            }
        }
        nobs = ntot;
    }

    //当前点的上下左右边界点都加入待处理的堆中
    void NavFn::initCost(int k, float v) 
    {
        potarr[k] = v;
        push_cur(k + 1); //x正方形
        push_cur(k - 1); //x负方向
        push_cur(k - nx); //x上方向 (y 负方向)
        push_cur(k + nx); //x下方向 (y 正方向)
    }

    #define INVSQRT2 0.707106781

    //Dijkstra 当前点代价计算并且更新邻居代价
    inline void NavFn::updateCell(int n) 
    {
        //获取上下左右网格到终点的代价
        const float l = potarr[n - 1];
        const float r = potarr[n + 1];
        const float u = potarr[n - nx];
        const float d = potarr[n + nx];

        //获取代价低的路径
        float ta, tc;
        if (l < r) { tc = l ;} else { tc = r ;}
        if (u < d) { ta = u ;} else { ta = d ;}

        if (costarr[n] < COST_OBS) {
            float hf = static_cast<float>(costarr[n]); //当前点的通行代价
            float dc = tc - ta;
            if (dc < 0) {
                dc = -dc; //存储的代价差的绝对值
                ta = tc; //临居方向中最小的代价(到目标点的代价)
            }
            
            //计算新的梯度代价
            float pot;
            if (dc >= hf) { //梯度大于原本代价，用小的代价 + hf 代替
                pot = ta + hf;  //当前点到终点代价 = 临边通行最小代价 + 当前点通行代价
            } else {
                const float div = dc / hf;
                const float v = -0.2301 * div * div + 0.5307 * div + 0.7040; //波传播算法经验公式，暂时理解不透
                pot = ta + hf * v;
            }

            //当前计算出来的pot 比 potarr[n]代价更小，则替换
            if (pot < potarr[n]) {
                potarr[n] = pot;
                float le = INVSQRT2 * static_cast<float>(costarr[n - 1]);
                float re = INVSQRT2 * static_cast<float>(costarr[n + 1]);
                float ue = INVSQRT2 * static_cast<float>(costarr[n - nx]);
                float de = INVSQRT2 * static_cast<float>(costarr[n + nx]);

                if (pot < curT) { //代价低的pot
                    //当前邻居代价 > 当前点 + le，则更新邻居点，下一轮updateCell 函数会重新计算邻居点代价赋值
                    if (l > pot + le) {push_next(n - 1);} 
                    if (r > pot + re) {push_next(n + 1);}
                    if (u > pot + ue) {push_next(n - nx);}
                    if (d > pot + de) {push_next(n + nx);}
                } else {
                    if (l > pot + le) {push_over(n - 1);}
                    if (r > pot + re) {push_over(n + 1);}
                    if (u > pot + ue) {push_over(n - nx);}
                    if (d > pot + de) {push_over(n + nx);}
                }
            }
        }
    }

    //AStar 当前点代价计算并且更新邻居代价，大部分和Dijkstra 算法兼容
    inline void NavFn::updateCellAstar(int n) 
    {
        // get neighbors
        float l = potarr[n - 1];
        float r = potarr[n + 1];
        float u = potarr[n - nx];
        float d = potarr[n + nx];
        // ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n",
        // potarr[n], l, r, u, d);
        // ROS_INFO("[Update] cost of %d: %d\n", n, costarr[n]);

        // find lowest, and its lowest neighbor
        float ta, tc;
        if (l < r) {tc = l;} else {tc = r;}
        if (u < d) {ta = u;} else {ta = d;}
        // do planar wave update
        if (costarr[n] < COST_OBS) 
        {  // don't propagate into obstacles
            float hf = static_cast<float>(costarr[n]);  // traversability factor
            float dc = tc - ta;  // relative cost between ta,tc
            if (dc < 0) {  // ta is lowest
                dc = -dc;
                ta = tc;
            }

            // calculate new potential
            float pot;
            if (dc >= hf) {  // if too large, use ta-only update
                pot = ta + hf;
            } else {  // two-neighbor interpolation update
                // use quadratic approximation
                // might speed this up through table lookup, but still have to
                //   do the divide
                const float div = dc / hf;
                const float v = -0.2301 * div * div + 0.5307 * div + 0.7040;
                pot = ta + hf * v;
            }

            // ROS_INFO("[Update] new pot: %d\n", costarr[n]);

            // now add affected neighbors to priority blocks
            if (pot < potarr[n]) {
                float le = INVSQRT2 * static_cast<float>(costarr[n - 1]);
                float re = INVSQRT2 * static_cast<float>(costarr[n + 1]);
                float ue = INVSQRT2 * static_cast<float>(costarr[n - nx]);
                float de = INVSQRT2 * static_cast<float>(costarr[n + nx]);

                // calculate distance
                int x = n % nx; //计算n在x轴上的点坐标
                int y = n / nx; //计算n在y轴上的点坐标
                //计算当前点到终点的估算代价 
                float dist = hypot(x - start[0], y - start[1]) * static_cast<float>(COST_NEUTRAL);

                potarr[n] = pot; //这里只保存 起点到当前点的代价 g ，目的是保持potarr纯净，只存储当前点实际代价，而不是当前点到终点的总代价 与Dijkstra算法兼容
                pot += dist; // f(n) = g(n) + h(n) 用来判断最终到终点的总代价
                if (pot < curT) {  // low-cost buffer block
                    if (l > pot + le) {push_next(n - 1);}
                    if (r > pot + re) {push_next(n + 1);}
                    if (u > pot + ue) {push_next(n - nx);}
                    if (d > pot + de) {push_next(n + nx);}
                } else {
                    if (l > pot + le) {push_over(n - 1);}
                    if (r > pot + re) {push_over(n + 1);}
                    if (u > pot + ue) {push_over(n - nx);}
                    if (d > pot + de) {push_over(n + nx);}
                }
            }
        }
    }

    /**
    * 利用 curT += priInc 来一次分层处理三个优先队列代价
    * 固定内存开辟 3个 PRIORITYBUFSIZE大的队列，然后处理点放到这三个队列
    * 思路内存池复用 + 交换队列buffer指针来 代替 一般意义上的open_list 和 close_list 动态分配内存
    */
    bool NavFn::propNavFnDijsktra(int cycles, std::function<bool()> cancelChecker, bool atStart) 
    {
        int nwv = 0; //优先队列最大数量
        int nc = 0;  // 实际放到有限队列的网格数量
        int cycle = 0; // 循环处理次数

        int startCell = start[i]*nx + start[0];

        for (; cycle < cycles; cycle ++) { //达到循环次数上线，或者外部取消
            if (cycle % terminal_checking_interval == 0 && cancelChecker()) {
                throw nav2_core::PlannerCancelled("Planner was cancelled");
            }

            if (curPe == 0 && nextPe == 0) { //优先队列为空
                break ;
            }

            nc += curPe;
            if (curPe > nwv) {
                nwv = curPe;
            }

            int *pb = curP; //优先队列指针
            int i = curPe; //优先队列数量
            while (i-- > 0) { //先清楚待处理标识
                pending[*(pb++)] = false;
            }

            pb = curP;
            i = curPe;
            while(i-- > 0) { //实际处理
                updateCell(*pb++);
            }

            curPe = nextPe; //交换nextPe，目的处理next 队列数据,下次循环处理的就是nextP 队列了？
            nextPe = 0;
            pb = curP;   //swap buffers
            curP = nextP;
            nextP = pb;

            if (curPe == 0) {
                curT += priInc;  // 这部分不太理解
                curPe = overPe; //切换当前的为overflow block
                overPe = 0;
                pb = curP;  //swap buffers
                curP = overP;
                overP = pb;
            }

            if (atStart) {
                if (potarr[startCell] < POT_HIGH) {
                    break ;
                }
            }
        }

        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",cycle, nc, (int)((nc * 100.0) / (ns - nobs)), nwv);

        return (cycle < cycles) ? true : false;
    }

    //
    bool NavFn::propNavFnAstar(int cycles, std::function<bool()> cancelChecker) {
        int nwv = 0;
        int nc = 0;
        int cycle = 0;

        //AStar 启发式估算代价
        float dist = hypot(goal[0] - start[0], goal[1] - start[1]) * static_cast<float>(COST_NEUTRAL);
        curT = dist = curT;

        int startCell = start[1] * nx + start[0];

        for(; cycle < cycles; cycle++) {
            if (cycle % terminal_checking_interval == 0 && cancelChecker()) {
                throw nav2_core::PlannerCancelled("Planner was cancelled");
            }

            if (curPe == 0 && nextPe == 0) { //优先队列为空
                break;
            }

            nc += curPe;
            if (curPe > nwv) {
                nwv = curPe;
            }

            int *pb = curP;
            int i = curPe;
            while(i-- > 0) {
                pending[*(pb++)] = false;
            }

            pb = curP;
            i = curPe;
            while(i-- > 0) {
                updateCellAstar(*pb++);
            }

            curPe = nextPe;
            nextPe = 0;
            pb = curP;  //swap buffers
            curP = nextP;
            nextP = pb;

            if (curPe == 0) {
                curT += priInc;
                curPe = overPe;
                overPe = 0;
                pb = curP;
                curP = overP;
                overP = pb;
            }

            //找到相近的点退出循环
            if (potarr[startCell < POT_HIGH]) {
                break ;
            }
        }

        last_path_cost_ = potarr[startCell];
        RCLCPP_DEBUG(
            rclcpp::get_logger("rclcpp"),
            "[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
            cycle, nc, (int)((nc * 100.0) / (ns - nobs)), nwv);

        if (potarr[startCell] < POT_HIGH) {
            return true;
        }
        return false;
    }

    float NavFn::getLastPathCost() 
    {
        return last_path_cost_;
    }

    /**
    * 从起点开始，沿着场下降的方向，生成一条平滑的连续路径达到目标点
    * 终止条件 是达到目标/n 次数耗尽
    * 策略1: 边界/障碍物/震荡情况，直接跳到8个邻域场势最小的点
    * 策略2：正常梯度下降
    *        计算四个角点梯度
    *        双线性插值得到精确地图方向(x,y)
    *        根据固定步长pathstep 来更新dx和dy
    */
    int NavFn::calcPath(int n, int *st) 
    {
        if (npathbuf < n) {
            if (pathx) { delete[] pathx;}
            if (pathy) { delete[] pathy;}
            pathx = new float[n];
            pathy = new float[n];
            npathbuf = n;
        }

        if (st == nullptr) {st = start; }
        if stc = st[1] * nx + st[0];

        float dx = 0; //初始化机器人在当前网格的dx位置 为0
        float dy = 0; //初始化机器人在当前网格的dy位置 为0

        npath = 0; //

        for (int i = 0; i < n; i++) {
            //max 和min 目的排除异常(dx, dy)点
            int nearest_point = std::max(
                0,
                std::min(
                    nx * ny - 1, stc + static_cast<int>(round(dx)) + static_cast<int>(nx * round(dy))));
            
            //场势很小接近于COST_NEUTRAL 表示达到目标区域
            if (potarr[nearest_point] < COST_NEUTRAL) {
                pathx[npath] = static_cast<float>(goal[0]);
                pathy[npath] = static_cast<float>(goal[1]);
                return ++npath;
            }

            if (stc < nx || stc > ns - nx) {
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Out of bounds");
                return 0;
            }

            pathx[npath] = stc % nx + dx; //行索引
            pathy[npath] = stc / nx + dy; //列索引
            npath++;

            //震荡检测  A -> B -> A 来回走
            bool oscillation_detected = false;
            if (npath > 2 && 
                pathx[npath - 1] == pathx[npath - 3] &&
                pathy[npath - 1] == pathy[npath - 3]) 
                {
                    RCLCPP_DEBUG(
                        rclcpp::get_logger("rclcpp"),
                        "[PathCalc] oscillation detected, attempting fix.");
                    
                    oscillation_detected = true;
                }

            int stcnx = stc + nx; //当前点正下方的点
            int stcpx = stc - nx; //当前点正上方的点

            //遍历处理当前点的上下左右前后
            if (potarr[stc] >= POT_HIGH ||
                potarr[stc + 1] >= POT_HIGH ||
                potarr[stc - 1] >= POT_HIGH ||
                potarr[stcnx] >= POT_HIGH ||
                potarr[stcnx + 1] >= POT_HIGH ||
                potarr[stcnx - 1] >= POT_HIGH ||
                potarr[stcpx] >= POT_HIGH ||
                potarr[stcpx + 1] >= POT_HIGH ||
                potarr[stcpx - 1] >= POT_HIGH ||
                oscillation_detected) {
                RCLCPP_DEBUG(
                    rclcpp::get_logger("rclcpp"),
                    "[Path] Pot fn boundary, following grid (%0.1f/%d)", potarr[stc], npath);
                
                int minc = stc; //先默认当前点对应的索引是最小值
                int minp = potarr[stc];//默认当前点对应的代价是最小值
                int sti = stcpx - 1;  //正上方点 - 1
                if (potarr[sti] < minp) { minp = potarr[sti]; minc = sti;}
                sti++; //正上方点
                if (potarr[sti] < minp) { minp = potarr[sti]; minc = sti;}
                sti++; //正上方点 + 1
                if (potarr[sti] < minp) { minp = potarr[sti]; minc = sti;}
                sti = stc - 1; //当前点 - 1
                if (potarr[sti] < minp) { minp = potarr[sti]; minc = sti;}
                sti = stc + 1; //当前点 + 1
                if (potarr[sti] < minp) { minp = potarr[sti]; minc = sti;}
                sti = stnx - 1; //正下方点 - 1
                if (potarr[sti] < minp) { minp = potarr[sti]; minc = sti;}
                sti++; //正下方点
                if (potarr[sti] < minp) { minp = potarr[sti]; minc = sti;}
                sti++; //正下方点 + 1
                if (potarr[sti] < minp) { minp = potarr[sti]; minc = sti;}
                stc = minc;
                dx = 0;
                dy = 0;
            } else {
                //计算以当前点为左上角原点的方格内的四个点的场势变化率
                gradCell(stc); //左上
                gradCell(stc + 1); //右上
                gradCell(stnx); //左下
                gradCell(stnx + 1); //右下

                //在gradx 场势下 求stc 和stc+1 之间 dx 位置的需要具体移动的趋势值(理解为加速度), 
                float x1 = (1.0 - dx) * gradx[stc] + dx * gradx[stc + 1];
                //求stcnx 和stcnx+1 之间 dx 位置的具体值
                float x2 = (1.0 - dx) * gradx[stcnx] + dx *gradx[stcnx + 1];
                float x = (1.0 - dy) * x1 + dy * x2;

                //在grady 场势下 求stc 和stc+1 之间 dx 位置的需要具体移动的趋势值(理解为加速度), 
                float y1 = (1.0 - dx) * grady[stc] + dx * grady[stc + 1];
                float y2 = (1.0 - dx) * grady[stcnx] + dx * grady[stcnx + 1];
                float y = (1.0 - dy) * y1 + dy * y2;

                #if 0
                // show gradients
                RCLCPP_DEBUG(
                    rclcpp::get_logger("rclcpp"),
                    "[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n",
                    gradx[stc], grady[stc], gradx[stc + 1], grady[stc + 1],
                    gradx[stcnx], grady[stcnx], gradx[stcnx + 1], grady[stcnx + 1],
                    x, y);
                #endif
                
                if (x == 0.0 && y == 0.0) {
                    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Zero gradient");
                    return 0;
                }

                //计算最终移动的距离
                float ss = pathStep / hypot(x, y);
                dx += x *ss;
                dy += y *ss;

                // 当子网格位置超出[0,1]范围时，切换到相邻网格
                if (dx > 1.0) {stc++; dx -= 1.0;}
                if (dx < -1.0) {stc--; dx += 1.0;}
                if (dy > 1.0) {stc += nx; dy -= 1.0;}
                if (dy < -1.0) {stc -=nx; dy += 1.0;}
            } 
        }
        //  return npath;  // out of cycles, return failure
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, path too long");
        // savemap("navfn_pathlong");
        return 0;  // out of cycles, return failure
    }

    //计算当前点n 在gradx 和grady 场势变化率
    float NavFn::gradCell(int n) 
    {
        //gradx 和grady 初始值是0，计算过后就不是了，
        if (gradx[n] + grady[n] > 0.0) {
            return 1.0;
        }

        if (n < nx || n > ns - nx) {
            return 0.0;
        }

        float cv = potarr[n];
        float dx = 0.0; //水平方向的势场变化率， 正值-> 势场向右减小向右走， 负值->势场向左减小 向左走
        float dy = 0.0; //垂直方向的势场变化率，正值 → 势场向下减小（应该向下走） 负值 → 势场向上减小（应该向上走）

        if (cv >= POT_HIGH) { //处理当前点在障碍物内
            if (potarr[n - 1] < POT_HIGH) { 
                dx = -COST_OBS; //左边可通行
            } else if (potarr[n + 1] < POT_HIGH) {
                dx = COST_OBS; //右边可通行
            }

            if (potarr[n - nx] < POT_HIGH) {
                dy = -COST_OBS; //上边可通行
            } else if (potarr[n + nx] > POT_HIGH) {
                dy = COST_OBS; //下边可通行
            }
        } else { //不在障碍物内
            if (potarr[n  1] < POT_HIGH) {
                dx += potarr[n - 1] - cv; //左邻居贡献
            }
            if (potarr[n + 1] < POT_HIGH) {
                dx += cv - potarr[n + 1];
            }

            if (potarr[n - nx] < POT_HIGH) {
                dy += potarr[n - nx] - cv; //上边可通行
            }

            if (potarr[n + nx] < POT_HIGH) {
                dy += cv - potarr[n + nx];
            }
        }

        float norm = hypot(dx, dy); //计算dx, dy梯度向量长度 √(dx² + dy²) 计算三角形斜边长
        if (norm > 0) {
            norm = 1.0 / norm;
            gradx[n] = norm * dx; //x分量
            grady[n] = norm * dy; //y分量
        }

        return norm;
    }
}