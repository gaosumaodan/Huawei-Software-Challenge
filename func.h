double round_angle(double robot_posx,double robot_posy,double stage_posx,double stage_posy,double robot_toward)
{//算出两个向量之间的夹角(返回目标角减去朝向角)
    double Ax=stage_posx-robot_posx;
    double Ay=stage_posy-robot_posy;
    double theta = atan2(Ay, Ax);
    return theta - robot_toward;
}

double angle(double robot_posx,double robot_posy,double stage_posx,double stage_posy){
    double Ax=stage_posx-robot_posx;
    double Ay=stage_posy-robot_posy;
    double theta = atan2(Ay, Ax);
    return theta;
}

bool ifslow(double robotx,double roboty,double stagex,double stagey)
{   //判断是否满足减速到零的(抵达工具台)条件
    double Ax=stagex-robotx;
    double Bx=stagey-roboty;
    double distance=sqrt(Ax*Ax+Bx*Bx);
    return distance<0.4;
}

struct robotCrash{
    int robotId, crashId;
    double x, y;
};

robotCrash crash[4];

string atob(int x)//十进制转二进制函数
{
    string s;
    s.resize(9);
    for(int i=0;i<9;i++)
    {
        s[i]='0';
    }
    int n=0;
    while(x!=0)
    {
        s[n]=((x%2)+'0');
        x=x/2;
        n++;
    }
    return s;
}

bool if_robot_access(read_by_frame &f,int robotID,int stageId)//判断robot是否距离目标足够近，以调用更精细的导航函数
{
    double Ax=f.robot[robotID].x;
    double Ay=f.robot[robotID].y;
    double Bx=f.stage[stageId].x;
    double By=f.stage[stageId].y;
    double distance=sqrt(pow((Ax-Bx),2)+pow((Ay-By),2));
    return distance<2.2;
}

bool if_have_stage(read_by_frame f, int type){
    for(int i = 0; i < stagingNum; i++){
        if(f.stage[i].type == type)
            return true;
    }
    return false;
}

bool if_square_have(double x, double y){
    if(((int)(x * 100)) % 50 == 0){
        int y1 = 99 - y * 2, x1 = x * 2 - 1;
        if(Map[y1 + 1][x1 + 1] == '.' && Map[y1][x1] == '.' &&
           Map[y1][x1 + 1] == '.' && Map[y1 + 1][x1] == '.')
            return true;
    }
    else{
        int y1 = 99.5 - y * 2, x1 = x * 2 - 0.5;
        if((Map[y1 - 1][x1 - 1] == '.' && Map[y1 - 1][x1] == '.' && Map[y1 - 1][x1 + 1] == '.' && 
            Map[y1][x1 - 1] == '.' && Map[y1][x1] == '.' && Map[y1][x1 + 1] == '.') ||
           (Map[y1][x1 - 1] == '.' && Map[y1][x1] == '.' && Map[y1][x1 + 1] == '.' && 
            Map[y1 + 1][x1 - 1] == '.' && Map[y1 + 1][x1] == '.' && Map[y1 + 1][x1 + 1] == '.') ||
           (Map[y1 - 1][x1 + 1] == '.' && Map[y1][x1 + 1] == '.' && Map[y1 + 1][x1 + 1] == '.' && 
            Map[y1 - 1][x1] == '.' && Map[y1][x1] == '.' && Map[y1 + 1][x1] == '.') ||
           (Map[y1 - 1][x1] == '.' && Map[y1][x1] == '.' && Map[y1 + 1][x1] == '.' && 
            Map[y1 - 1][x1 - 1] == '.' && Map[y1][x1 - 1] == '.' && Map[y1 + 1][x1 - 1] == '.'))
           return true;
    }
    return false;
    // if(((int)(x * 100)) % 50 == 0){
    //     int y1 = 99 - y * 2, x1 = x * 2 - 1;
    //     if(Map[y1 + 1][x1 + 1] == '#' || Map[y1][x1] == '#' ||
    //        Map[y1][x1 + 1] == '#' || Map[y1 + 1][x1] == '#')
    //         return false;
    // }
    // else{
    //     int y1 = 99.5 - y * 2, x1 = x * 2 - 0.5;
    //     if(Map[y1][x1 - 1] == '#' || Map[y1 - 1][x1 + 1] == '#' || 
    //        Map[y1][x1 + 1] == '#' || Map[y1 + 1][x1 - 1] == '#' || 
    //        Map[y1 - 1][x1] == '#' || Map[y1 - 1][x1 - 1] == '#' || 
    //        Map[y1 + 1][x1] == '#' || Map[y1 + 1][x1 + 1] == '#')
    //        return false;
    //     }
    // return true;
}

robotCrash find_space(read_by_frame f, int robotID, int crashID, int num){
    crashPoint = path[0][0].end();
    if(robotID == 0)
        tempPoint = startXy0;
    else if(robotID == 1)
        tempPoint = startXy1;
    else if(robotID == 2)
        tempPoint = startXy2;
    else
        tempPoint = startXy3;
    if(crashID == 0)
        crashPoint = startXy0;
    else if(crashID == 1)
        crashPoint = startXy1;
    else if(crashID == 2)
        crashPoint = startXy2;
    else
        crashPoint = startXy3;
    crashStart = crashPoint;
    auto it = path[task_start[robotID]][task_toward[robotID]].begin();
    if(tempPoint != it){
        tempPoint--;
    }
    for(int i = 0;; i++){
        double x = (*crashPoint)->x, y = (*crashPoint)->y;
        if(((int)(x * 100)) % 50 == 0){
            int y1 = 99 - y * 2, x1 = x * 2 - 1;
            Map[y1 + 1][x1 + 1] = '#', Map[y1][x1] = '#', Map[y1][x1 + 1] = '#', Map[y1 + 1][x1] = '#';
        }
        else{
            int y1 = 99.5 - y * 2, x1 = x * 2 - 0.5;
            Map[y1][x1 - 1] = '#', Map[y1 - 1][x1 + 1] = '#', Map[y1][x1 + 1] = '#', Map[y1 + 1][x1 - 1] = '#', 
            Map[y1 - 1][x1] = '#', Map[y1 - 1][x1 - 1] = '#', Map[y1 + 1][x1] = '#', Map[y1 + 1][x1 + 1] = '#';
        }
        crashPoint++;
        if(crashPoint == path[task_start[crashID]][task_toward[crashID]].end() || crashPoint == pathRobot[crashID].end())
            break;
    }
    for(double a = 0.5;; a += 0.5){
        for(double b = 0.5; b <= a; b += 0.5){
            for(int c = -1; c <= 1; c += 2){
                for(int d = -1; d <= 1; d += 2){
                    double x1 = (*tempPoint)->x + a * c, y1 = (*tempPoint)->y + b * d;
                    if(if_square_have((*tempPoint)->x + a * c, (*tempPoint)->y + b * d)){
                        pair<double, double> target;
                        target.first = (*tempPoint)->x + a * c, target.second = (*tempPoint)->y + b * d;
                        crashPoint = crashStart;
                        for(int i = 0;; i++){
                            double x = (*crashPoint)->x, y = (*crashPoint)->y;
                            if(((int)(x * 100)) % 50 == 0){
                                int y1 = 99 - y * 2, x1 = x * 2 - 1;
                                Map[y1 + 1][x1 + 1] = '.', Map[y1][x1] = '.', Map[y1][x1 + 1] = '.', Map[y1 + 1][x1] = '.';
                            }
                            else{
                                int y1 = 99.5 - y * 2, x1 = x * 2 - 0.5;
                                Map[y1][x1 - 1] = '.', Map[y1 - 1][x1 + 1] = '.', Map[y1][x1 + 1] = '.', Map[y1 + 1][x1 - 1] = '.', 
                                Map[y1 - 1][x1] = '.', Map[y1 - 1][x1 - 1] = '.', Map[y1 + 1][x1] = '.', Map[y1 + 1][x1 + 1] = '.';
                            }
                            crashPoint++;
                            if(crashPoint == path[task_start[crashID]][task_toward[crashID]].end() || crashPoint == pathRobot[crashID].end())
                                break;
                        }
                        crashPoint = crashStart;
                        for(int i = 0; i <= num; i++){
                            crashPoint++;
                        }
                        double x = (*crashPoint)->x, y = (*crashPoint)->y;
                        return {robotID, crashID, target.first, target.second};
                    }
                }
            }
        }
    }
    return {0, 0, 0, 0};
}

robotCrash judge_crash(read_by_frame f, int robotID){
    auto crash0 = startXy0, crash1 = startXy1, crash2 = startXy2, crash3 = startXy3;
    for(int i = 0; i < 10; i++){
        if(robotID == 0 && crash1 != path[task_start[1]][task_toward[1]].end() && crash1 != path[0][0].end() && crash1 != pathRobot[1].end()){
            if((*crash0)->x >= (*crash1)->x - 0.25 && (*crash0)->y >= (*crash1)->y - 0.25 && 
               (*crash0)->x <= (*crash1)->x + 0.25 && (*crash0)->y <= (*crash1)->y + 0.25)
                return find_space(f, 0, 1, i);
            crash1++;
        }
        if(robotID == 0 && crash2 != path[task_start[2]][task_toward[2]].end() && crash2 != path[0][0].end() && crash2 != pathRobot[2].end()){
            if((*crash0)->x >= (*crash2)->x - 0.25 && (*crash0)->y >= (*crash2)->y - 0.25 && 
               (*crash0)->x <= (*crash2)->x + 0.25 && (*crash0)->y <= (*crash2)->y + 0.25)
                return find_space(f, 0, 2, i);
            crash2++;
        }
        if(robotID == 0 && crash3 != path[task_start[3]][task_toward[3]].end() && crash3 != path[0][0].end() && crash3 != pathRobot[3].end()){
            if((*crash0)->x >= (*crash3)->x - 0.25 && (*crash0)->y >= (*crash3)->y - 0.25 && 
               (*crash0)->x <= (*crash3)->x + 0.25 && (*crash0)->y <= (*crash3)->y + 0.25)
                return find_space(f, 0, 3, i);
            crash3++;
        }
        if(robotID == 1 && crash2 != path[task_start[2]][task_toward[2]].end() && crash2 != path[0][0].end() && crash2 != pathRobot[2].end()){
            if((*crash1)->x >= (*crash2)->x - 0.25 && (*crash1)->y >= (*crash2)->y - 0.25 && 
               (*crash1)->x <= (*crash2)->x + 0.25 && (*crash1)->y <= (*crash2)->y + 0.25)
                return find_space(f, 1, 2, i);
            crash2++;
        }
        if(robotID == 1 && crash3 != path[task_start[3]][task_toward[3]].end() && crash3 != path[0][0].end() && crash3 != pathRobot[3].end()){
            if((*crash1)->x >= (*crash3)->x - 0.25 && (*crash1)->y >= (*crash3)->y - 0.25 && 
               (*crash1)->x <= (*crash3)->x + 0.25 && (*crash1)->y <= (*crash3)->y + 0.25)
                return find_space(f, 1, 3, i);
            crash3++;
        }
        if(robotID == 2 && crash3 != path[task_start[3]][task_toward[3]].end() && crash3 != path[0][0].end() && crash3 != pathRobot[3].end()){
            if((*crash2)->x >= (*crash3)->x - 0.25 && (*crash2)->y >= (*crash3)->y - 0.25 && 
               (*crash2)->x <= (*crash3)->x + 0.25 && (*crash2)->y <= (*crash3)->y + 0.25)
                return find_space(f, 2, 3, i);
            crash3++;
        }
    }
    return {0, 0, 0, 0};
} //判断是否即将发生碰撞

void move1(read_by_frame f, int robotID, double targetX, double targetY){
    double a = 0.3, b = angle(f.robot[robotID].x, f.robot[robotID].y, targetX, targetY);
    if((b > pi / 4 - 0.1 && b < pi / 4 + 0.1) || (b > -3 * pi / 4 - 0.1 && b < -3 * pi / 4 + 0.1)||
       (b > -pi / 4 - 0.1 && b < -pi / 4 + 0.1) || (b > 3 * pi / 4 - 0.1 && b < pi / 4 + 0.1))
       a = 0.1;
    double angle=round_angle(f.robot[robotID].x, f.robot[robotID].y, targetX, targetY, f.robot[robotID].forward);
    if(fabs(angle) < a){
        printf("rotate %d %f\n", robotID, 0.0);
        printf("forward %d %f\n", robotID, 6.0);
        return;
    }
    else{
        if(angle >= 0){
            if(fabs(angle) > pi){
                printf("forward %d %f\n", robotID, 0);
                printf("rotate %d %f\n", robotID, -pi);
            }
            else{
                printf("forward %d %f\n",robotID, 0);
                printf("rotate %d %f\n", robotID, pi);
            }
        }
        else{
            if(fabs(angle)>pi){
                printf("forward %d %f\n",robotID, 0);
                printf("rotate %d %f\n", robotID, pi);
            }
            else{
                printf("forward %d %f\n",robotID, 0);
                printf("rotate %d %f\n", robotID, -pi);
            }  
        }
        return;
    }
}

void move(read_by_frame f, int robotID, int stageID){
    if(frameID == 2030)
        int a = 0;
    if(robotID == 0 && (startXy0 == pathRobot[0].end() || startXy0 == path[0][0].end())){   
        startXy0 = path[f.robot[robotID].inStage][stageID].begin();
        task_start[0] = f.robot[robotID].inStage;
    }
    if(robotID == 0 && startXy0 != pathRobot[0].end() && startXy0 != path[0][0].end()){
        move1(f, robotID, (*startXy0)->x, (*startXy0)->y);
        auto startXy = startXy0;
        startXy++;
        if(startXy != path[task_start[robotID]][task_toward[robotID]].end() && startXy != pathRobot[robotID].end()){
            if(f.robot[robotID].x > (*startXy0)->x - 0.5 && f.robot[robotID].y > (*startXy0)->y - 0.5 && 
               f.robot[robotID].x < (*startXy0)->x + 0.5 && f.robot[robotID].y < (*startXy0)->y + 0.5)
                startXy0++;
        }
    }
    if(robotID == 1 && (startXy1 == pathRobot[1].end() || startXy1 == path[0][0].end())){   
        startXy1 = path[f.robot[robotID].inStage][stageID].begin();
        task_start[1] = f.robot[robotID].inStage;
    }
    if(robotID == 1 && startXy1 != pathRobot[1].end() && startXy1 != path[0][0].end()){
        move1(f, robotID, (*startXy1)->x, (*startXy1)->y);
        auto startXy = startXy1;
        startXy++;
        if(startXy != path[task_start[robotID]][task_toward[robotID]].end() && startXy != pathRobot[robotID].end()){
            if(f.robot[robotID].x > (*startXy1)->x - 0.5 && f.robot[robotID].y > (*startXy1)->y - 0.5 && 
               f.robot[robotID].x < (*startXy1)->x + 0.5 && f.robot[robotID].y < (*startXy1)->y + 0.5)
                startXy1++;
        }
    }
    if(robotID == 2 && (startXy2 == pathRobot[2].end() || startXy2 == path[0][0].end())){   
        startXy2 = path[f.robot[robotID].inStage][stageID].begin();
        task_start[2] = f.robot[robotID].inStage;
    }
    if(robotID == 2 && startXy2 != pathRobot[2].end() && startXy2 != path[0][0].end()){
        move1(f, robotID, (*startXy2)->x, (*startXy2)->y);
        auto startXy = startXy2;
        startXy++;
        if(startXy != path[task_start[robotID]][task_toward[robotID]].end() && startXy != pathRobot[robotID].end()){
            if(f.robot[robotID].x > (*startXy2)->x - 0.5 && f.robot[robotID].y > (*startXy2)->y - 0.5 && 
               f.robot[robotID].x < (*startXy2)->x + 0.5 && f.robot[robotID].y < (*startXy2)->y + 0.5)
                startXy2++;
        }
    }
    if(robotID == 3 && (startXy3 == pathRobot[3].end() || startXy3 == path[0][0].end())){   
        startXy3 = path[f.robot[robotID].inStage][stageID].begin();
        task_start[3] = f.robot[robotID].inStage;
    }
    if(robotID == 3 && startXy3 != pathRobot[3].end() && startXy3 != path[0][0].end()){
        move1(f, robotID, (*startXy3)->x, (*startXy3)->y);
        auto startXy = startXy3;
        startXy++;
        if(startXy != path[task_start[robotID]][task_toward[robotID]].end() && startXy != pathRobot[robotID].end()){
            if(f.robot[robotID].x > (*startXy3)->x - 0.5 && f.robot[robotID].y > (*startXy3)->y - 0.5 && 
               f.robot[robotID].x < (*startXy3)->x + 0.5 && f.robot[robotID].y < (*startXy3)->y + 0.5)
                startXy3++;
        }
    }
    if(crash_cross[robotID] == 0 && crash_judge[robotID] == 0)
        crash[robotID] = judge_crash(f, robotID);
    auto startXY1 = startXy1, startXY2 = startXy2, startXY3 = startXy3;
    double x, y;
    if(crashPoint != path[0][0].end() && crashPoint != path[task_start[0]][task_toward[0]].end() &&
       crashPoint != path[task_start[1]][task_toward[1]].end() && crashPoint != path[task_start[2]][task_toward[2]].end() && 
       crashPoint != path[task_start[3]][task_toward[3]].end())
        x = (*crashPoint)->x, y = (*crashPoint)->y;
    if(f.robot[robotID].x < x + 0.2 && f.robot[robotID].x > x - 0.2 &&
       f.robot[robotID].y < y + 0.2 && f.robot[robotID].y > y - 0.2)
        crash_judge[robotID] = 0;
    if((crash[robotID].crashId == 1 &&
       ++startXY1 != pathRobot[1].end() && startXY1 != path[task_start[1]][task_toward[1]].end() && startXY1 != path[0][0].end()) ||
       (crash[robotID].crashId == 2 &&
       ++startXY2 != pathRobot[2].end() && startXY2 != path[task_start[2]][task_toward[2]].end() && startXY2 != path[0][0].end()) ||
       (crash[robotID].crashId == 3 &&
       ++startXY3 != pathRobot[3].end() && startXY3 != path[task_start[3]][task_toward[3]].end() && startXY3 != path[0][0].end())){
        if((crash[robotID].crashId == 1 && (*startXy1)->x <= x + 0.25 && (*startXy1)->x >= x - 0.25 && 
            (*startXy1)->y <= y + 0.25 && (*startXy1)->y >= y - 0.25) || 
            (crash[robotID].crashId == 2 && (*startXy2)->x <= x + 0.25 && (*startXy2)->x >= x - 0.25 && 
            (*startXy2)->y <= y + 0.25 && (*startXy2)->y >= y - 0.25) || 
            (crash[robotID].crashId == 3 && (*startXy3)->x <= x + 0.25 && (*startXy3)->x >= x - 0.25 && 
            (*startXy3)->y <= y + 0.25 && (*startXy3)->y >= y - 0.25)){
            crash_cross[robotID] = 0;
            crash[robotID].crashId = 0, crash[robotID].robotId = 0, pathCrash[robotID].clear();
        }
        else{
            move1(f, robotID, crash[robotID].x, crash[robotID].y);
            crash_cross[robotID] = 1, crash_judge[robotID] = 1;
            if(f.robot[robotID].x <= crash[robotID].x + 0.2 && f.robot[robotID].x >= crash[robotID].x - 0.2 &&
                f.robot[robotID].y <= crash[robotID].y + 0.2 && f.robot[robotID].y >= crash[robotID].y - 0.2){
                printf("rotate %d %f\n", robotID, pi);
                printf("forward %d %f\n", robotID, 0);
                tempFrameID[robotID]++;
                if(tempFrameID[robotID] == 200){
                    tempFrameID[robotID] = 0;
                    crash_cross[robotID] = 0;
                    crash[robotID].crashId = 0, crash[robotID].robotId = 0, pathCrash[robotID].clear();
                }
            }
            // if(robotID == 0){
            //     if(pathCrash[robotID].empty()){
            //         Astar astar;
            //         astar.Init(Map);
            //         Point start((*startXy0)->x, (*startXy0)->y);
            //         Point end(crash[robotID].x, crash[robotID].y);
            //         pathCrash[robotID] = astar.GetPath_1_5(start, end, false);
            //         CrashXy0 = pathCrash[0].begin();
            //     }
            //     else{
            //         if(robotID == 0 && CrashXy0 != pathCrash[0].end()){
            //             move1(f, robotID, (*CrashXy0)->x, (*CrashXy0)->y);
            //             auto CrashXy = CrashXy0;
            //             CrashXy++;
            //             if(CrashXy != path[task_start[robotID]][task_toward[robotID]].end() && CrashXy != pathRobot[robotID].end()){
            //                 if(f.robot[robotID].x > (*CrashXy0)->x - 0.5 && f.robot[robotID].y > (*CrashXy0)->y - 0.5 && 
            //                 f.robot[robotID].x < (*CrashXy0)->x + 0.5 && f.robot[robotID].y < (*CrashXy0)->y + 0.5)
            //                     CrashXy0++;
            //             }
            //         }
            //     }
            // }
            // else if(robotID == 1){
            //     if(pathCrash[robotID].empty()){
            //         Astar astar;
            //         astar.Init(Map);
            //         Point start((*startXy1)->x, (*startXy1)->y);
            //         Point end(crash[robotID].x, crash[robotID].y);
            //         pathCrash[robotID] = astar.GetPath_1_5(start, end, false);
            //         CrashXy1 = pathCrash[1].begin();
            //     }
            //     else{
            //         if(robotID == 1 && CrashXy1 != pathCrash[1].end()){
            //             move1(f, robotID, (*CrashXy1)->x, (*CrashXy1)->y);
            //             auto CrashXy = CrashXy1;
            //             CrashXy++;
            //             if(CrashXy != path[task_start[robotID]][task_toward[robotID]].end() && CrashXy != pathRobot[robotID].end()){
            //                 if(f.robot[robotID].x > (*CrashXy1)->x - 0.5 && f.robot[robotID].y > (*CrashXy1)->y - 0.5 && 
            //                 f.robot[robotID].x < (*CrashXy1)->x + 0.5 && f.robot[robotID].y < (*CrashXy1)->y + 0.5)
            //                     CrashXy1++;
            //             }
            //         }
            //     }
            // }
            // else if(robotID == 2){
            //     if(pathCrash[robotID].empty()){
            //         Astar astar;
            //         astar.Init(Map);
            //         Point start((*startXy2)->x, (*startXy2)->y);
            //         Point end(crash[robotID].x, crash[robotID].y);
            //         pathCrash[robotID] = astar.GetPath_1_5(start, end, false);
            //         CrashXy2 = pathCrash[2].begin();
            //     }
            //     else{
            //         if(robotID == 2 && CrashXy2 != pathCrash[2].end()){
            //             move1(f, robotID, (*CrashXy2)->x, (*CrashXy2)->y);
            //             auto CrashXy = CrashXy2;
            //             CrashXy++;
            //             if(CrashXy != path[task_start[robotID]][task_toward[robotID]].end() && CrashXy != pathRobot[robotID].end()){
            //                 if(f.robot[robotID].x > (*CrashXy2)->x - 0.5 && f.robot[robotID].y > (*CrashXy2)->y - 0.5 && 
            //                 f.robot[robotID].x < (*CrashXy2)->x + 0.5 && f.robot[robotID].y < (*CrashXy2)->y + 0.5)
            //                     CrashXy2++;
            //             }
            //         }
            //     }
            // }
        }
    }
}

bool ask_target_occupied(read_by_frame &f,int productID,int *task_toward)//取物时，问询所取物品要送去的目标工作台材料格是否占满
{
    if(productID==1)
    {
        int have=0;
        for(auto it=f.stage_type[4].begin();it!=f.stage_type[4].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[1]=='0')
            {
                have++;
            }
        }
        for(auto it=f.stage_type[5].begin();it!=f.stage_type[5].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[1]=='0')
            {
                have++;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.stage[task_toward[i]].type==1&&f.robot[i].productType==0)
            {
                have--;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.robot[i].productType==1)
            {
                have--;
            }
        }
        return have<=0;     
    }
    if(productID==2)
    {
        int have=0;
        for(auto it=f.stage_type[4].begin();it!=f.stage_type[4].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[2]=='0')
            {
                have++;
            }
        }
        for(auto it=f.stage_type[6].begin();it!=f.stage_type[6].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[2]=='0')
            {
                have++;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.stage[task_toward[i]].type==2&&f.robot[i].productType==0)
            {
                have--;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.robot[i].productType==2)
            {
                have--;
            }
        }
        return have<=0;     
    }
    if(productID==3)
    {
        int have=0;
        for(auto it=f.stage_type[5].begin();it!=f.stage_type[5].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[3]=='0')
            {
                have++;
            }
        }
        for(auto it=f.stage_type[6].begin();it!=f.stage_type[6].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[3]=='0')
            {
                have++;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.stage[task_toward[i]].type==3&&f.robot[i].productType==0)
            {
                have--;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.robot[i].productType==3)
            {
                have--;
            }
        }
        return have<=0;    
    }
    if(productID==4)
    {
        int have=0;
        for(auto it=f.stage_type[7].begin();it!=f.stage_type[7].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[4]=='0')
            {
                have++;
            }
        }
        if(f.stage_type[7].empty())
        {
            for(auto it=f.stage_type[9].begin();it!=f.stage_type[9].end();it++)
            {
                string s=atob(f.stage[(*it)].materialStatus);
                if(s[4]=='0')
                {
                    return false;
                }
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.stage[task_toward[i]].type==4&&f.robot[i].productType==0)
            {
                have--;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.robot[i].productType==4)
            {
                have--;
            }
        }
        return have<=0;   
    }
    if(productID==5)
    {
        int have=0;
        for(auto it=f.stage_type[7].begin();it!=f.stage_type[7].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[5]=='0')
            {
                have++;
            }
        }
        if(f.stage_type[7].empty())
        {
            for(auto it=f.stage_type[9].begin();it!=f.stage_type[9].end();it++)
            {
                string s=atob(f.stage[(*it)].materialStatus);
                if(s[5]=='0')
                {
                    return false;
                }
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.stage[task_toward[i]].type==5&&f.robot[i].productType==0)
            {
                have--;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.robot[i].productType==5)
            {
                have--;
            }
        }
        return have<=0;  
    }
    if(productID==6)
    {
        int have=0;
        for(auto it=f.stage_type[7].begin();it!=f.stage_type[7].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[6]=='0')
            {
                have++;
            }
        }
        if(f.stage_type[7].empty())
        {
            for(auto it=f.stage_type[9].begin();it!=f.stage_type[9].end();it++)
            {
                string s=atob(f.stage[(*it)].materialStatus);
                if(s[6]=='0')
                {
                    return false;
                }
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.stage[task_toward[i]].type==6&&f.robot[i].productType==0)
            {
                have--;
            }
        }
        for(int i=0;i<4;i++)
        {
            if(f.robot[i].productType==6)
            {
                have--;
            }
        }
        return have<=0; 
    }
    if(productID==7)
    {
        for(auto it=f.stage_type[8].begin();it!=f.stage_type[8].end();it++)
        {
            string s=atob(f.stage[(*it)].materialStatus);
            if(s[7]=='0')
            {
                return false;
            }
        }
        return true;  
    }
    return 0;                    
}

bool ask_if_haveproduct(read_by_frame &f,int productID)//取物时，问询生产该类型材料的工作台是否生产完毕
{
    if(!f.stage_type[productID].empty())
    {
        for(auto it=f.stage_type[productID].begin();it!=f.stage_type[productID].end();it++)
        {
            if(f.stage[(*it)].productStatus==0 || (stagingNum == 18 && (f.stage[(*it)].remainTime > 50 && f.stage[(*it)].productStatus == 0)))
            {
                continue;
            }
            else
            {
                return true;
            }
        }
        return false;
    }
    else
    {
        return false;
    }
}

void get_product(read_by_frame &f,int robotId, int *task_toward, int n)//取物指令
{
    int End;
    int flag = 0;
    for (auto it = f.stage_value_order.begin(); it != f.stage_value_order.end(); it++)
    {
        if(path[f.robot[robotId].inStage][(*it)].empty())
            continue;
        if (f.stage[(*it)].type == 1 && ask_if_haveproduct(f, 1))
        {
            if (f.stage[(*it)].productStatus == 1)
            {
                if (!flag)
                {
                    End = *it;
                    flag = 1;
                }
                if (ask_target_occupied(f, 1, task_toward))
                {
                    continue;
                }
                else
                {
                    if (task_toward[0] != (*it) && task_toward[1] != (*it) && task_toward[2] != (*it) && task_toward[3] != (*it))
                    {
                        if ((*it) != n)
                            task_toward[robotId] = (*it);
                        return;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }
        }
        else if (f.stage[(*it)].type == 2 && ask_if_haveproduct(f, 2))
        {
            if (f.stage[(*it)].productStatus == 1)
            {
                if (!flag)
                {
                    End = *it;
                    flag = 1;
                }
                if (ask_target_occupied(f, 2, task_toward))
                {
                    continue;
                }
                else
                {
                    if (task_toward[0] != (*it) && task_toward[1] != (*it) && task_toward[2] != (*it) && task_toward[3] != (*it))
                    {
                        if ((*it) != n)
                            task_toward[robotId] = (*it);
                        return;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }
        }
        else if (f.stage[(*it)].type == 3 && ask_if_haveproduct(f, 3))
        {

            if (f.stage[(*it)].productStatus == 1)
            {
                if (!flag)
                {
                    End = *it;
                    flag = 1;
                }
                if (ask_target_occupied(f, 3, task_toward))
                {
                    continue;
                }
                else
                {
                    if (task_toward[0] != (*it) && task_toward[1] != (*it) && task_toward[2] != (*it) && task_toward[3] != (*it))
                    {
                        if ((*it) != n)
                            task_toward[robotId] = (*it);
                        return;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }
        }
        else if (f.stage[(*it)].type == 4 && ask_if_haveproduct(f, 4))
        {
            if (f.stage[(*it)].productStatus == 1)
            {
                if (ask_target_occupied(f, 4, task_toward))
                {
                    continue;
                }
                else
                {
                    if ((task_toward[0] != (*it) || f.robot[0].productType != 0) && (task_toward[1] != (*it) || f.robot[1].productType != 0) && (task_toward[2] != (*it) || f.robot[2].productType != 0) && (task_toward[3] != (*it) || f.robot[3].productType != 0))
                    {
                        if ((*it) != n)
                            task_toward[robotId] = (*it);
                        return;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }
        }
        else if (f.stage[(*it)].type == 5 && ask_if_haveproduct(f, 5))
        {
            if (f.stage[(*it)].productStatus == 1)
            {
                if (ask_target_occupied(f, 5, task_toward))
                {
                    continue;
                }
                else
                {
                    if ((task_toward[0] != (*it) || f.robot[0].productType != 0) && (task_toward[1] != (*it) || f.robot[1].productType != 0) && (task_toward[2] != (*it) || f.robot[2].productType != 0) && (task_toward[3] != (*it) || f.robot[3].productType != 0))
                    {
                        if ((*it) != n)
                            task_toward[robotId] = (*it);
                        return;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }
        }
        else if (f.stage[(*it)].type == 6 && ask_if_haveproduct(f, 6))
        {
            if (f.stage[(*it)].productStatus == 1)
            {
                if (ask_target_occupied(f, 6, task_toward))
                {
                    continue;
                }
                else
                {
                    if ((task_toward[0] != (*it) || f.robot[0].productType != 0) && (task_toward[1] != (*it) || f.robot[1].productType != 0) && (task_toward[2] != (*it) || f.robot[2].productType != 0) && (task_toward[3] != (*it) || f.robot[3].productType != 0))
                    {
                        if ((*it) != n)
                            task_toward[robotId] = (*it);
                        return;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }
        }
        else if (f.stage[(*it)].type == 7 && ask_if_haveproduct(f, 7))
        {
            if (f.stage[(*it)].productStatus == 1)
            {
                if (ask_target_occupied(f, 7, task_toward))
                {
                    continue;
                }
                else
                {
                    if ((task_toward[0] != (*it) || f.robot[0].productType != 0) && (task_toward[1] != (*it) || f.robot[1].productType != 0) && (task_toward[2] != (*it) || f.robot[2].productType != 0) && (task_toward[3] != (*it) || f.robot[3].productType != 0))
                    {
                        if ((*it) != n)
                            task_toward[robotId] = (*it);
                        return;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }
        }
    }
    if (f.stage.size() == 43)
    {
        if (frameID == 1)
        {
            if (robotId == 3 || robotId == 2)
            {
                task_toward[robotId] = 42;
            }
            else if (robotId == 1)
            {
                task_toward[robotId] = 0;
            }
            else
            {
                task_toward[robotId] = 41;
            }
        }
        else
        {
            task_toward[robotId] = End;
        }
    }
}

void post_product(read_by_frame &f,int robotID,int *task_toward,int objtype)//优先级一(1,2,3->4,5,6)的送物指令
{
    for(auto it=f.stage_value_order.begin();it!=f.stage_value_order.end();it++)
    {
       if(objtype==1)
       {
            if(f.stage[(*it)].type==4||f.stage[(*it)].type==5)
            {
                string s;
                s=atob(f.stage[(*it)].materialStatus);
                {
                    if(s[objtype]=='0')
                    {
                        if((task_toward[0]!=(*it)||f.robot[0].productType!=objtype)&&(task_toward[1]!=(*it)||f.robot[1].productType!=objtype)&&(task_toward[2]!=(*it)||f.robot[2].productType!=objtype)&&(task_toward[3]!=(*it)||f.robot[3].productType!=objtype))
                        {
                            task_toward[robotID]=(*it);
                            return;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
       else if(objtype==2)
       {
            if(f.stage[(*it)].type==4||f.stage[(*it)].type==6)
            {
                string s;
                s=atob(f.stage[(*it)].materialStatus);
                {
                    if(s[objtype]=='0')
                    {
                        if((task_toward[0]!=(*it)||f.robot[0].productType!=objtype)&&(task_toward[1]!=(*it)||f.robot[1].productType!=objtype)&&(task_toward[2]!=(*it)||f.robot[2].productType!=objtype)&&(task_toward[3]!=(*it)||f.robot[3].productType!=objtype))
                        {
                            task_toward[robotID]=(*it);
                            return;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
       else if(objtype==3)
       {
            if(f.stage[(*it)].type==5||f.stage[(*it)].type==6)
            {
                string s;
                s=atob(f.stage[(*it)].materialStatus);
                {
                    if(s[objtype]=='0')
                    {
                        if((task_toward[0]!=(*it)||f.robot[0].productType!=objtype)&&(task_toward[1]!=(*it)||f.robot[1].productType!=objtype)&&(task_toward[2]!=(*it)||f.robot[2].productType!=objtype)&&(task_toward[3]!=(*it)||f.robot[3].productType!=objtype))
                        {
                            task_toward[robotID]=(*it);
                            return;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
        else if(objtype==4||objtype==5||objtype==6)
        {
            if(f.stage[(*it)].type==7||f.stage[(*it)].type==9)
            {
                string s;
                s=atob(f.stage[(*it)].materialStatus);
                {
                    if(s[objtype]=='0')
                    {
                        if((task_toward[0]!=(*it)||f.robot[0].productType!=objtype||f.stage[task_toward[0]].type==9)&&(task_toward[1]!=(*it)||f.robot[1].productType!=objtype||f.stage[task_toward[1]].type==9)&&(task_toward[2]!=(*it)||f.robot[2].productType!=objtype||f.stage[task_toward[2]].type==9)&&(task_toward[3]!=(*it)||f.robot[3].productType!=objtype||f.stage[task_toward[3]].type==9))
                        {
                            task_toward[robotID]=(*it);
                            return;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
        else if(objtype==7)
        {
            if(f.stage[(*it)].type==8||f.stage[(*it)].type==9)
            {
                if(task_toward[0]!=(*it)&&task_toward[1]!=(*it)&&task_toward[2]!=(*it)&&task_toward[3]!=(*it))
                {
                    task_toward[robotID]=(*it);
                    return;
                }
                else
                {
                    continue;
                }
            }
        }
    }   
}

void task_give(read_by_frame &f,int robotId,int *task_toward,int frameID, int n)//通过调用子函数实现任务分发，分发前问询取物工作台的状态以决定优先级
{
    int productId=f.robot[robotId].productType;
    if(productId==0)
    {
        get_product(f,robotId,task_toward, n);
    }
    else
    {
        post_product(f,robotId,task_toward,productId);
    }
}
struct cmp//重载set排序方式
{
    bool operator()(pair<int,double> a,pair<int,double> b)const
    {
        return a.second > b.second;
    }
};

void add_stage_value(read_by_frame *f, int type){
    for(int i = 0; i < stagingNum; i++){
        if(f->stage[i].type == type)
            f->stage[i].value += 100;
    }
}

bool judge_have_material(read_by_frame f, int type, int stageId){
    int j = f.stage[stageId].materialStatus;
    if((type == 1 && (j == 2 || j == 6 || j == 10)) || (type == 2 && (j == 4 || j == 6 || j == 12)) || (type == 3 && (j == 8 || j == 10 || j == 12) || 
       (type == 4 && (j == 16 || j == 48 || j == 80 || j == 112) || (type == 5 && (j == 32 || j == 48 || j == 96 || j == 112)) || 
       (type == 6 && (j == 64 || j == 80 || j == 96 || j == 112)) || (type == 7 && j == 128))))
        return false;
    else
        return true;
}

bool compare(pair<int, int> a, pair<int, int> b){
    return a.second > b.second;
}

bool compare2(pair<int, int> a, pair<int, int> b){
    return a.second < b.second;
}

void set_stage_value(read_by_frame *f){
    int exist[6] = {0}, need[6] = {0};
    for(int i = 0; i < stagingNum; i++){
        for(int j = 0; j < 6; j++){
            if(!judge_have_material(*f, j + 1, i))
                exist[j] += 1;
        }
    } //判断场上工作台已有商品类型个数
    for(int j = 0; j < 4; j++){
        if(f->robot[j].productType != 0)
            exist[f->robot[j].productType - 1]++;
    } //判断场上机器人携带商品类型个数
    for(int i = 0; i < stagingNum; i++){
        if(f->stage[i].type == 4){
            need[0]++, need[1]++;
            if(f->stage[i].remainTime != -1)
                exist[3]++;
            if(f->stage[i].productStatus == 1)
                exist[3]++;
        }
        else if(f->stage[i].type == 5){
            need[0]++, need[2]++;
            if(f->stage[i].remainTime != -1)
                exist[4]++;
            if(f->stage[i].productStatus == 1)
                exist[4]++;
        }
        else if(f->stage[i].type == 6){
            need[1]++, need[2]++;
            if(f->stage[i].remainTime != -1)
                exist[5]++;
            if(f->stage[i].productStatus == 1)
                exist[5]++;
        }
        else if(f->stage[i].type == 7)
            need[3]++, need[4]++, need[5]++;
    } //统计各类型商品总需求数和产品格内已存在数
    vector<pair<int, int>> v;
    for(int i = 0; i < 6; i++){
        int m = (need[i] - exist[i]) * 10 / (need[i] + 1);
        pair<int, int> p(i + 1, m);
        v.push_back(p);
    } //以需求差值/总需求数作为权值计算并统计，权值比例为25
    sort(v.begin(), v.end(), compare);
    for(int i = 0; i < 6; i++){
        v[i].second *= 25;
    }
    for(int i = 0; i < stagingNum; i++){
        for(int j = 1; j < 7; j++){
            if(f->stage[i].type == j){
                for(int k = 0; k < 6; k++){
                    if(v[k].first == j)
                        f->stage[i].value += v[k].second;
                }
            }
        }
    } //统计权值
    for(int i = 0; i < stagingNum; i++){
        if(f->stage[i].remainTime == -1)
            f->stage[i].value += 100;
    } //判断工作台是否生产，如果未生产，增加权值
    for(int i = 0; i < stagingNum; i++){
        if(f->stage[i].type == 4){
            if(f->stage[i].materialStatus == 2 || f->stage[i].materialStatus == 4){
                f->stage[i].value += 20;
            }
        }
        if(f->stage[i].type == 5){
            if(f->stage[i].materialStatus == 2 || f->stage[i].materialStatus == 8){
                f->stage[i].value += 20;
            }
        }
        if(f->stage[i].type == 6){
            if(f->stage[i].materialStatus == 4 || f->stage[i].materialStatus == 8){
                f->stage[i].value += 20;
            }
        }
        if(f->stage[i].type == 7){
            if(f->stage[i].materialStatus == 16 || f->stage[i].materialStatus == 32 || f->stage[i].materialStatus == 64){
                f->stage[i].value += 20;
            }
            else if(f->stage[i].materialStatus == 48 || f->stage[i].materialStatus == 80 || f->stage[i].materialStatus == 96){
                f->stage[i].value += 40;
            }
        }
    } //判断工作台材料格剩余量，根据后续所需材料数进行权值赋值，占比为20
}

void sort_by_distance(read_by_frame *f, int robotID)//将工作台ID以工作台到机器人的距离远近排名
{
    vector<pair<int,int>> stage_distance;
    vector<int> help;
    int v = 200;
    for(int i = 0; i < stagingNum; i++){
        if(path[f->robot[robotID].inStage][i].empty())
            continue;
        auto a = path[f->robot[robotID].inStage][i].end();
        a--;
        if((*a)->G == 0)
            a = path[f->robot[robotID].inStage][i].begin();
        pair<int, int> x;
        x.first = i, x.second = (*a)->G;
        stage_distance.push_back(x);
    }
    sort(stage_distance.begin(), stage_distance.end(), compare2);
    for(int i = 0; i < stage_distance.size(); i++){
        f->stage[stage_distance[i].first].value += v;
        v -= 20;
    }
    set_stage_value(f);
    vector<pair<int, int>> stage_value;
    for(int i = 0; i < stagingNum; i++){
        int v = f->stage[i].value;
        pair<int, int> s(i, v);
        stage_value.push_back(s);
    }
    sort(stage_value.begin(), stage_value.end(), compare);
    for(auto it=stage_value.begin();it!=stage_value.end();it++)
    {
        help.push_back((*it).first);
    }
    f->stage_value_order.clear();
    f->stage_value_order.assign(help.begin(),help.end());
    for(int i = 0; i < stagingNum; i++){
        f->stage[i].value = 0;
    }
}
