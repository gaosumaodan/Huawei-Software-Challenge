#define robot_radius_empty 0.45
#define robot_radius_full 0.53
#define pi acos(-1)
struct staging // 定义工作台
{
    int type = 0, productStatus = 0, materialStatus = 0, remainTime = 0, value = 0;
    double x = 0, y = 0;
    bool operator<(const staging &stage) const
    {
        if (value != stage.value)
            return value > stage.value;
        return 0;
    }
};

struct Robot // 定义机器人
{
    int inStage = 0, productType = 0;
    double timeValue = 0, bangValue = 0, angelSpeed = 0, forward = 0, x = 0, y = 0, xLineSpeed = 0, yLineSpeed = 0;
};
vector<list<Point *>> Path[10][10];
int frameID, tempFrameID[4] = {0};
int money = 200000;
int stagingNum = 0;
int task_toward[4] = {-1, -1, -1, -1}; // robot的任务状态值
vector<vector<char>> Map;
list<Point *> path[50][50] = {};
list<Point *> pathCrash[4] = {};
auto CrashXy0 = path[0][0].end(), CrashXy1 = path[0][0].end(), CrashXy2 = path[0][0].end();
list<Point *> pathRobot[4] = {};
int task_start[4]={-1, -1, -1, -1};
int crash_cross[4] = {0, 0, 0, 0};
int crash_judge[4] = {0, 0, 0, 0};
auto startXy0 = pathRobot[0].begin(), crash0 = path[0][0].end();
auto startXy1 = pathRobot[1].begin(), crash1 = path[0][0].end();
auto startXy2 = pathRobot[2].begin(), crash2 = path[0][0].end();
auto startXy3 = pathRobot[3].begin(), crash3 = path[0][0].end();
auto tempPoint = path[0][0].end(), crashStart = path[0][0].end(), crashPoint = path[0][0].end();
class read_by_frame // 定义处理每一帧数据的函数
{
public:
    void readUntilOK()
    {
        cin >> money;
        cin >> stagingNum;
        robot.resize(4);
        stage.resize(stagingNum);
        stage_type.resize(10);
        for (int i = 0; i < stagingNum; i++)
        {
            cin >> stage[i].type >> stage[i].x >> stage[i].y >> stage[i].remainTime;
            cin >> stage[i].materialStatus >> stage[i].productStatus;
            if (stage[i].type == 1 || stage[i].type == 2 || stage[i].type == 3)
                stage[i].value = 0;
            else if (stage[i].type == 4 || stage[i].type == 5 || stage[i].type == 6)
                stage[i].value = 500;
            else if (stage[i].type == 7)
                stage[i].value = 1000;
            stage_value_order.push_back(i);
            stage_type[stage[i].type].push_back(i);
        }
        for (int i = 0; i < 4; i++)
        {
            cin >> robot[i].inStage >> robot[i].productType >> robot[i].timeValue;
            cin >> robot[i].bangValue >> robot[i].angelSpeed >> robot[i].xLineSpeed;
            cin >> robot[i].yLineSpeed >> robot[i].forward >> robot[i].x >> robot[i].y;
        }
        string s;
        cin >> s;
    }
    vector<Robot> robot;            // 存放机器人本体
    vector<staging> stage;          // 存放工作台本体
    vector<int> stage_value_order;  // 存放按工作台权值排名的工作台ID
    vector<vector<int>> stage_type; // 存放按工作台类型分类的工作台ID
};
// void get_zz(int row1, int col1, int row2, int col2, read_by_frame read_frame)
// {
//     for (int i = 0; i < Path[row1][col1].size(); i++)
//     {
//         list<Point *> temp;
//         for (int j = 0; j < Path[row2][col2].size(); j++)
//         {
//             auto begin1 = Path[row1][col1][i].begin();
//             auto begin2 = Path[row2][col2][j].begin();
//             int ttt = (*begin1)->end_stageId;
//             int tttt = (*begin2)->begin_stageId;
//             if (((*begin1)->end_stageId != (*begin2)->begin_stageId))
//             {
//                 continue;
//             }
//             else
//             {

//                 if (path[(*begin1)->begin_stageId][(*begin2)->end_stageId].empty())
//                 {
//                     temp = Path[row1][col1][i];
//                     auto itt = Path[row2][col2][j].begin();
//                     itt++;
//                     temp.splice(temp.end(), Path[row2][col2][j], itt, Path[row2][col2][j].end());
//                     auto begin = temp.begin();
//                     (*begin)->begin_stageId = (*begin1)->begin_stageId;
//                     (*begin)->end_stageId = (*begin2)->end_stageId;
//                     int t1 = (*begin)->begin_stageId;
//                     int t2 = (*begin)->end_stageId;
//                     path[t1][t2] = temp;
//                     path[t1][t2].reverse();
//                     path[t2][t1] = path[t1][t2];
//                     path[t1][t2].reverse();
//                     if (col2 == 7)
//                     {
//                         Path[row1][col2].push_back(temp);
//                         temp.reverse();
//                         auto beginn = temp.begin();
//                         (*beginn)->begin_stageId = (*begin2)->end_stageId;
//                         (*beginn)->end_stageId = (*begin1)->begin_stageId;
//                         Path[col2][row1].push_back(temp);
//                     }
//                 }
//                 else
//                 {
//                     auto end = path[(*begin1)->begin_stageId][(*begin2)->end_stageId].end();
//                     end--;
//                     auto end1 = Path[row1][col1][i].end();
//                     end1--;
//                     auto end2 = Path[row2][col2][j].end();
//                     end2--;
//                     int sum = 0;
//                     sum += (*end1)->G;
//                     sum += (*end2)->G;
//                     if (sum < (*end)->G)
//                     {
//                         temp = Path[row1][col1][i];
//                         auto itt = Path[row2][col2][j].begin();
//                         itt++;
//                         temp.splice(temp.end(), Path[row2][col2][j], itt, Path[row2][col2][j].end());
//                         auto begin = temp.begin();
//                         (*begin)->begin_stageId = (*begin1)->begin_stageId;
//                         (*begin)->end_stageId = (*begin2)->end_stageId;
//                         int t1 = (*begin)->begin_stageId;
//                         int t2 = (*begin)->end_stageId;
//                         path[t1][t2] = temp;
//                         path[t1][t2].reverse();
//                         path[t2][t1] = path[t1][t2];
//                         path[t1][t2].reverse();
//                         if (col2 == 7)
//                         {
//                             Path[row1][col2].push_back(temp);
//                             temp.reverse();
//                             auto beginn = temp.begin();
//                             (*beginn)->begin_stageId = (*begin2)->end_stageId;
//                             (*beginn)->end_stageId = (*begin1)->begin_stageId;
//                             Path[col2][row1].push_back(temp);
//                         }
//                     }
//                 }
//             }
//         }
//     }
// }

double length(read_by_frame f, int robotId, int stageId)
{
    double l = sqrt(pow(f.robot[robotId].x - f.stage[stageId].x, 2) + pow(f.robot[robotId].y - f.stage[stageId].y, 2));
    return l;
}

void initialize_map() // 初始化地图数据
{
    bool exist = 0;
    char line[101];
    double j = 0;
    read_by_frame read_frame;
    while (cin >> line)
    {
        if (line[0] != 'O' && line[1] != 'K')
        {
            vector<char> map1;
            for (int i = 0; i < 100; i++)
            {
                if (line[i] != '.' && line[i] != '#' && line[i] != 'A' && line[i] != '\n' && line[i] != 0)
                {
                    read_frame.stage.push_back({line[i] - 48, 0, 0, -1, 0, i * 0.5 + 0.25, 50 - (j * 0.5 + 0.25)});
                    if (line[i] - 48 == 7)
                    {
                        exist = 1;
                    }
                }
                if (line[i] == 'A')
                    read_frame.robot.push_back({-1, 0, 0, 0, 0, 0, i * 0.5 + 0.25, 50 - (j * 0.5 + 0.25), 0, 0});
                map1.push_back(line[i]);
            }
            Map.push_back(map1);
            j++;
            continue;
        }
        else
        {
            break;
        }
    }
    Astar astar;
    astar.Init(Map);
    // for (int i = 0; i < read_frame.stage.size(); i++)
    // {
    //     if (read_frame.stage[i].type == 1)
    //     {
    //         for (int j = 0; j < read_frame.stage.size(); j++)
    //         {
    //             if (read_frame.stage[j].type == 4 || read_frame.stage[j].type == 5)
    //             {
    //                 if (path[i][j].empty())
    //                 {
    //                     auto start = new Point(read_frame.stage[i].x, read_frame.stage[i].y);
    //                     auto end = new Point(read_frame.stage[j].x, read_frame.stage[j].y);
    //                     path[i][j] = astar.GetPath_1_5(*start, *end, false);
    //                     path[j][i] = astar.GetPath_1(*end, *start, false);
    //                     auto it = path[j][i].begin();
    //                     (*it)->begin_stageId = j;
    //                     (*it)->end_stageId = i;
    //                     Path[read_frame.stage[j].type][1].push_back(path[j][i]);
    //                     path[j][i].reverse();
    //                     auto ittt = path[j][i].begin();
    //                     (*ittt)->begin_stageId = i;
    //                     (*ittt)->end_stageId = j;
    //                     Path[1][read_frame.stage[j].type].push_back(path[j][i]);
    //                     path[j][i].reverse();
    //                 }
    //             }
    //         }
    //     }

    //     if (read_frame.stage[i].type == 2)
    //     {
    //         for (int j = 0; j < read_frame.stage.size(); j++)
    //         {
    //             if (read_frame.stage[j].type == 4 || read_frame.stage[j].type == 6)
    //             {
    //                 if (path[i][j].empty())
    //                 {
    //                     auto start = new Point(read_frame.stage[i].x, read_frame.stage[i].y);
    //                     auto end = new Point(read_frame.stage[j].x, read_frame.stage[j].y);
    //                     path[i][j] = astar.GetPath_1_5(*start, *end, false);
    //                     path[j][i] = astar.GetPath_1(*end, *start, false);
    //                     auto it = path[j][i].begin();
    //                     (*it)->begin_stageId = j;
    //                     (*it)->end_stageId = i;
                        
    //                     Path[read_frame.stage[j].type][2].push_back(path[j][i]);
    //                     path[j][i].reverse();
    //                     auto ittt = path[j][i].begin();
    //                     (*ittt)->begin_stageId = i;
    //                     (*ittt)->end_stageId = j;
    //                     Path[2][read_frame.stage[j].type].push_back(path[j][i]);
    //                     path[j][i].reverse();
    //                 }
    //             }
    //         }
    //     }
    //     if (read_frame.stage[i].type == 3)
    //     {
    //         for (int j = 0; j < read_frame.stage.size(); j++)
    //         {
    //             if (read_frame.stage[j].type == 5 || read_frame.stage[j].type == 6)
    //             {
    //                 if (path[i][j].empty())
    //                 {
    //                     auto start = new Point(read_frame.stage[i].x, read_frame.stage[i].y);
    //                     auto end = new Point(read_frame.stage[j].x, read_frame.stage[j].y);
    //                     path[i][j] = astar.GetPath_1_5(*start, *end, false);
    //                     path[j][i] = astar.GetPath_1(*end, *start, false);
    //                     auto it = path[j][i].begin();
    //                     (*it)->begin_stageId = j;
    //                     (*it)->end_stageId = i;
    //                     Path[read_frame.stage[j].type][3].push_back(path[j][i]);
    //                     path[j][i].reverse();
    //                     auto ittt = path[j][i].begin();
    //                     (*ittt)->begin_stageId = i;
    //                     (*ittt)->end_stageId = j;
    //                     Path[3][read_frame.stage[j].type].push_back(path[j][i]);
    //                     path[j][i].reverse();
    //                 }
    //             }
    //         }
    //     }
    //     if (read_frame.stage[i].type == 4 || read_frame.stage[i].type == 5 || read_frame.stage[i].type == 6)
    //     {
    //         for (int j = 0; j < read_frame.stage.size(); j++)
    //         {
    //             if (i != j)
    //             {
    //                 if (read_frame.stage[j].type == 4 || read_frame.stage[j].type == 5 || read_frame.stage[j].type == 6) // between4,5,6
    //                 {
    //                     if (path[i][j].empty())
    //                     {
    //                         auto start = new Point(read_frame.stage[i].x, read_frame.stage[i].y);
    //                     auto end = new Point(read_frame.stage[j].x, read_frame.stage[j].y);
    //                     path[i][j] = astar.GetPath_1_5(*start, *end, false);
    //                     path[j][i] = astar.GetPath_1(*end, *start, false);
                            
    //                     }
    //                 }
    //                 if (exist) // you 7
    //                 {
    //                     if (read_frame.stage[j].type == 7) // 4,5,6 ->7
    //                     {
    //                         if (path[i][j].empty())
    //                         {
    //                             auto start = new Point(read_frame.stage[i].x, read_frame.stage[i].y);
    //                     auto end = new Point(read_frame.stage[j].x, read_frame.stage[j].y);
    //                     path[i][j] = astar.GetPath_1_5(*start, *end, false);
    //                     path[j][i] = astar.GetPath_1(*end, *start, false);
    //                             auto it = path[j][i].begin();
    //                             (*it)->begin_stageId = j;
    //                             (*it)->end_stageId = i;
    //                             Path[7][read_frame.stage[i].type].push_back(path[j][i]);
    //                             path[j][i].reverse();
    //                             auto ittt = path[j][i].begin();
    //                             (*ittt)->begin_stageId = i;
    //                             (*ittt)->end_stageId = j;
    //                             Path[read_frame.stage[i].type][7].push_back(path[j][i]);
    //                             path[j][i].reverse();
    //                         }
    //                     }
    //                 }
    //                 else // mei 7 bi you 9
    //                 {
    //                     if (read_frame.stage[j].type == 9) // 4,5,6 ->9
    //                     {
    //                         if (path[i][j].empty())
    //                         {
    //                            auto start = new Point(read_frame.stage[i].x, read_frame.stage[i].y);
    //                     auto end = new Point(read_frame.stage[j].x, read_frame.stage[j].y);
    //                     path[i][j] = astar.GetPath_1_5(*start, *end, false);
    //                     path[j][i] = astar.GetPath_1(*end, *start, false);
    //                             auto it = path[j][i].begin();
    //                             (*it)->begin_stageId = j;
    //                             (*it)->end_stageId = i;
    //                             Path[9][read_frame.stage[i].type].push_back(path[j][i]);
    //                             path[j][i].reverse();
    //                             auto ittt = path[j][i].begin();
    //                             (*ittt)->begin_stageId = i;
    //                             (*ittt)->end_stageId = j;
    //                             Path[read_frame.stage[i].type][9].push_back(path[j][i]);
    //                             path[j][i].reverse();
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }
    //     if (read_frame.stage[i].type == 7)
    //     {
    //         for (int j = 0; j < read_frame.stage.size(); j++)
    //         {
    //             if (i != j)
    //             {
    //                 if (read_frame.stage[j].type == 7 || read_frame.stage[j].type == 8 || read_frame.stage[j].type == 9)
    //                 {
    //                     if (path[i][j].empty())
    //                     {
    //                         auto start = new Point(read_frame.stage[i].x, read_frame.stage[i].y);
    //                     auto end = new Point(read_frame.stage[j].x, read_frame.stage[j].y);
    //                     path[i][j] = astar.GetPath_1_5(*start, *end, false);
    //                     path[j][i] = astar.GetPath_1(*end, *start, false);
    //                         if (read_frame.stage[j].type == 8 || read_frame.stage[j].type == 9)
    //                         {
    //                             auto it = path[j][i].begin();
    //                             (*it)->begin_stageId = j;
    //                             (*it)->end_stageId = i;
    //                             Path[read_frame.stage[j].type][7].push_back(path[j][i]);
    //                             path[j][i].reverse();
    //                             auto ittt = path[j][i].begin();
                                
    //                             (*ittt)->begin_stageId = i;
    //                             (*ittt)->end_stageId = j;
    //                             Path[7][read_frame.stage[j].type].push_back(path[j][i]);
    //                             path[j][i].reverse();
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }
    // if (exist)
    // {
    //     if (!Path[1][4].empty() && !Path[4][7].empty())
    //     {
    //         get_zz(1, 4, 4, 7, read_frame);
    //     }
    //     if (!Path[1][5].empty() && !Path[5][7].empty())
    //     {
    //         get_zz(1, 5, 5, 7, read_frame);
    //     }
    //     if (!Path[2][4].empty() && !Path[4][7].empty())
    //     {
    //         get_zz(2, 4, 4, 7, read_frame);
    //     }
    //     if (!Path[2][6].empty() && !Path[6][7].empty())
    //     {
    //         get_zz(2, 6, 6, 7, read_frame);
    //     }
    //     if (!Path[3][5].empty() && !Path[5][7].empty())
    //     {
    //         get_zz(3, 5, 5, 7, read_frame);
    //     }
    //     if (!Path[3][6].empty() && !Path[6][7].empty())
    //     {
    //         get_zz(3, 6, 6, 7, read_frame);
    //     }
    // }

    // if (exist)
    // {
    //     for (int i = 8; i <= 9; i++)
    //     {
    //         for (int j = 1; j <= 6; j++)
    //         {
    //             if (!Path[j][7].empty() && !Path[7][i].empty())
    //             {
    //                 get_zz(j, 7, 7, i, read_frame);
    //             }
    //         }
    //     }
    // }
    // else
    // {
    //     if (!Path[1][4].empty() && !Path[4][9].empty())
    //     {
    //         get_zz(1, 4, 4, 9, read_frame);
    //     }
    //     if (!Path[1][5].empty() && !Path[5][9].empty())
    //     {
    //         get_zz(1, 5, 5, 9, read_frame);
    //     }
    //     if (!Path[2][4].empty() && !Path[4][9].empty())
    //     {
    //         get_zz(2, 4, 4, 9, read_frame);
    //     }
    //     if (!Path[2][6].empty() && !Path[6][9].empty())
    //     {
    //         get_zz(2, 6, 6, 9, read_frame);
    //     }
    //     if (!Path[3][5].empty() && !Path[5][9].empty())
    //     {
    //         get_zz(3, 5, 5, 9, read_frame);
    //     }
    //     if (!Path[3][6].empty() && !Path[6][9].empty())
    //     {
    //         get_zz(3, 6, 6, 9, read_frame);
    //     }
    // }
    for(int i = 0; i < read_frame.stage.size(); i++){
        for(int j = 0; j < read_frame.stage.size(); j++){
            if(i != j){
                if(path[i][j].empty()){
                    auto start = new Point(read_frame.stage[i].x, read_frame.stage[i].y);
                    auto end = new Point(read_frame.stage[j].x, read_frame.stage[j].y);
                    path[i][j] = astar.GetPath_1_5(*start, *end, false);
                    path[i][j].reverse();
                    path[j][i] = path[i][j];
                    path[i][j].reverse();
                }
            }
        }
    }
    int num = 0;
    if(read_frame.stage.size() == 9)
        num = 3;
    for (int i = num; i < 4; i++)
    {
        int temp = 0;
        double tempLength = 10000;
        for (int j = 0; j < read_frame.stage.size(); j++)
        {
            if (j != task_toward[0] && j != task_toward[1] && j != task_toward[2] && j != task_toward[3] &&
                (read_frame.stage[j].type == 1 || read_frame.stage[j].type == 2 || read_frame.stage[j].type == 3))
            {
                if (tempLength > length(read_frame, i, j))
                {
                    tempLength = length(read_frame, i, j);
                    temp = j;
                }
            }
        }
        task_toward[i] = temp;
    }
    if (task_toward[3] == 0 && read_frame.stage[0].type != 1 && read_frame.stage[0].type != 2 && read_frame.stage[0].type != 3)
    {
        int temp = task_toward[0];
        double tempLength = 10000;
        for (int j = 0; j < 3; j++)
        {
            if (tempLength > length(read_frame, 3, task_toward[j]))
            {
                tempLength = length(read_frame, 3, task_toward[j]);
                temp = task_toward[j];
            }
        }
        task_toward[3] = temp;
    }
    for (int i = num; i < 4; i++)
    {
        Point start(read_frame.robot[i].x, read_frame.robot[i].y);
        Point end(read_frame.stage[task_toward[i]].x, read_frame.stage[task_toward[i]].y);
        pathRobot[i] = astar.GetPath_1(start, end, false);
    }
    startXy0 = pathRobot[0].begin();
    startXy1 = pathRobot[1].begin();
    startXy2 = pathRobot[2].begin();
    startXy3 = pathRobot[3].begin();
    if (line[0] == 'O' && line[1] == 'K')
    {
        cout << "OK" << endl;
        return;
    }
}
