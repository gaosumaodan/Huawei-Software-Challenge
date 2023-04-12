#include <bits/stdc++.h>
using namespace std;

#include "dstar.h"
#include "initialize.h"
#include "func.h"

int main()
{
    initialize_map();
    while (scanf("%d", &frameID) != EOF)
    {
        read_by_frame read_frame;
        read_frame.readUntilOK();
        printf("%d\n",frameID);
        int num = 0;
        if(stagingNum == 9){
            num = 3;
        }
        for(int i = num; i < 4; i++){
            if(task_toward[i] != -1){
                if(read_frame.robot[i].productType==0){
                    if(read_frame.robot[i].inStage != task_toward[i])
                        move(read_frame, i, task_toward[i]);
                    else{
                        printf("buy %d\n", i);
                        task_toward[i]=-1;
                        if(!pathRobot[i].empty())
                            pathRobot[i].clear();
                        if(i == 0)
                            startXy0 = path[0][0].end();
                        else if(i == 1)
                            startXy1 = path[0][0].end();
                        else if(i == 2)
                            startXy2 = path[0][0].end();
                        else
                            startXy3 = path[0][0].end();
                    }
                }
                else{
                    if(read_frame.robot[i].inStage != task_toward[i])
                        move(read_frame, i, task_toward[i]);
                    else{
                        printf("sell %d\n", i);
                        task_toward[i]=-1;
                        if(i == 0)
                            startXy0 = path[0][0].end();
                        else if(i == 1)
                            startXy1 = path[0][0].end();
                        else if(i == 2)
                            startXy2 = path[0][0].end();
                        else
                            startXy3 = path[0][0].end();
                    }
                }
            }
            else{   
                sort_by_distance(&read_frame,i);
                task_give(read_frame,i,task_toward,frameID, -1); 
            }
        }
        cout << "OK" << endl;
    }
    return 0;
}