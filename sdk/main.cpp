#include <iostream>
#include<vector>
using namespace std;
struct  Ins
{
    pair<int,double> forward;
    pair<int,double> rotate;
    int  buy;
    int sell;
    int destroy;
    Ins(pair<int,double> _forward, pair<int,double> _rotate,int  _buy,int _sell,int _destroy):
    forward(_forward),rotate(_rotate),buy(_buy),sell(_sell),destroy(_destroy){

    }
};

struct Robot
{
    int id;
    int target_id;//正在赶往的工作台；
    int get_id;//手上拿着的货物编号
    double x;
    double y;
    Ins ins;
    Robot(int _id,int _target_id,int _get_id,double _x,double _y,Ins _ins):
    id(_id),target_id(_target_id),get_id(_get_id),x(_x),y(_y),ins(_ins){

    }
};

struct  Studio
{
    int id;
    int r_id;//正在赶来的机器人id
    double x;
    double y;
    int r_time;//剩余生产时间（帧数）
    int bitSatus;//原材料格状态
    int pStatus;//产品格状态 
    Studio(int _id,int _r_id,double _x,double _y,int _r_time,int _bitSatus,int _pStatus):
    id(_id),r_id(_r_id),x(_x),y(_y),r_time(_r_time),bitSatus(_bitSatus),pStatus(_pStatus)
    {

    }
};
vector<vector<double>>dis;
vector<Studio> studios;
vector<Robot> robots;

bool readUntilOK() {
    char line[1024];
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
    }
    return false;
}

int main() {
    readUntilOK();
    puts("OK");
    fflush(stdout);
    int frameID;
    while (scanf("%d", &frameID) != EOF) {
        readUntilOK();
        printf("%d\n", frameID);
        int lineSpeed = 3;
        double angleSpeed = 1.5;
        for(int robotId = 0; robotId < 4; robotId++){
            printf("forward %d %d\n", robotId, lineSpeed);
            printf("rotate %d %f\n", robotId, angleSpeed);
        }
        printf("OK\n", frameID);
        fflush(stdout);
    }
    return 0;
}
