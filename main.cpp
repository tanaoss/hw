#include <iostream>
#include <vector>
#include <cmath>
#include <sstream>
#include <string>
using namespace std;
struct  State
{
    int time; 
    int money;
};
struct  Ins
{
    int robID;
    double forward;
    double rotate;
    int  buy;
    int sell;
    int destroy;
    Ins(int _robID,double _forward, double _rotate,int  _buy=-1,int _sell=-1,int _destroy=-1):
    robID(_robID),forward(_forward),rotate(_rotate),buy(_buy),sell(_sell),destroy(_destroy){

    }
    friend ostream& operator<<(ostream& os,Ins ins){
        cout<<"forward "<<ins.robID<<" "<<ins.forward<<"\n"
        <<"rotate "<<ins.robID<<" "<<ins.rotate<<endl;
        if(ins.buy!=-1){
            cout<<"buy "<<ins.robID<<endl;
        }
        if(ins.sell!=-1){
            cout<<"sell "<<ins.robID<<endl;
        }
        if(ins.destroy!=-1){
            cout<<"destroy "<<ins.robID<<endl;
        }
     return os;   
    }
};

struct Robot
{
    int id;
    int get_type;//携带物品类型
    double time_val;
    double collision_val;
    double angular_velocity;
    pair<double,double>xy_pos;
    double direction;
    pair<double,double>pos;
    int target_id;//正在赶往的工作台；
    Robot(int _id,int _get_type,double _time_val,double _collision_val,double _angular_velocity,pair<double,double>& _xy_pos,
    double _direction,pair<double,double>&_pos,int _target_id=0):
    xy_pos(_xy_pos),pos(_pos)
    {
        id=_id;
        get_type=_get_type;
        time_val=_time_val;
        collision_val=_collision_val;
        angular_velocity=_angular_velocity;
        direction=_direction;
        target_id=_target_id;
    }
    void set(int _id,int _get_type,double _time_val,double _collision_val,double _angular_velocity,pair<double,double>& _xy_pos,
    double _direction,pair<double,double>&_pos,int _target_id=0)
    {
        id=_id;
        get_type=_get_type;
        time_val=_time_val;
        collision_val=_collision_val;
        angular_velocity=_angular_velocity;
        direction=_direction;
        target_id=_target_id;
        xy_pos=_xy_pos;
        pos=_pos;
    }
};

struct  Studio
{
    int id;
    int class_id;
    int r_id;//正在赶来的机器人id
    pair<double,double>pos;
    int r_time;//剩余生产时间（帧数）
    int bitSatus;//原材料格状态
    int pStatus;//产品格状态 
    Studio(int _id,int class_id,int _r_id, pair<double,double>&_pos,int _r_time,int _bitSatus,int _pStatus):
    id(_id),class_id(class_id),r_id(_r_id),pos(_pos),r_time(_r_time),bitSatus(_bitSatus),pStatus(_pStatus)
    {

    }
    void set(int _id,int class_id,int _r_id, pair<double,double>&_pos,int _r_time,int _bitSatus,int _pStatus){
        id=_id;
        class_id = class_id;
        r_id=_r_id;
        pos=_pos;
        r_time=_r_time;
        bitSatus=_bitSatus;
        pStatus=_pStatus;
    }
};
vector<vector<double>>dis(50,vector<double>(50,0));
vector<Studio> studios;
vector<Robot> robots;
State state;
double calcuDis(pair<double, double> a, pair<double, double> b) {
    return sqrt((a.first - b.first) * (a.first - b.first) + (a.second -b.second) * (a.second -b.second));
}

void calcuStudioDis() {
    int num = studios.size();
    int i,j;
    for(i = 0; i < num; i++) {
        for (j = 0; j < i; j++) {
            dis[j][i] = dis[i][j] = calcuDis(studios[i].pos, studios[j].pos);
        }
    }
}


bool readMapUntilOK() {
    char line[1024];
    int count = 0;
    int count_robot = 0, count_studio = 0;
    double x,y;
    int i;
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
        for(i=0;i<100;i++){
            if(line[i] == 'A'){
                x = i*0.5+0.25;
                y = (100-count)*0.5-0.25;
                pair<double,double>pos_robot(x,y);
                pair<double,double>xy_pos_robot(0,0);
                Robot robot = Robot(count_robot,0,0.0,0.0,0.0,xy_pos_robot,0.0,pos_robot);
                robots.push_back(robot);
                count_robot++;
            }
            else if(line[i] >= '1' && line[i] <= '9'){
                x = i*0.5+0.25;
                y = (100-count)*0.5-0.25;
                pair<double,double>pos_studio(x,y);
                Studio studio = Studio(count_studio,0,0,pos_studio,0,0,0);
                studio.class_id = (int)line[i]-48;
                studios.push_back(studio);
                count_studio++;
            }
        }

	    count++;
    }
    return false;
}
bool readStatusUntilOK() {
    char line[1024];
    int count = 0;
    int i;
    int start,end;
    int number;
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //dosomething
        stringstream sstream(line);
        if(count == 0){
            sstream >> state.time;
            sstream >> state.money;
        }
        if(count == 2){
            
        }
    }
    return false;
}

bool init(){
    return true;
}

int main() {
    readMapUntilOK();
    puts("OK");
    // readStatusUntilOK();
    // puts("OK");
    fflush(stdout);
    int frameID;
    while (scanf("%d", &frameID) != EOF) {
        readStatusUntilOK();
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
