#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <algorithm>
#include <cstring>
#include <queue>
#include<unordered_map>
#include<iomanip>
#include<set>
#include"vec.h"
#include "class.h"

// #include "line.h"

using namespace std;
vector<Studio> studios;
vector<Robot> robots;
State state;//当前帧数，全局可见
vector<Ins> ins(4);
vector<int> material[4][8];
vector<int> product[8];
vector<int> full_product;
vector<int> studios_type[10];
vector<PayLoad> payloads;
vector<pane> panes;
int class_map;
int price[8][2];
double EPS=1e-7;
double acceleration_no;
double acceleration_has;
double angular_acceleration_no;
double angular_acceleration_has;
vector<bool> need_stop(4,false);
int robot_get_type[8];
int last_solution[4][4];
int studios_rid[50][8];
int studio_material[6][8];
int studio_level[5][2];
int material_send[8][3];
int RootFlag=-2;
int robot_last_last_state[4][2];
int robot_last_state[4][2];
int Flag_sumulate=0;
int last_count[50];
int lack_material[8];
int produce_product[8];
int priority[8]; 
int max_wait_time[4];
int robot_area[4];
int contr_print_flag=0;
int graph[100][100];
int sum_matrix[2][100][100];
int product_time[8];
// int graphs[2][100][100];
// int target_sequence[2][500][500];
// int wail[101][101];
// double dis_area[2][500][500];
// double studio_dis[2][50][50];
// double init_robot_dis[4][50];
double new_cllo_time = 0;
pair<double ,double> Root;
pair<double ,double> Collision_point;

vector<type_area>types[2];
vector<pair<double,double>>arri_Set;
double Compute_redundancy=0;
Ins ins_set[8];
unordered_map<int,vector<Graph_node>> graph_edge[2];//点id的边集
unordered_map<int,pair<double,double>> exist_id[2];//确定存在的id，便于建立边关系
unordered_map<int,int> exist_id_type[2];//确定存在的点的关系
unordered_map<int,int> stu_transID;//建立工作台id与转换后id的关系
unordered_map<int,int> rob_transID;//建立机器人id与转换后id的关系
unordered_map<int,bool> can_arrival[2];//判断任意两个小格之间是否可以直达;
int graph_trans[100][100];

vector<int> next_node[50][2];//next_node[studio_id][2][node_id]:node_id去往studio_id工作台的下一个点
vector<double> dis_to_studios[50][2];//dis_studios[studio_id][2][node_id]:node_id去往studio_id工作台的距离

int bar_sum[100][100][2];


void initrobotInfo() {

    
    double weightMin = 0.45 * 0.45 * Pi * 20;
    double weightMax = 0.53 * 0.53 * Pi * 20;
    double inertiaMin = weightMin * 0.45 * 0.45 *0.5;
    double inertiaMax = weightMax * 0.53 * 0.53 *0.5;

    acceleration_no = 250/ weightMin;
    acceleration_has = 250 / weightMax;

    angular_acceleration_no = 50 / inertiaMin;
    angular_acceleration_has = 50 /inertiaMax;

    memset(last_solution, -1, sizeof(last_solution));


    for(int i = 0; i < 6; ++i) {
        if(i < 3) ins_set[i].forward = 0;
        if(i % 3 == 0) ins_set[i].rotate = 0;
        else if(i % 3 == 1) ins_set[i].rotate = Pi;
        else ins_set[i].rotate = -Pi;
    }
    ins_set[6].forward = 0;
    ins_set[7].forward = -2;
}
void init_studio_parameter(){
    int studio_id;
    for(int i=0;i<50;i++){
        last_count[i]=0;
        for(int j=0;j<8;j++)studios_rid[i][j]=-1;
    }
    for(int i=0;i<studios.size();i++){
        studios_type[studios[i].type].push_back(i);
    }

    studio_material[0][0]=2;
    studio_material[0][1]=1;
    studio_material[0][2]=2;
    studio_material[1][0]=2;
    studio_material[1][1]=1;
    studio_material[1][2]=3;
    studio_material[2][0]=2;
    studio_material[2][1]=2;
    studio_material[2][2]=3;
    studio_material[3][0]=3;
    studio_material[3][1]=4;
    studio_material[3][2]=5;
    studio_material[3][3]=6;
    studio_material[4][0]=1;
    studio_material[4][1]=7;
    studio_material[5][0]=7;
    studio_material[5][1]=1;
    studio_material[5][2]=2;
    studio_material[5][3]=3;
    studio_material[5][4]=4;
    studio_material[5][5]=5;
    studio_material[5][6]=6;
    studio_material[5][7]=7;
    studio_level[2][0] = 1;
    studio_level[2][1] = 3;
    studio_level[3][0] = 4;
    studio_level[3][1] = 6;
    studio_level[4][0] = 7;
    studio_level[4][1] = 7;
    material_send[1][0] = 2;
    material_send[1][1] = 4;
    material_send[1][2] = 5;
    material_send[2][0] = 2;
    material_send[2][1] = 4;
    material_send[2][2] = 6;
    material_send[3][0] = 2;
    material_send[3][1] = 5;
    material_send[3][2] = 6;
    material_send[4][0] = 1;
    material_send[4][1] = 7;
    material_send[5][0] = 1;
    material_send[5][1] = 7;
    material_send[6][0] = 1;
    material_send[6][1] = 7;
    material_send[7][0] = 1;
    material_send[7][1] = 8;
    price[1][0]=3000;
    price[1][1]=6000;
    price[2][0]=4400;
    price[2][1]=7600;
    price[3][0]=5800;
    price[3][1]=9200;
    price[4][0]=15400;
    price[4][1]=22500;
    price[5][0]=17200;
    price[5][1]=25000;
    price[6][0]=19200;
    price[6][1]=27500;
    price[7][0]=76000;
    price[7][1]=105000;
    product_time[1] = 50;
    product_time[2] = 50;
    product_time[3] = 50;
    product_time[4] = 500;
    product_time[5] = 500;
    product_time[6] = 500;
    product_time[7] = 1000;
    for(int i=0;i<studios.size();i++){
        if(studios[i].type>3){
            // cerr<<"studio : "<<i<<"type = "<<studios[i].type<<endl;
            for(int j = 1;j<=studio_material[studios[i].type-4][0];j++){
                // cerr<<" material type = "<<studio_material[studios[i].type-4][j]<<endl;
                for(int k = 0;k<studios_type[studio_material[studios[i].type-4][j]].size();k++){
                    studio_id = studios_type[studio_material[studios[i].type-4][j]][k];
                    // cerr<<" studio_id "<<studio_id<<" dist = "<<dis_to_studios[studio_id][1][studios[i].node_id]<<endl;
                    if(!(eq(dis_to_studios[studio_id][1][studios[i].node_id],10000))){
                        // cerr<<"j = "<<j<<endl;
                        studios[i].material_studios[j-1].push_back(studio_id);
                        // cerr<<' '<<studio_id<<' ';
                    }
                }
                // cerr<<endl;
            }
        }
    }

}




bool readMapUntilOK() {
    char line[1024];
    int count = 0;
    int count_robot = 0, count_studio = 0;
    double x,y;
    int i;
    int row = 0;
    int num = 0;
    // for(int k = 0;k<101;k++){
    //     for(int j = 0; j < 101; j++){
    //         wail[k][j] = 0;
    //     }
    // }
    while (cin.getline(line,sizeof(line))) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
        for(i=0;i<100;i++){
            pane train;
            train.id = num;
            train.pos.first = i * 0.5 + 0.25;
            train.pos.second = (100 - count) * 0.5 - 0.25;
            train.type = -1;
            if(line[i] == 'A'){
                x = i*0.5+0.25;
                y = (100-count)*0.5-0.25;
                pair<double,double>pos_robot(x,y);
                pair<double,double>xy_pos_robot(0,0);
                // cout<<x<<" "<<y<<endl;
                Robot  robot(count_robot,0,0,0,1,1,xy_pos_robot,0,pos_robot,-1, (99-row)*100+i);
                rob_transID[(99-row)*100+i] = count_robot;
                stu_transID[(99-row)*100+i] = -1;
                robot.pane_id = train.id;
                robots.push_back(robot);
                count_robot++;
            }
            else if(line[i] >= '1' && line[i] <= '9'){
                x = i*0.5+0.25;
                y = (100-count)*0.5-0.25;
                pair<double,double>pos_studio(x,y);
                // cout<<x<<" "<<y<<endl;
                Studio studio(count_studio,0,-1,pos_studio,0,0,0, (99-row)*100+i);
                stu_transID[(99-row)*100+i] = count_studio;
                studio.type = (int)line[i]-48;
                studio.pane_id = train.id;
                studios.push_back(studio);
                count_studio++;
            }
            if (line[i] == '#')
            {
                graph[99-row][i] = -2;
                train.type = -2;
                // wail[99-row][i] = -2;
                // wail[99-row][i+1] = -2;
                // wail[99-(row+1)][i] = -2;
                // wail[99-(row+1)][i+1] = -2;
            }
            else
                graph[99-row][i] = 0;
            panes.push_back(train);
            num++;
        }
        row++;
	    count++;
    }
    return false;
}
bool readStatusUntilOK() {
    string line;
    cin>>state.money;
    cin.ignore();
    int K;
    int studio_id=0;
    int rob_id=0;
    cin>>K;
    cin.ignore();
    for(int i=0;i<4;i++){
        for(int j=0;j<8;j++){
            material[i][j].clear();
        }
    }
    for(int i=0;i<8;++i) {
        product[i].clear();
        produce_product[i]=0;
        lack_material[i]=0;
        priority[i]=0;
    }
    for(int i=0;i<4;i++){
        max_wait_time[i]=0;
    }
    while (K--)
    {
        vector<double> tmp(6,0);
        for(int i=0;i<tmp.size();i++){
            cin>>tmp[i];
        }
        studios[studio_id].set(studio_id,tmp[0],pair<double,double>(tmp[1],tmp[2]),tmp[3],tmp[4],tmp[5]);
        if(studios[studio_id].pStatus == 1 ){
            product[studios[studio_id].type].push_back(studio_id);
            if (studios[studio_id].type >= 4 && studios[studio_id].type<=6){
                produce_product[studios[studio_id].type]++;
            }
        }
        if (studios[studio_id].type >= 4 && studios[studio_id].type<=7){
            
            if (studios[studio_id].bitSatus != 0 ){
                for(int i = 1; i <= studio_material[studios[studio_id].type-4][0]; i++){
                    if ((studios[studio_id].bitSatus & (int)pow(2, studio_material[studios[studio_id].type-4][i]))==0){
                        lack_material[studio_material[studios[studio_id].type - 4][i]]++;
                    }
                }
            }
        }
        if (studios[studio_id].type > 3)
        {
            if(studios[studio_id].type < 8){
                
                    for(int i = 0;i < 4;i++){
                        if(studios[studio_id].type == i+4){
                            for(int j = 0;j<studio_material[i][0];j++){
                                if((studios[studio_id].bitSatus & (int)pow(2,studio_material[i][j+1])) == 0){
                                    // if(studios_rid[studio_id][studio_material[i][j+1]] == -1)material[studio_material[i][j+1]].push_back(studio_id);
                                    for(int k = 0;k<4;k++){
                                        if(!eq(dis_to_studios[studio_id][0][robots[k].node_id],10000))
                                            material[k][studio_material[i][j+1]].push_back(studio_id);
                                    }
                                }
                            }
                        }
                    }
            
            }
            if(studios[studio_id].type == 8){
                for(int k = 0;k<4;k++){
                    if(!eq(dis_to_studios[studio_id][0][robots[k].node_id],10000))
                        material[k][7].push_back(studio_id);
                }
                                    
                // material[7].push_back(studio_id);
            }
            if(studios[studio_id].type == 9){
                for(int h = 1;h <=7;h++){
                    for(int k = 0;k<4;k++){
                        if(!eq(dis_to_studios[studio_id][0][robots[k].node_id],10000))
                            material[k][h].push_back(studio_id);
                    }
                    // material[h].push_back(studio_id);
                }
            }
        }
        studio_id++;
    }
    for(int i=1;i<7;i++){
        if(produce_product[i]<lack_material[i]){
            priority[i] = lack_material[i]-produce_product[i];
        }
    }
    for(int i=0;i<4;i++){
        vector<double> tmp(10,0);
        for(int i=0;i<tmp.size();i++){
            cin>>tmp[i];
        }
        robots[rob_id].collision_val_pre=robots[rob_id].collision_val;
        robots[rob_id].set(rob_id,tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],pair<double,double>(tmp[5],tmp[6]),tmp[7],
        pair<double,double>(tmp[8],tmp[9]));
        robots[rob_id].node_id = trans_pos_to_nodeID(rob_id);
        robots[rob_id].radius = (robots[rob_id].get_type == 0? 0.45: 0.53);

        // if(gt(robots[rob_id].collision_val_pre, robots[rob_id].collision_val) && robots[rob_id].get_type != 0)
        //     cerr<<"time-collision:"<< state.FrameID <<"collision" <<rob_id<< endl<<endl;
        rob_id++;
    }
    cin>>line;
    if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
    return false;
}

void out_put(){
    for(auto tmp:ins){
        cout<<tmp;
    }
    cout<<"OK\n";
    cout.flush();
}

pair<double, double> subVector(pair<double, double> a, pair<double, double> b) {
    return make_pair(a.first - b.first, a.second - b.second);
}

pair<double, double> addVector(pair<double, double> a, pair<double, double> b) {
    return make_pair(a.first + b.first, a.second + b.second);
}

pair<double, double> calVectorProduct(pair<double, double> a, double x) {
    return make_pair(a.first * x, a.second * x);
}


double calVectorProduct(pair<double, double> a, pair<double, double> b) {
    return a.first * b.first + a.second * b.second;
}

double calVectorSize(pair<double, double> a) {
    return sqrt(a.first * a.first + a.second * a.second);
}

double calcuDis(pair<double, double> a, pair<double, double> b)
{
    return sqrt((a.first - b.first) * (a.first - b.first) + (a.second - b.second) * (a.second - b.second));
}





// void calcuStudioDis(){
//     int num = studios.size();
//     int i, j;
//     for (i = 0; i < num; i++)
//     {
//         for (j = 0; j < i; j++)
//         {
//             dis[j][i] = dis[i][j] = calcuDis(studios[i].pos, studios[j].pos);
//         }
//     }
// }

void print_matr(){
    int i = 0;
    int j;
    for(i = 1 ; i <= 7; i++){
    }
}

double calAngle(pair<double, double> a, pair<double, double> b) {
    return acos(calVectorProduct(a, b) / calVectorSize(a) / calVectorSize(b));
}

double calAngle(pair<double, double> a) {

    double angle = acos(a.first / calVectorSize(a));
    return lt(a.second, 0.0) ? 2 * Pi- angle: angle;
}


PayLoad calPayload(Robot robot, pair<double, double> virtual_pos) {
    
    //int target = rand() % ((int)studios.size());
    //robots[robotID].target_id = target;

    //cerr << robotID << target<<endl;

    // Robot robot = robots[robotID];
    // pair<double, double> virtual_pos = robots[robotID].virtual_pos;


    // cerr << robotID << "--"<< robot.target_id<<endl;
    

    double distance = calcuDis(robot.pos, virtual_pos);
    double angular_acceleration = robot.get_type == 0? angular_acceleration_no :angular_acceleration_has;
    double acceleration = robot.get_type == 0? acceleration_no: acceleration_has;
    double speed = calVectorSize(robot.xy_pos) * (ge(calVectorProduct(robot.xy_pos, transformVector(robot.direction)), 0.0)? 1: -1);

    if(robot.target_id == -1) {
        return PayLoad((robot.get_type == 0? 0.45: 0.53), 0, 0, 0, 0, speed, 0);
    }

    // 计算机器人与目标点构成的向量与x轴正方向夹角
    pair<double, double> robotToStudio = subVector(virtual_pos, robot.pos);
    double angle1 = calAngle(robotToStudio);

    double angle2 = ge(robot.direction, 0.0) ? robot.direction: 2 * Pi + robot.direction;
    // double angle2 = calAngle(robot.xy_pos);

    double angle = angle2 - angle1;

    
    // if(state.FrameID==7010&& robotID==2) {
    //     printPair(robot.xy_pos);
    //     cerr<<"payload-speed:"<<speed<<endl;
    // }
    int sign;

    if(ge(angle, 0) && lt(angle, Pi) || lt(angle, -Pi))
        sign = -1;
    else
        sign = 1;
    angle = fabs(angle);
    angle  = gt(angle, Pi)? 2 * Pi - angle: angle;


    // cerr<<"**"<< angle1<<"**dir:"<<robot.direction<<"**"<<angle2<<endl;
    // cerr<<"**"<< angle << "**"<<distance<<"**"<<sign<<endl;

    return PayLoad((robot.get_type == 0? 0.45: 0.53), angle, angular_acceleration, acceleration, distance, speed, sign);
}

bool eq(double a, double b) { return abs(a - b) < EPS; } // ==
bool gt(double a, double b) { return a - b > EPS; }      // >
bool lt(double a, double b) { return a - b < -EPS; }     // <
bool ge(double a, double b) { return a - b > -EPS; }     // >=
bool le(double a, double b) { return a - b < EPS; }      // <=

double getRobotRadius(int robot_id) {
    return robots[robot_id].get_type == 0? 0.45: 0.53;
}




double calNextTimeDistance(double speed, double time, double  acceleration) {
    double speed_max = min(speed + time * acceleration, 6.0);
    double time_rest = max(time - (speed_max - speed) / acceleration, 0.0);
    return (speed_max * speed_max - speed * speed) / 2 / acceleration + speed_max * time_rest;
}

// bool checkTimeEnough(int robot_id, int target_id, int frame) {
//     double dis = distance(robot_id, target_id).second;
//     double time = frame * 0.02;//剩余秒数
//     double speed = calVectorSize(robots[robot_id].xy_pos);
//     double acceleration = robots[robot_id].get_type == 0? acceleration_no: acceleration_has;
//     // if((state.FrameID > 8500 )){
//     //     cerr<<"FrameID "<<state.FrameID<<endl;
//     //     cerr<<robot_id<<"-"<<target_id<<endl;
//     //     cerr<<"dis:"<<dis<<" speed:"<<speed<<endl;
//     //     cerr<<calNextTimeDistance(speed, time, acceleration)<<endl;
//     // }
//     if(lt(calNextTimeDistance(speed, time, acceleration), dis+3))
//         return false;
        
//     return true;
// }

bool checkEnough(int robot_id, int target_id, int frame)
{
    double dis = dis_to_studios[target_id][1][robots[robot_id].node_id]+0.4;
    if (dis > 0)
    {
        double time = dis/6/0.02; // 剩余秒数
        // cerr<<"time = "<<time<<" least time = "<<frame<<endl;
        if (time > ((frame)+2))
            return true;
        else
            return false;
    }
    return false;
}

pair<double, double> transformVector(double direction) {
    direction = gt(direction, 0)? direction: Pi * 2 + direction;
    return make_pair(cos(direction), sin(direction));
}

pair<double, double> getNextSpeed(double direction, pair<double, double> speed)
{
    direction = gt(direction, 0) ? direction : 2 * Pi + direction;
    double val = calVectorSize(speed);
    return make_pair(cos(direction) * val, sin(direction) * val);
}

pair<double, double> getNextPos(int robot_id)
{
    return addVector(robots[robot_id].pos, calVectorProduct(robots[robot_id].xy_pos, 0.02));
}

pair<double, double> getNextPos(pair<double, double> pos, pair<double, double> speed)
{
    return addVector(pos, calVectorProduct(speed, 0.02));
}

pair<double, double> getNextTimePos(int time, pair<double, double> pos, pair<double, double> speed, double direction, double angular_velocity)
{
    while (time--)
    {
        pos = getNextPos(pos, speed);
        speed = getNextSpeed(direction, speed);
        direction = direction + angular_velocity * 0.02;
    }
    return pos;
}


int getAvoidDirection(int goID, int stopID)
{
    double angle1 = calAngle(subVector(robots[stopID].pos, robots[goID].pos));
    double angle2 = ge(robots[goID].direction, 0.0) ? robots[goID].direction : 2 * Pi + robots[goID].direction;
    double angle3 = angle2 - angle1;

    double included_angle = fabs(angle3);
    included_angle = gt(included_angle, Pi) ? 2 * Pi - included_angle : included_angle;
    int sign;
        
    // 如果stopID-goID方向与goTD前进方向是锐角，go旋转
    if (gt(fabs(included_angle), Pi / 2))
        sign = 0;
    else if (eq(angle3, 0) || eq(angle3, Pi) || eq(angle3, -Pi))
        sign = gt(robots[goID].angular_velocity, 0)? 1: -1;
    else if (gt(angle3, 0) && lt(angle3, Pi) || lt(angle3, -Pi))
        sign = 1;
    else
        sign = -1;
        
        // if(lt(included_angle, robots[goID].angular_velocity * 0.05)) {
        //     ins[goID].rotate = gt(robots[goID].angular_velocity, 0)? Pi: -Pi;
        // }
        // if (lt(robots[goID].angular_velocity * sign, 0) && ge(fabs(robots[goID].angular_velocity) * 0.01, included_angle))
        //     sign = -sign;

        // if (state.FrameID == 4093){
        //     cerr << "included_angle" << included_angle << "sign:" << sign << endl;
        //     cerr<<sign<<"*"<<robots[goID].angular_velocity<<endl;
        // }
        
    return sign;
}

bool isAcuteAngle(pair<double, double> a, pair<double, double> b)
{
    return gt(calVectorProduct(a, b), 0);
}

bool isAcuteAngle(pair<double, double> a, double x)
{
    x = gt(x, 0) ? x: 2 * Pi + x;
    return gt(calVectorProduct(a, make_pair(cos(x), sin(x))), 0);
}

void printPair(pair<double,double> a) {
    cerr<<"pos:("<<a.first<<", "<<a.second<<")"<<endl;
}


bool isNearWall(int id) {
    int i=robots[id].pos.first;
    int j=robots[id].pos.second;
    if(i-1<=0||j-2<=0||i+2>=50||j+2>=50)
        return true;
    return false;
}




void updateLastRate()
{
    for (int i = 0; i < 4; ++i)
    {
        robots[i].lastRate = ins[i].rotate;
    }
}

double calc_priority(int studio_id){
    double priority_value = 1;
    // if(class_map == 3) {
    //     if(studios[studio_id].type == 5 || studios[studio_id].type == 6) {
    //         priority_value = 0.1;
    //     }
    // }
    if (class_map == 4)
    {

        if (studios[studio_id].type == 1 )
        {
            priority_value = 0.8;
        }
        else if (studios[studio_id].type == 2){
            priority_value = 0.9;
        }
        else
            {
                if (studios[studio_id].type > 3)
                {
                    if (priority[studios[studio_id].type] > 0)
                    {

                        priority_value -= (priority[studios[studio_id].type]) * 0.2;
                        if (priority_value < 0.5)
                            priority_value = 0.5;
                    }
                }
                // else
                // {
                //     priority_value -= (priority[studios[studio_id].type]) * 0.1;
                //     if (priority_value < 0.5)
                //         priority_value = 0.5;
                // }
            }
        
    }
    else if(class_map != 3){
        if(priority[studios[studio_id].type]>0){
            if(studios[studio_id].type>3){
                if(class_map == 2){
                    priority_value -= (priority[studios[studio_id].type]) * 0.2;
                }
                else priority_value -= (priority[studios[studio_id].type]) * 0.3;
                if (priority_value < 0.5)
                    priority_value=0.5;
            }
            else{
                priority_value -= (priority[studios[studio_id].type]) * 0.1;
                if (priority_value < 0.5)
                    priority_value = 0.5;
            }
        }
    }
    // if (class_map == 3)
    //     priority_value = 1;
     return priority_value;
}

void control(){
    contr_print_flag=1;
    payloads.clear();
    for(int i=0;i<4;i++){
        auto tins=contr_one_rob(robots[i]);
        ins[i].forward=tins.forward;
        ins[i].rotate=tins.rotate;
        ins[i].robID=i;
        payloads.emplace_back(calPayload(robots[i], robots[i].virtual_pos));
    }
    
    // if(state.FrameID>=1003&&state.FrameID<=1023){
    //     cerr<<state.FrameID<<" "<< ins[2].forward<<" "<<payLoad[2].distance<<" "<<payLoad[2].distance*sin(payLoad[2].angle) << endl;
    //     // cerr<<state.FrameID<<" "<<stop_dis<<" "<<payload.speed<<" "<<can_st<<" "<<(sin(cmpAngle)*payload.distance)<<" "<<robot.need_rote_wall<<endl;
    //     // cerr<<robot.pos.first<<"-"<<robot.pos.second<<endl;
    // }
     contr_print_flag=0;
    for(int i=0;i<4;i++){
        if(ins[i].forward==-1||ins[i].rotate==-1){
            cerr<<"io err"<<endl;
        }
    }
    //control
    // if(state.FrameID==1){
    //     cerr<<"------------------------------------"<<endl;
    //     auto tmp=Calculate_the_trajectory(robots[0],0,100);
    //     auto iter=tmp.rbegin();
    //     int pos=0;
        
    //     cerr<<tmp.size()<<endl;
    //     for(iter;iter!=tmp.rend();iter++){
    //         cerr<<state.FrameID+pos<<": "<<iter->first<<"-"<<iter->second<<" ";pos++;
    //     }
        
    //     cerr<<endl;
    //     cerr<<"------------------------------------"<<endl;
    // }

    // Collision_detection(payLoad);

    // if(state.FrameID >= 5600 && state.FrameID < 5610) {
    //     cerr<<state.FrameID<<endl;
    //     // cerr<<payloads[2].angle<<payloads[2].sign<<endl;
    //     cerr<<"ins:"<<ins[2].forward<<"  "<<ins[2].rotate<<endl;
    // }
    // if(state.FrameID>=854&&state.FrameID<=858){
    //     cerr<<state.FrameID<<" ins befoer "<<ins[0].forward<<endl;
    //     cerr<<check_will_colloWithWall(robots[0])<<endl;
    // }
    // collision_solve(25);

    // if(state.FrameID >= 5600 && state.FrameID < 5610) {
    //     cerr<<"~ins:"<<ins[2].forward<<"  "<<ins[2].rotate<<endl;
    // }

    //  if(state.FrameID >= 212 && state.FrameID < 267) {
    //     cerr<<"~ins:"<<ins[3].forward<<"  "<<ins[3].rotate<<endl;
    // }
  
    // if(state.FrameID >= 720 && state.FrameID <= 730)
    //     cerr<<"hello"<< robots[2].target_id<<endl;

    // if(state.FrameID >= 2760 && state.FrameID < 2780) {
    //     cerr<<state.FrameID<<endl;
    //     cerr<<"ins:"<<ins[0].forward<<"  "<<ins[0].rotate<<endl;
    //     cerr<<"tar_dis: "<<payLoad[2].distance<<endl;
    // }
    // for(int i=3;i<=3;i++){
    //     if(state.FrameID==1800){
    //         cerr<<"------------------"<<i<<"------------------"<<endl;
    //         Calculate_the_trajectory(robots[i],0,25);
    //         cerr<<"------------------"<<i<<"------------------"<<endl;
    //     }        
    // }

    // if(state.FrameID==149){
    //     cerr<<"------------------------------------"<<endl;
    //     Calculate_the_trajectory(robots[0],0,20,0);
    //     cerr<<"------------------------------------"<<endl;
    // }
    // if(state.FrameID==5490){
    //     cerr<<"------------------------------------"<<endl;
    //     Calculate_the_trajectory(robots[0],0,20,0);
    //     cerr<<"------------------------------------"<<endl;
    // }

    // if(state.FrameID==2962){
    //     cerr<<ins[2].
    // }
    
    // if(state.FrameID>=1800&&state.FrameID<=1825)
    // {
    //     for(int i=3;i<=3;i++){
    //     cerr<<" && "<<state.FrameID<<":real_wv  "
    //     <<robots[i].angular_velocity<<" real_dire: "<<robots[i].direction<<" real_pos "<<robots[i].pos.first<<"-"<<robots[i].pos.second
    //     <<" real_v_xy "<<robots[i].xy_pos.first<<"-"<<robots[i].xy_pos.second<<endl ;
    //     cerr<< payloads[i].speed<<endl;       
    //     }

    // }

    // if(state.FrameID == 2940) {
    //     for(int i = 0;i<4;++i)
    //         trajectory[i]=Calculate_the_trajectory(robots[i], 0, 25);
    // }

    // if(state.FrameID >= 2940 && state.FrameID < 2940+25) {
    //     cerr<<endl<<state.FrameID<<endl;
    //     for(int j=0;j<4;++j){
    //         cerr<<robots[j].id<<":"<<endl<<robots[j].pos.first<<","<<robots[j].pos.second<<endl;
    //         cerr<<"predict"<<endl;
    //         cerr<<trajectory[j][state.FrameID -2940].first<<","<<trajectory[j][state.FrameID -2940].second<<endl;
    //     }
    // }
    // if(state.FrameID>=4855&&state.FrameID<=4900){
    //     cerr<<state.FrameID<<" ins after "<<ins[1].forward<<endl;
    // }

    // for(int i =0;i<4;++i){
    //     if(lt(payloads[i].speed, 3)) {
    //         cerr<<state.FrameID<<":"<<i<<"speed:"<<payloads[i].speed<<endl;
    //     }
    // }

    updateLastRate();

    out_put();
}
double get_at_stop_test(double t,double a,double v,int sign_v1){
    double lef_time=0;
    double s=0;
    a*=sign_v1;
    if(gt(a*sign_v1,0))a*=sign_v1;
    if(lt(fabs(v),Pi)){
        double tmpTime=(0-v)/(a);
        double realTime=min(tmpTime,t);
        s=v*realTime+0.5*a*realTime*realTime;
        double res=(s);
        if(le(t,tmpTime)){
            if(gt(res*sign_v1,0)){
                return fabs(res);
            }
            else{
                return -1*fabs(res);
            }
            return (s);
        }
        t=t-realTime;
    }
    double res=(s+sign_v1*Pi*t);
    if(gt(res*sign_v1,0)){
        return fabs(res);
    }else{
        return -1*fabs(res);
    }
    return (s+sign_v1*Pi*t);
}

// double anger_to_length(int robot_id,int studio_id){
//     double length;
//     double anger = calPayload(robots[robot_id], robots[robot_id].virtual_pos).angle;
//     length = anger/Pi*6;
//     // cerr<<"length = "<<length<<endl;
//     return length;
// }

// double close_threshold(int robot_id,int target_id,int close_threshold){
//     int count = 0;
//     int dist;
//     for(int i=0;i<4;i++){
//         if(i!=robot_id){
//             if(robots[i].target_id != -1){
//                 dist = calcuDis(studios[robots[i].target_id].pos,studios[i].pos);
//                 if(dist < close_threshold) count++;
//             }
//         }
//     }
//     return 1+count*0.5;
// }

// double close_threshold2(int robot_id,int target_id,int close_threshold){
//     int count = 0;
//     double dist,k,b,y1,y2;
//     k= (studios[target_id].pos.second-robots[robot_id].pos.second)/((studios[target_id].pos.first-robots[robot_id].pos.first));
//     b= robots[robot_id].pos.second-k*robots[robot_id].pos.first;
//     //cerr<<" studios[target_id].pos.second-robots[robot_id].pos.second = "<<studios[target_id].pos.second-robots[robot_id].pos.second<<endl;
//     for(int i=0;i<4;i++){
//         if(i!=robot_id){
//             if(robots[i].target_id != -1){
//                 if((studios[target_id].pos.second-robots[robot_id].pos.second)>0){
//                     if(studios[robots[i].target_id].pos.second<studios[target_id].pos.second && robots[i].pos.second>robots[robot_id].pos.second){
//                         y1=k*(studios[robots[i].target_id].pos.first-close_threshold)+b;
//                         y2=k*(studios[robots[i].target_id].pos.first+close_threshold)+b;
//                         if(studios[robots[i].target_id].pos.second<=y2 && studios[robots[i].target_id].pos.second>=y1){
//                             //cerr<<" robots[i].direction-robots[robot_id].direction- = "<<robots[i].direction-robots[robot_id].direction<<endl;
//                             if(((robots[i].direction-robots[robot_id].direction)>(Pi*0.6))&&((robots[i].direction-robots[robot_id].direction)<(Pi*1.4))){
//                                 count++;
//                             }
//                         }
//                     }
//                 }
//             }
//             else{ 
//                 if(studios[robots[i].target_id].pos.second>studios[target_id].pos.second && robots[i].pos.second<robots[robot_id].pos.second){
//                     y1=k*(studios[robots[i].target_id].pos.first-close_threshold)+b;
//                     y2=k*(studios[robots[i].target_id].pos.first+close_threshold)+b;
//                     if(studios[robots[i].target_id].pos.second<=y2 && studios[robots[i].target_id].pos.second>=y1){
//                         //cerr<<" robots[i].direction-robots[robot_id].direction = "<<robots[i].direction-robots[robot_id].direction<<endl;
//                         if(((robots[i].direction-robots[robot_id].direction<(-Pi*0.6))&&((robots[i].direction-robots[robot_id].direction)>(-Pi*1.4)))){
//                             count++;
//                         }
//                     }
//                 }
//             }
//         }
//     }
//     //cerr<<"robot "<<robot_id<<" count = "<<count<<" studio_id = "<<target_id<<endl;
//     return 1+count*2;
// }
bool check_no_send(int studio_id){
    int i;
    for(i = 1;i<= studio_material[studios[studio_id].type-4][0];i++){
        if(studios_rid[studio_id][i]!=-1)break;
    }
    if(i!=studio_material[studios[studio_id].type-4][0]+1)return false;
    return true;
}
// bool check_send_dis(int studio_id ,double dist){
//     int i;
//     double dis;
//     for(i = 1;i<= studio_material[studios[studio_id].type-4][0];i++){
//         if(studios_rid[studio_id][i]!=-1){
//             dis=calcuDis(robots[studios_rid[studio_id][i]].pos,studios[studio_id].pos);
//             if(dis<dist)return true;
//         }
//     }
//     return false;
// }
// double wait_dis(int robot_id ,int studio_id){
//     double dis=0;
//     double dist_time = distance(robot_id,studio_id).first;
//     double dist = distance(robot_id,studio_id).second;
//     if(studios[studio_id].type>=4 && (!check_no_send(studio_id))&& (studios[studio_id].pStatus!=1)) return 100; 
//     if((studios[studio_id].pStatus==1||checkEnough(robot_id,studio_id,studios[studio_id].r_time))){
//         // cerr << "robot " << robot_id << " studio " << studio_id << " check_enough or studios[studio_id].pStatus==1" << checkEnough(robot_id, studio_id, studios[studio_id].r_time) << ' ' << studios[studio_id].pStatus << endl;
//         if (!check_send_dis(studio_id, dist))
//         {
//             return 0;
//         }
//         else return 1000;
//     }
//     else{
//         // cerr<<" studios[studio_id].r_time = "<<studios[studio_id].r_time<<" (dist/6.0/0.02) "<<(dist/6.0/0.02)<<endl;
//         if(class_map == 1 || class_map == 3)dis = 1000;
//         else{
//             dis = studios[studio_id].r_time*0.02-dist_time;
//             // cerr<<" dis = "<<dis<<endl;
//             if(class_map == 4){
//                 if(studios[studio_id].type==7){
//                     if(dis>3)return 1000;
//                 }
//                 else if (dis > 1)
//                     return 1000;
//             }
//             else{
//                 if(dis>1)return 1000; 
//             }
//         }
//         // cerr<<" wait dis = "<<dis<<endl;
//     }
//     return dis;  
// }
/*
  control target_id
*/
double check_root(){
    if(Root.first<0 &&Root.second<0 )return -1;
    if(Root.first>0 && Root.second>0){
        if(Root.first>Root.second)return Root.second;
        else return Root.first;
    }
    if(Root.first>Root.second)return Root.first;
    return Root.second;
}
double Calc_collisions_dis(int robot_id,int studio_id){
    int i,count = 0,target;
    double time;
    double dis = 0;
    pair<double,double> line_speed; 
    double dist = calcuDis(robots[robot_id].pos,studios[studio_id].pos);
    line_speed.first = robots[robot_id].xy_pos.first;
    line_speed.second = robots[robot_id].xy_pos.second;
    //cerr<<"ddd"<<robots[robot_id].xy_pos.first<<' '<<robots[robot_id].xy_pos.second<<' '<<robots[robot_id].target_id<<' '<<robots[robot_id].pos.first<<' '<<robots[robot_id].pos.second;
    robots[robot_id].xy_pos.first = (6/dist)*(studios[studio_id].pos.first-robots[robot_id].pos.first);
    robots[robot_id].xy_pos.second = (6/dist)*(studios[studio_id].pos.second-robots[robot_id].pos.second);
    target = robots[robot_id].target_id;
    robots[robot_id].target_id = studio_id;
    for(i = 0;i<4;i++){
        if(i!=robot_id&&(robots[i].target_id != -1)){
            vector<Robot>tmpS=robots;
            will_collision(robot_id,i);
            for(int i=0;i<tmpS.size();i++){
                if(tmpS[i]!=robots[i]){
                    // cerr<<"--+--"<<" "<<i<<endl;
                }
            }
            // time = check_root();
            // cerr<<" time = "<<time;
            // time = 1;
            if(time>0){
                if(studios[studio_id].pos.first>(robots[robot_id].pos.first+robots[robot_id].xy_pos.first*time)){
                    if(studios[studio_id].pos.second>(robots[robot_id].pos.second+robots[robot_id].xy_pos.second*time))count++;
                }
            } 
        }
    }
    // cerr<<" count = "<<count;
    if(count>=2){
        dis = 0.53*2*Pi*(1.3+(count-2)*0.3);
    }
    robots[robot_id].xy_pos.first = line_speed.first;
    robots[robot_id].xy_pos.second = line_speed.second;
    robots[robot_id].target_id = target;

    return dis;
}
// double back_dis(int robot_id,int studio_id){
//     int i;
//     double min = 10000;
//     double dist;
//     int min_subscript = -1;
//     for(i=0;i<material[robot_id][studios[studio_id].type].size();i++){
//         dist=dis_to_studios[i][1][studios[studio_id].node_id];
//         if(eq(dist,10000))continue;
//         if(dist<min){
//             min=dist;
//             min_subscript = material[robot_id][studios[studio_id].type][i];
//         }
//     }
//     return min;
// }


// double get_lack(int studio_id){
//     double lack = 1;
//     // if ((class_map == 4)){
//     //     if (studios[studio_id].type == 4)
//     //     {
//     //         lack = 0.1;
//     //     }
//     // }
//     return lack;
// }

// double precise_distance(int robot_id,int studio_id){
//     double r = 6/Pi;
//     double temp;
//     pair<double,double> center1((robots[robot_id].pos.first+cos(robots[robot_id].direction+Pi/2)*r),(robots[robot_id].pos.second+sin(robots[robot_id].direction+Pi/2)*r));
//     pair<double,double> center2((robots[robot_id].pos.first+cos(robots[robot_id].direction-Pi/2)*r),(robots[robot_id].pos.second+sin(robots[robot_id].direction-Pi/2)*r));
//     // cerr<<(robots[robot_id].pos.first+cos(robots[robot_id].direction+Pi/2)*r)<<endl;
//     // cerr<<(robots[robot_id].pos.first+cos(robots[robot_id].direction-Pi/2)*r)<<endl;
//     // cerr<<robots[robot_id].direction<<endl;
//     pair<double,double> center;
//     double dist1 = calcuDis(center1,studios[studio_id].pos);
//     double dist2 = calcuDis(center2,studios[studio_id].pos);
//     double center_studio_dis;
//     double robot_studio_dis = calcuDis(robots[robot_id].pos,studios[studio_id].pos);
//     if(dist1 <dist2 ){
//         center_studio_dis = dist1;
//         center = center1;
//     }
//     else{
//         center_studio_dis = dist2;
//         center = center2;
//     }
//     temp = (r*r+center_studio_dis*center_studio_dis-robot_studio_dis*robot_studio_dis)/(2*r*center_studio_dis);
//     if((temp)>1){
//         temp = 1;
//     }
//     if((temp)<-1){
//         temp = -1;
//     }
//     double angle_robot_center_studio = acos(temp);
//     temp = r/center_studio_dis;
//     if((temp)>1){
//         temp = 1;
//     }
//     if((temp)<-1){
//         temp = -1;
//     }
//     double studio_point_tangency = acos(temp);
//     double robot_studio_angle = calPayload(robots[robot_id], robots[robot_id].virtual_pos).angle;
//     double rounded_corner;
//     if(robot_studio_angle<(Pi/2)){
//         rounded_corner = angle_robot_center_studio - studio_point_tangency;
//     }
//     else{
//         rounded_corner = 2*Pi-studio_point_tangency-angle_robot_center_studio;
//     }
//     double arc_length = rounded_corner *r;
//     double length = center_studio_dis*sin(studio_point_tangency)+arc_length;
//     // cerr<<"start"<<endl;
//     // cerr<< robots[robot_id].pos.first<<' '<<robots[robot_id].pos.second<<' '<<center.first<<' '<<center.second<<' '<<studios[studio_id].pos.first<<' '<<studios[studio_id].pos.second<<' '<<center_studio_dis<<endl;
//     // cerr<< angle_robot_center_studio<<' '<<studio_point_tangency<<' '<<robot_studio_angle<<' '<<rounded_corner<<' '<<arc_length<<' '<<length<<endl;
//     // cerr<<"end"<<endl;
//     return length;
// }
// double target_obstacle_avoidance(int robot_id,int studio_id){
//     int i;
//     double dist1,dist2;
//     double count = 0;
//     double time1 = distance(robot_id,studio_id).first;
//     for(i=0;i<4;i++){
//         if(robot_id != i && robot_id != -1){
//             if(robots[i].target_id == studio_id){
//                 double time2 = distance(i,robots[i].target_id).first;
//                 if(fabs(time1-time2)<0.8){
//                     count+=1.5;
//                 }
//             }
//         }
//     }
//     return count;
// }
// pair<double,double> distance(int  robot_id,int studio_id){
//     double dist = 1000000;
//     double time;
//     pair<double,double> inflection;
//     // auto tmp=Calculate_the_trajectory(robots[robot_id],0,10);
//     // inflection.first = tmp[0].first;
//     // inflection.second = tmp[0].second;
//     if(state.FrameID<-5) {
//          int target = robots[robot_id].target_id;
//         robots[robot_id].target_id = studio_id;
//         Robot tmpRobt=robots[robot_id];
//         auto tmp=Calculate_the_trajectory(tmpRobt,0,25);
     
//         inflection.first = tmp[tmp.size()-1].first;
//         inflection.second = tmp[tmp.size()-1].second;
//         dist=tmp.size()*0.02*6;
//         dist += calcuDis(inflection,studios[studio_id].pos);
//         robots[robot_id].target_id = target;
//         time = tmp.size()*0.02+calcuDis(inflection,studios[studio_id].pos)/6;
//     }
//     double dist2 = precise_distance(robot_id,studio_id);

//     if(class_map == 3 && state.FrameID <= 8000) 
//         dist2 = calcuDis(robots[robot_id].pos,studios[studio_id].pos);

//     if(fabs(dist-dist2)>10){ 
//         dist = dist2;
//         time = dist/6;
//     }
    
//     return pair<double,double>(time,dist);
// }

// bool check_area(int robot_id,int studio_id){
//     int count = 0;
//     for(int i =0;i<4;i++){
//         if(i!=robot_id){
//             if (studios[studio_id].area == robot_area[i] && robot_area[i]!=0)
//             {
//                 count++;
                
//             }
//         }
//     }
//     if(count>=2)return false;
//     return true;
// }

// bool check_area_right(int studio_id) {
//     if (studios[studio_id].area == 2|| studios[studio_id].area == 4)
//     {
//         return false;
//     }
//     return true;
// }
// double change_area(int robot_id,int studio_id){
//     if(studios[robots[robot_id].target_id].type <=3){
//         if(studios[studio_id].area == 2 || studios[studio_id].area == 4)
//             return 1000;
//         if (robot_area[robot_id] != studios[studio_id].area){
//             return 2;
//         }
//     }
//     return 1;
// }

// pair<int, double>pick_point(int robot_id, int state_type)
// {
//     pair<double,double> pos = robots[robot_id].pos;
//     double min = 1000;
//     int min_subscript = -1;
//     int i,j;
//     int studio_id;
//     double dist;
//     int item_type = robots[robot_id].get_type;
//     if(state_type == 1){
//         //qu 123
//         for(i=0;i<studios.size();i++){
//             if(studios[i].type >= 1 && studios[i].type <= 3 && (studios[i].r_id==-1 ) ){  
//                 if(studios[i].pStatus == 1||(studios[i].r_time>0)){
//                     if(robot_get_type[studios[i].type]< material[robot_id][studios[i].type].size()){
//                         dist = init_robot_dis[robot_id][i];
//                         if(eq(dist,1000))continue;
//                         if (lt(dist,min))
//                         {
//                             min = dist;
//                             min_subscript = i;
//                         }
                            
//                     }
//                 }
//             }
//         }
//     }
//     else if(state_type == 2){
//         //qu 123
//         for(i=0;i<studios.size();i++){
//             if(studios[i].type >= 1 && studios[i].type <= 3 && (studios[i].r_id==-1 ) ){  
//                 if(studios[i].pStatus == 1){
//                     if(robot_get_type[studios[i].type]< material[robot_id][studios[i].type].size()){
//                         dist = studio_dis[0][robots[robot_id].loc_id][i];
//                         if(eq(dist,1000))continue;
//                         if (lt(dist,min))
//                         {
//                             min = dist;
//                             min_subscript = i;
//                         }
                            
//                     }
//                 }
//             }
//         }
//     }
//     else if(state_type == 3){
//         //qu 456
//         for(i=0;i<studios.size();i++){
//             if(studios[i].type >= 4 && studios[i].type <= 6 && (studios[i].r_id==-1 )){  //456 and no robot choose ,get
//                 // if(studios[i].pStatus == 1 ||(studios[i].r_time>0&&(checkEnough(robot_id,i,studios[i].r_time)))){
//                     // cerr<<"ccc1"<<endl;
//                 if(studios[i].pStatus == 1){
//                     if(robot_get_type[studios[i].type]< material[robot_id][studios[i].type].size()){
//                         dist = studio_dis[0][robots[robot_id].loc_id][i];
//                         if(eq(dist,1000))continue;
//                         if (lt(dist,min))
//                         {
//                             min = dist;
//                             min_subscript = i;
//                         }       
//                     }
//                 }
//             }
//         }
//     }
//     else if(state_type == 4){
//         //qu 7
//         for(i=0;i<studios.size();i++){
//             if(studios[i].type ==7 && (studios[i].r_id==-1)){  
//                 if(studios[i].pStatus == 1){
//                     if(robot_get_type[studios[i].type]< material[robot_id][studios[i].type].size()){
//                         dist = studio_dis[0][robots[robot_id].loc_id][i];
//                         if(eq(dist,1000))continue;
//                         if (lt(dist,min))
//                         {
//                             min = dist;
//                             min_subscript = i;
//                         }
//                     }
//                 }
//             }
//         }
//     }
//     else if(state_type == 5){             
//             //send
//         for(i=0;i<studios.size();i++){
//             for(j=1;j<=material_send[item_type][0];j++){
//                 if(item_type != 7){
//                     if(studios[i].type == material_send[item_type][j] && (studios_rid[i][item_type] == -1) ){
//                         if(((studios[i].bitSatus & ((int)pow(2,item_type)))==0)||((check_material_full(i)&&(studios[i].pStatus != 1)&&(studios[i].r_time>0&&(checkEnough(robot_id,i,studios[i].r_time)))))){
//                             dist = studio_dis[1][robots[robot_id].loc_id][i];
//                             if(eq(dist,1000))continue;
//                             if (lt(dist,min))
//                             {
//                                 min = dist;
//                                 min_subscript = i;
//                             }
//                         }
//                     }
//                 }
//                 else{
//                     if(studios[i].type == material_send[item_type][j]){
//                        dist = studio_dis[1][robots[robot_id].loc_id][i];
//                        if(eq(dist,1000))continue;
//                         if (lt(dist,min))
//                         {
//                             min = dist;
//                             min_subscript = i;
//                         }
//                     }
//                 }
//             }
//             if(studios[i].type == 9){
//                 dist = studio_dis[1][robots[robot_id].loc_id][i];
//                 if(eq(dist,1000))continue;
//                 if (lt(dist,min))
//                 {
//                     min = dist;
//                     min_subscript = i;
//                 }
//             }
//         }
//     }
//     return pair<int, double>(min_subscript, min);
// }
// bool check_get_send(int robot_id,int studio_id){
//     int j;
//     for(int i=0;i<material[robot_id][studios[studio_id].type].size();i++){
//         j = material[robot_id][studios[studio_id].type][i];
//         // cerr<<"robot : "<<robot_id<<" studio :"<<studio_id<<" studio_type "<<studios[studio_id].type<<endl;
//         // cerr<<j<<"  studio_type "<<studios[j].type<<endl;
//         if(studios_rid[j][studios[studio_id].type]== -1){
//             if(lt(dis_to_studios[j][1][studios[studio_id].node_id],10000))
//                 // cerr<<"true"<<endl;
//                 return true;
//         }
//     }
//     return false;
// }
// pair<int, double>pick_point(int robot_id, int state_type)
// {
//     pair<double,double> pos = robots[robot_id].pos;
//     double min = 10000;
//     int min_subscript = -1;
//     int i,j;
//     int studio_id;
//     double dist;
//     int item_type = robots[robot_id].get_type;
//     if(state_type == 1){
//         //qu 123
//         for(i=0;i<studios.size();i++){
//             if(studios[i].type >= 1 && studios[i].type <= 3 && (studios[i].r_id==-1 ) ){  
//                 if(studios[i].pStatus == 1||(studios[i].r_time>0)){
//                     if(robot_get_type[studios[i].type]< material[robot_id][studios[i].type].size()){
//                         dist = dis_to_studios[robots[robot_id].node_id][i][0];
//                         if(eq(dist,10000))continue;
//                         if(check_get_send(robot_id,i)){
//                             if (lt(dist,min))
//                             {
//                                 // cerr<<"bb"<<endl;
//                                 min = dist;
//                                 min_subscript = i;
//                             }
//                             // cerr<<"aaa"<<endl;
//                         }
//                     }
//                 }
//             }
//             if(studios[i].type >= 1 && studios[i].type <= 3){
//                 if (((studios[i].r_id != -1)&&((studios[i].r_id < 50)))){
//                     if((robot_get_type[studios[i].type]< material[robot_id][studios[i].type].size())){
//                         dist = dis_to_studios[robots[robot_id].node_id][i][0];
//                         if(eq(dist,10000))continue;
//                         if(lt(dist/6/0.02,50))continue;
//                         if(check_get_send(robot_id,i)){
//                             if (lt(dist,min))
//                             {
//                                 // cerr<<"bb"<<endl;
//                                 min = dist;
//                                 min_subscript = i;
//                             }
//                             // cerr<<"aaa"<<endl;
//                         }       
//                     }
//                 }
//             }
//         }
//     }
//     else if(state_type == 2){
//         //qu 123
//         for(i=0;i<studios.size();i++){
//             if(studios[i].type >= 1 && studios[i].type <= 3 && (studios[i].r_id==-1 ) ){  
//                 if(studios[i].pStatus == 1){
//                     if(robot_get_type[studios[i].type]< material[robot_id][studios[i].type].size()){
//                         dist = dis_to_studios[robots[robot_id].node_id][i][0];
//                         if(eq(dist,10000))continue;
//                         if(check_get_send(robot_id,i)){
//                             if (lt(dist,min))
//                             {
//                                 min = dist;
//                                 min_subscript = i;
//                             }
//                         } 
//                     }
//                 }
            
//             }
//             if(studios[i].type >= 1 && studios[i].type <= 3){
//                 if (((studios[i].r_id != -1)&&((studios[i].r_id < 50)))){
//                     if(robot_get_type[studios[i].type]< material[robot_id][studios[i].type].size()){
//                         dist = dis_to_studios[robots[robot_id].node_id][i][0];
//                         if(eq(dist,10000))continue;
//                         if(lt(dist/6/0.02,50))continue;
//                         if(check_get_send(robot_id,i)){
//                             if (lt(dist,min))
//                             {
//                                 min = dist;
//                                 min_subscript = i;
//                             }
//                         }
//                     }
//                 }
//             }
//         }
//     }
//     else if(state_type == 3){
//         //qu 456
//         for(i=0;i<studios.size();i++){
//             if(studios[i].type >= 4 && studios[i].type <= 6 && (studios[i].r_id==-1 )){  //456 and no robot choose ,get
//                 // if(studios[i].pStatus == 1 ||(studios[i].r_time>0&&(checkEnough(robot_id,i,studios[i].r_time)))){
//                     // cerr<<"ccc1"<<endl;
//                 if(studios[i].pStatus == 1){
//                     if(robot_get_type[studios[i].type]< material[robot_id][studios[i].type].size()){
//                         dist = dis_to_studios[robots[robot_id].node_id][i][0];
//                         if(eq(dist,10000))continue;
//                         if(check_get_send(robot_id,i)){
//                             if (lt(dist,min))
//                             {
//                                 min = dist;
//                                 min_subscript = i;
//                             }
//                         }
//                     }
//                 }
//             }
//         }
//     }
//     else if(state_type == 4){
//         //qu 7
//         for(i=0;i<studios.size();i++){
//             if(studios[i].type ==7 && (studios[i].r_id==-1)){  
//                 if(studios[i].pStatus == 1){
//                     if(robot_get_type[studios[i].type]< material[robot_id][studios[i].type].size()){
//                         dist = dis_to_studios[robots[robot_id].node_id][i][0];
//                         if(eq(dist,10000))continue;
//                         if(check_get_send(robot_id,i)){
//                             if (lt(dist,min))
//                             {
//                                 min = dist;
//                                 min_subscript = i;
//                             }
//                         }
//                     }
//                 }
//             }
//         }
//     }
//     else if(state_type == 5){             
//             //send
//         for(i=0;i<studios.size();i++){
//             for(j=1;j<=material_send[item_type][0];j++){
//                 if(item_type != 7){
//                     if(studios[i].type == material_send[item_type][j] && (studios_rid[i][item_type] == -1) ){
//                         if(((studios[i].bitSatus & ((int)pow(2,item_type)))==0)||((check_material_full(i)&&(studios[i].pStatus != 1)&&(studios[i].r_time>0&&(checkEnough(robot_id,i,studios[i].r_time)))))){
//                             dist = dis_to_studios[robots[robot_id].node_id][i][1];
//                             if(eq(dist,10000))continue;
//                             if (lt(dist,min))
//                             {
//                                 min = dist;
//                                 min_subscript = i;
//                             }
//                         }
//                     }
//                 }
//                 else{
//                     if(studios[i].type == material_send[item_type][j]){
//                        dist = dis_to_studios[robots[robot_id].node_id][i][1];
//                        if(eq(dist,10000))continue;
//                         if (lt(dist,min))
//                         {
//                             min = dist;
//                             min_subscript = i;
//                         }
//                     }
//                 }
//             }
//             if(studios[i].type == 9){
//                 dist = dis_to_studios[robots[robot_id].node_id][i][1];
//                 if(eq(dist,10000))continue;
//                 if (lt(dist,min))
//                 {
//                     min = dist;
//                     min_subscript = i;
//                 }
//             }
//         }
//     }
//     else if (state_type == 6){
//         for (i = 0; i < studios.size(); i++)
//         {
//             if (studios[i].r_id == -1)
//             { // 7 and no robot choose ,get
//                 // if(studios[i].pStatus == 1 ||(studios[i].r_time>0&&(checkEnough(robot_id,i,studios[i].r_time)))){
//                 // cerr<<"ddd1"<<endl;
//                 if (studios[i].pStatus == 1 || (studios[i].r_time > 0))
//                 {
//                     // if (studios[i].pStatus == 1)
//                     // {
//                     // cerr<<"ddd2"<<endl;
//                     if (robot_get_type[studios[i].type] < material[robot_id][studios[i].type].size())
//                     {
                        
//                             dist = dis_to_studios[robots[robot_id].node_id][i][0]+back_dis(robot_id,i);
//                             if (dist < ((15000 - state.FrameID) * 0.02)*2)
//                             {
//                                 if (dist < min)
//                                 {
//                                     min = dist;
//                                     min_subscript = i;
//                                 }
//                             }
                        
//                     }
//                 }
//             }
//         }
//     }
//     return pair<int, double>(min_subscript, min);
// }


// pair<int, double> choose_lack(int studio_id, int threshold)
// {
//     double dist;
//     double min = 100;
//     int min_subscript = -1;
//     if (studios[studio_id].type > 3 && studios[studio_id].type < 8)
//         {
//             for (int i = 0; i < 4; i++)
//             {
//                 if (studios[studio_id].type == i + 4)
//                 {
//                     for (int j = 0; j < studio_material[i][0]; j++)
//                     {
//                         if ((studios[studio_id].bitSatus & (int)pow(2, studio_material[i][j + 1])) == 0)
//                         {
//                             if (studios_rid[studio_id][studio_material[i][j + 1]] == -1)
//                             {
//                                             // cerr<<" product[studio_material[i][j+1]].size() "<<product[studio_material[i][j+1]].size()<<endl;
//                             for (int k = 0; k < product[studio_material[i][j + 1]].size(); k++)
//                             {
//                                 if (studios[product[studio_material[i][j + 1]][k]].r_id == -1)
//                                 {
//                                                     // cerr<<" studios[studio_id].pos = "<<studios[studio_id].pos.first<<' '<<studios[studio_id].pos.second<<endl;
//                                                     // cerr<<" studios[product[studio_material[i][j+1]][k]].pos"<<studios[product[studio_material[i][j+1]][k]].pos.first<<' '<<studios[product[studio_material[i][j+1]][k]].pos.second<<endl;
//                                     dist = dis[studios[studio_id].pane_id][studios[product[studio_material[i][j + 1]][k]].pane_id];
//                                     if (dist < min)
//                                     {
//                                         min = dist;
//                                         min_subscript = product[studio_material[i][j + 1]][k];
//                                     }
//                                 }
//                             }
//                         }
//                                         // cerr<<" dist = "<<dist<<" threshold = "<<threshold<<endl;
//                     }
//                 }
//             }
//         }
//     }
//     if (min < threshold)
//         return pair<int, double>(min_subscript, min);
//     return pair<int, double>(-1, -1);
// }
// void first_action()
// {
//     int i, j;
//     pair<int, double> p;
//     pair<int, double> f;
//     for (i = 0; i < robots.size(); i++)
//     {
//         robots[i].target_id = pick_point(i, 1).first;

//     // for (i = 0; i < robots.size(); i++)
//     // {
//     //     studios[robots[i].target_id].r_id = i;
//     //     for (j = i + 1; j < robots.size(); j++)
//     //     {
//     //         if (robots[i].target_id == robots[j].target_id)
//     //         {
//     //             p = pick_point(i, 1);
//     //             f = pick_point(j, 1);
//     //             if (gt(p.second, f.second))
//     //             {
//     //                 studios[robots[i].target_id].r_id = i;
//     //                 robots[j].target_id = f.first;
//     //                 studios[robots[j].target_id].r_id = j;
//     //             }
//     //             else
//     //             {
//     //                 studios[robots[j].target_id].r_id = j;
//     //                 robots[i].target_id = p.first;
//     //                 studios[robots[i].target_id].r_id = i;
//     //             }
//     //         }
//     //     }
//     // }
//     // for (i = 0; i < robots.size(); i++)
//     // {
//         if(robots[i].target_id != -1){
//             // if(robots[i].robot_area_type != studios[robots[i].target_id].studio_area_type){
//             //     robots[i].virtual_pos = types[0][robots[i].robot_area_type[0]].entrance[target_sequence[0][robots[i].robot_area_type[0]][studios[robots[i].target_id].studio_area_type[0]]];
//             // }
//             // else{
//             //     robots[i].virtual_pos = studios[robots[i].target_id].pos;
//             // }
//             // cerr<<i<<"-"<<robots[i].target_id<<endl;
//             // cerr<<robots[i].node_id<<endl;
//             // cerr<<studios[robots[i].target_id].node_id<<endl;
//             // printPath(i, 1, robots[i].target_id, 0);
//             robots[i].virtual_id = next_node[robots[i].node_id][robots[i].target_id][(robots[i].get_type != 0)];
//             studios[robots[i].target_id].r_id = i;
//             robot_get_type[studios[robots[i].target_id].type]++;
//             // if(robots[i].get_type==0)cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<studios[robots[i].target_id].type<<" buy "<<ins[i].buy<<" sell "<<ins[i].sell<<endl;
//             // else cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" buy "<<ins[i].buy<<" sell "<<ins[i].sell<<endl;
//         }
//         else {
//             robots[i].virtual_id = -1;
//             robots[i].virtual_pos = pair<double,double>(0,0);
//         }
//     }
// }

// bool judge_full(int level, double threshold)
// {
//     int i, j;
//     int count = 0;
//     int full_count = 0;
//     double v;
//     full_product.clear();
//     if (level == 2)
//     {
//                         for (i = 0; i < studios.size(); i++)
//                         {
//                             if (studios[i].type >= 4 && studios[i].type <= 7)
//                             {
//                                 count++;
//                                 if (studios[i].pStatus == 1 && studios[i].r_id == -1)
//                                 {
//                                     if(class_map == 3){
//                                         full_count++;
//                                         full_product.push_back(i);
//                                     }
//                                     else{
//                                         for (j = 0; j < studio_material[studios[i].type - 4][0]; j++)
//                                         {
//                                             if ((studios[i].bitSatus & (int)pow(2, studio_material[studios[i].type - 4][j + 1])) == 0)
//                                                 break;
//                                         }
//                                         if (j == studio_material[studios[i].type - 4][0])
//                                         {
//                                             full_count++;
//                                             full_product.push_back(i);
//                                             // cerr<<" stuidio_id = "<<i<<" studios[i].type = "<<studios[i].type<<" studios[i].pStatus = "<<studios[i].pStatus<<" studios[i].bitSatus = "<<studios[i].bitSatus<<endl;
//                                         }
//                                     }
//                                 }
//                             }
//                         }
//                         v = (double)full_count / (double)count;
//                         // cerr<<" full_count = "<<full_count<<" count = "<<count<<" bilu "<<v<<endl;
//                         if (v >= threshold)
//                         {
//                             // print_matr();
//                             return true;
//                         }
//     }
//     if (level == 3)
//     {
//                         for (i = 0; i < studios.size(); i++)
//                         {
//                             if (studios[i].type == 7)
//                             {
//                                 count++;
//                                 if (studios[i].pStatus == 1 && studios[i].r_id == -1)
//                                 {
//                                     for (j = 0; j < studio_material[studios[i].type - 4][0]; j++)
//                                     {
//                                         if ((studios[i].bitSatus & (int)pow(2, studio_material[studios[i].type - 4][j + 1])) == 0)
//                                             break;
//                                     }
//                                     if (j == studio_material[studios[i].type - 4][0])
//                                     {
//                                         // cerr<<" stuidio_id = "<<i<<" studios[i].type = "<<studios[i].type<<" studios[i].pStatus = "<<studios[i].pStatus<<" studios[i].bitSatus = "<<studios[i].bitSatus<<endl;
//                                         full_product.push_back(i);
//                                         full_count++;
//                                     }
//                                 }
//                             }
//                         }
//                         v = (double)full_count / (double)count;
//                         // cerr<<" full_count = "<<full_count<<" count = "<<count<<" bilu "<<v<<endl;
//                         if (v >= threshold)
//                         {
//                             // print_matr();
//                             return true;
//                         }
//     }
//     return false;
// }
bool check_material_full(int studio_id)
{
    int j;
    if(studios[studio_id].type>=3 &&studios[studio_id].type<=7){
        for (j = 0; j < studio_material[studios[studio_id].type - 4][0]; j++)
        {
            if ((studios[studio_id].bitSatus & (int)pow(2, studio_material[studios[studio_id].type - 4][j + 1])) == 0)
                break;
        }
        if (j == studio_material[studios[studio_id].type - 4][0])
        {
            return true;
        }
    }
    return false;
}
int find_closest_studio(int robot_id){
    double dist;
    double min = 10000;
    double min_subscript = -1;
    for(int i = 0;i<studios.size();i++){
        dist = calcuDis(robots[robot_id].pos,studios[i].pos);
        if(lt(dist,min)){
            min=dist;
            min_subscript = i;
        }
    }
    return min_subscript;
}
// void robot_judge_sol(int threshold_lack,int full){
//     int i,k,m,is_take;
//     int target,min_subscript=-1;
//     double min_dist=10000,dist;
//     int x,y;
//     pair<int,double> temp1;
//     pair<int,double> temp2;
//     //cerr<<robots.size()<<endl;
//     for(i = 0; i < robots.size(); i++){
//         if(robots[i].get_type==0)is_take=0;
//         else is_take=1;
//         // cerr<<"robot :"<<i<<"distance : "<<calcuDis(robots[i].pos,robots[i].virtual_pos)<<endl;
//         if(lt(calcuDis(robots[i].pos,robots[i].virtual_pos),0.2)&&robots[i].target_id!=-1){
            
//             // if(robots[i].robot_area_type[is_take] != studios[robots[i].target_id].studio_area_type[is_take]){
//             //     x= robots[i].robot_area_type[is_take];
//             //     y= target_sequence[is_take][robots[i].robot_area_type[is_take]][studios[robots[i].target_id].studio_area_type[is_take]];
//             //     // if((eq(robots[i].virtual_pos.first,types[x].entrance[y].first)&&eq(robots[i].virtual_pos.second,types[x].entrance[y].second))){
//             //     //     robots[i].virtual_pos = types[y].entrance[x];
//             //     // }
//             //     // else if(eq(robots[i].virtual_pos.first,types[y].entrance[x].first)&&eq(robots[i].virtual_pos.second,types[y].entrance[x].second)){
//             //     //     x = y;
//             //     //     y = target_sequence[x][studios[robots[i].target_id].studio_area_type];
//             //     //     robots[i].virtual_pos = types[x].entrance[y];
//             //     // }
//             //     if(x != y){
//             //         if(!(eq(robots[i].virtual_pos.first,types[is_take][x].entrance[y].first)&&eq(robots[i].virtual_pos.second,types[is_take][x].entrance[y].second))&&!(eq(robots[i].virtual_pos.first,types[is_take][y].entrance[x].first)&&eq(robots[i].virtual_pos.second,types[is_take][y].entrance[x].second))){
//             //             robots[i].virtual_pos = types[is_take][x].entrance[y];
//             //         }
//             //         else robots[i].virtual_pos = types[is_take][y].entrance[x];
//             //     }
//             //     else{
//             //         robots[i].virtual_pos = types[is_take][x].entrance[y];
//             //     }
//             // }
//             // else{
//             //     robots[i].virtual_pos = studios[robots[i].target_id].pos;
//             // }
//             robots[i].virtual_id = next_node[robots[i].node_id][robots[i].target_id][(robots[i].get_type != 0)];
//         }
//         if((robots[i].loc_id == robots[i].target_id && robots[i].target_id != -1)){
//             if(robots[i].get_type == 0){
//                 if(studios[robots[i].target_id].pStatus == 1){
//                     int now_i=robots[i].pos.second/0.5;
//                     int now_j=robots[i].pos.first/0.5;
//                     int tmpID=now_i*100+now_j;
//                     bool con1=(exist_id[1].count(tmpID)!=0)?true:false;
//                     if(!con1){
//                         ins[i].buy = -1;
//                         ins[i].sell = -1;
//                         continue;
//                     }
//                     robots[i].lastSign=0;
//                     robots[i].isTurn=0;
//                     robots[i].get_type = studios[robots[i].loc_id].type;
//                     if (studios[robots[i].loc_id].r_id >= 50)
//                         studios[robots[i].loc_id].r_id -= 50;
//                     else
//                         studios[robots[i].loc_id].r_id = -1;
//                     studios[robots[i].loc_id].pStatus = 0;
                    
//                     robots[i].target_id = pick_point(i, 5).first;
//                     // cerr<<"target_id "<<robots[i].target_id<<endl;
//                     if (robots[i].target_id != -1)
//                      {
//                         // studios[robots[i].target_id].r_id = i;
//                         // x= robots[i].robot_area_type[is_take];
//                         // y= studios[robots[i].target_id].studio_area_type[is_take];
//                         // if(x != y){
//                         //     y= target_sequence[is_take][robots[i].robot_area_type[is_take]][studios[robots[i].target_id].studio_area_type[is_take]];
//                         //     if(lt(calcuDis(robots[i].pos,types[is_take][x].entrance[y]),0.6)){
//                         //         robots[i].virtual_pos = types[is_take][y].entrance[x];
//                         //     }
//                         //     else{
//                         //         robots[i].virtual_pos = types[is_take][x].entrance[y];
//                         //     }
//                         //     // cerr<<"sss"<<i<<' '<<x<<' '<<y<<endl;
//                         //     // printPair(robots[i].virtual_pos);
//                         // }
//                         // else{
//                         //     robots[i].virtual_pos = studios[robots[i].target_id].pos;
//                         // }
//                         robots[i].virtual_id = next_node[robots[i].node_id][robots[i].target_id][(robots[i].get_type != 0)];
//                         if(studios[robots[i].target_id].type!=8&&studios[robots[i].target_id].type!=9)studios_rid[robots[i].target_id][robots[i].get_type] = i;
//                         // cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<" flag "<<studios_rid[robots[i].target_id][robots[i].get_type]<<endl;
//                     }
//                     else robots[i].virtual_id = -1;
//                     ins[i].buy = 1;
//                     ins[i].sell = -1;
//                     if(state.FrameID>13000){
//                         // cerr <<"***";
//                         if(!checkEnough(i,robots[i].target_id,13000-state.FrameID)){
//                             ins[i].buy = -1;
//                             // robot_get_type[studios[robots[i].target_id].type]--;
//                             // cerr<<"kkk"<<endl;
//                             if (studios[robots[i].target_id].type != 8 && studios[robots[i].target_id].type != 9) studios_rid[robots[i].target_id][robots[i].get_type] = -1;
//                             robots[i].target_id = -1;
//                             robots[i].virtual_id = -1;
//                         }
//                     }
//                 }
//                 else{
//                     ins[i].buy = -1;
//                     ins[i].sell = -1;
//                     // cerr<<" robot "<<i<<" can not buy "<<studios[robots[i].target_id].type<<endl;
//                 }
//             }
//             else{
//                 // cerr<<endl;
//                 //dosomething sell
//                 ins[i].sell = 1;
//                 ins[i].buy = -1;
//                 robots[i].lastSign=0;
//                 robots[i].isTurn=0;
//                 // cerr<<"loss = "<<robots[i].time_val*robots[i].collision_val<<' '<<robots[i].time_val<<' '<<robots[i].collision_val<<' '<<robots[i].get_type<<' '<<(price[robots[i].get_type][1]*robots[i].time_val*robots[i].collision_val-price[robots[i].get_type][0])<<endl;
//                 // cerr<<"robots "<< i<<" sell "<<robots[i].get_type<<endl;
//                 studios[robots[i].loc_id].bitSatus += (int)pow(2,robots[i].get_type);
//                 //studios[robots[i].loc_id].r_id = -1;
//                 studios_rid[robots[i].loc_id][robots[i].get_type] = -1;
//                 // robot_get_type[robots[i].get_type]--;
//                 robots[i].get_type = 0;
//                 // target = -1;
//                 if(state.FrameID>12000){
//                     robots[i].target_id = pick_point(i,6).first;
//                     if (robots[i].target_id != -1){
//                         if (studios[robots[i].target_id].r_id != -1 && studios[robots[i].target_id].r_id != i)
//                             studios[robots[i].target_id].r_id += 50;
//                         else
//                             studios[robots[i].target_id].r_id = i;
//                         robot_get_type[studios[robots[i].target_id].type]++;
//                         robots[i].virtual_id = next_node[robots[i].node_id][robots[i].target_id][(robots[i].get_type != 0)];
//                     }
//                     else{
//                         robots[i].virtual_id = -1;
//                     }
//                 }
//                 else{
//                     // target = choose_lack(robots[i].loc_id, threshold_lack).first;
//                     //     // if (class_map == 3)target = -1;
//                     //         // cerr<<"robots[i].loc_id "<<robots[i].loc_id<<"target = "<<target<<endl;
//                     // if (target != -1)
//                     // {
//                     //     robots[i].target_id = target;
//                     //     robot_get_type[studios[robots[i].target_id].type]++;
//                     //     studios[robots[i].target_id].r_id = i;
//                     // }
//                     // else{
//                         min_subscript = -1;
//                         min_dist = 10000;
//                         // cerr<<"dddd "<<full<<endl;
//                         // if(full !=0){
//                         //     //cerr<<"fdfdf "<<endl;
//                         //     temp1=pick_point(i,1);
//                         //     //cerr<<"temp1 = "<<temp1.first<<' '<<temp1.second<<endl;
//                         //     for(int j=2;j<=4;j++){
//                         //         temp2=pick_point(i,j);
//                         //         dist = temp2.second;
//                         //         if(min_dist>dist){
//                         //             min_dist=dist;
//                         //             min_subscript=temp2.first;
//                         //             k=j;
//                         //         }
//                         //     }
//                         //         //cerr<<"min = "<<min_subscript<<' '<<min_dist<<endl;
//                         //         // if(class_map==1){
//                         //         //     if(temp1.second<min_dist*1.5){
//                         //         //         min_dist=temp1.second;
//                         //         //         min_subscript=temp1.first;
//                         //         //     }
//                         //         // }
//                         //         // else{
//                         //         if(temp1.second<min_dist*1.5){
//                         //             min_dist=temp1.second;
//                         //             min_subscript=temp1.first;
//                         //         }
//                         //         // }
//                         // }
//                         // else{
//                             for(int j=2;j<=4;j++){
//                                 temp1=pick_point(i,j);
//                                 dist = temp1.second;
//                                 if(min_dist>dist){
//                                     min_dist=dist;
//                                     min_subscript=temp1.first;
//                                     k=j;
//                                 }
//                             }
//                             // }
//                         // }
//                         // if(state.FrameID>4700 &&state.FrameID<4800&&i==2){
//                         //     cerr<<"wwww"<<k<<' '<<temp1.first<<' '<<temp1.second<<endl;
//                         // }
//                         if(robots[i].target_id != -1)
//                             robots[i].last_target_id = robots[i].target_id;
//                         robots[i].target_id = min_subscript;
//                             // cerr<<"aaa"<<endl;
//                         if(min_subscript != -1){
//                             // if(robots[i].robot_area_type[is_take] != studios[robots[i].target_id].studio_area_type[is_take]){
//                             //     x= robots[i].robot_area_type[is_take];
//                             //     y= target_sequence[is_take][robots[i].robot_area_type[is_take]][studios[robots[i].target_id].studio_area_type[is_take]];
//                             //     if(lt(calcuDis(robots[i].pos,types[is_take][x].entrance[y]),0.6)){
//                             //         robots[i].virtual_pos = types[is_take][y].entrance[x];
//                             //     }
//                             //     else{
//                             //         robots[i].virtual_pos = types[is_take][x].entrance[y];
//                             //     }
//                             // }
//                             // else{
//                             //     robots[i].virtual_pos = studios[robots[i].target_id].pos;
//                             // }
//                             robots[i].virtual_id = next_node[robots[i].node_id][robots[i].target_id][(robots[i].get_type != 0)];
//                             if (studios[robots[i].target_id].r_id != -1 && studios[robots[i].target_id].r_id != i)
//                                 studios[robots[i].target_id].r_id += 50;
//                             else
//                                 studios[robots[i].target_id].r_id = i;
//                             robot_get_type[studios[robots[i].target_id].type]++;
//                         }
//                         else robots[i].virtual_id = -1;
//                     }
                

//             }
//         }       
//         else{
            
//             ins[i].buy = -1;
//             ins[i].sell = -1;
//             ins[i].destroy = -1;
//         }
//         if(robots[i].target_id == -1){
//             min_dist=10000;
//             min_subscript = -1;
//             if(robots[i].get_type ==0){
//                 // cerr<<endl;
//                 for(int j=2;j<=4;j++){
//                     temp1=pick_point(i,j);
//                     // cerr<<temp1.first<<' '<<temp1.second<<endl;
//                     dist = temp1.second;

//                     if(min_dist>dist){
//                         min_dist=dist;
//                         min_subscript=temp1.first;
//                         // k=j;
//                     }
//                 }
//                 // cerr<<min_subscript<<endl;
                
//                 robots[i].target_id = min_subscript;
//                 // cerr<<"ccc"<<endl;
//                 if(robots[i].target_id!= -1){
//                     // if(robots[i].robot_area_type != studios[robots[i].target_id].studio_area_type){
//                     //     x= robots[i].robot_area_type;
//                     //     y= target_sequence[robots[i].robot_area_type][studios[robots[i].target_id].studio_area_type];
//                     //     if(lt(calcuDis(robots[i].pos,types[x].entrance[y]),0.6)){
//                     //         robots[i].virtual_pos = types[y].entrance[x];
//                     //     }
//                     //     else{
//                     //         robots[i].virtual_pos = types[x].entrance[y];
//                     //     }
//                     // }
//                     // else{
//                     //     robots[i].virtual_pos = studios[robots[i].target_id].pos;
//                     // // }
//                     robots[i].virtual_id = next_node[robots[i].node_id][robots[i].target_id][(robots[i].get_type != 0)];
//                     // cerr<<"aaa"<<robots[i].road_id <<"aaa"<<road[(robots[i].get_type != 0)][robots[i].road_id][robots[i].now_index].id<<endl;
//                         //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
//                     robot_get_type[studios[robots[i].target_id].type]++;
//                     if (studios[robots[i].target_id].r_id != -1)
//                         studios[robots[i].target_id].r_id += 50;
//                     else
//                         studios[robots[i].target_id].r_id = i;
//                     //cerr<< "kkkkk"<<endl;
//                 }
//             }
//             else{
//                 cerr<<endl;
                
//                 robots[i].target_id = pick_point(i,5).first;
//                 // cerr<<"eee"<<endl;
//                 if(robots[i].target_id != -1){
//                     // x= robots[i].robot_area_type;
//                     // y= studios[robots[i].target_id].studio_area_type;
//                     // if(x != y){
//                     //     y= target_sequence[robots[i].robot_area_type][studios[robots[i].target_id].studio_area_type];
//                     //     if(lt(calcuDis(robots[i].pos,types[x].entrance[y]),0.6)){
//                     //         robots[i].virtual_pos = types[y].entrance[x];
//                     //     }
//                     //     else{
//                     //         robots[i].virtual_pos = types[x].entrance[y];
//                     //     }
//                     //         // cerr<<"sss"<<i<<' '<<x<<' '<<y<<endl;
//                     //         // printPair(robots[i].virtual_pos);
//                     // }
//                     // else{
//                     //     robots[i].virtual_pos = studios[robots[i].target_id].pos;
//                     // // }
//                     robots[i].virtual_id = next_node[robots[i].node_id][robots[i].target_id][(robots[i].get_type != 0)];
//                     if(studios[robots[i].target_id].type != 8 && studios[robots[i].target_id].type != 9)studios_rid[robots[i].target_id][robots[i].get_type] = i;
//                     //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
//                 }
//                 else robots[i].virtual_id = -1;
//             }
//         }
//         // if(robots[i].get_type==0)cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<studios[robots[i].target_id].type<<" buy "<<ins[i].buy<<" sell "<<ins[i].sell<<endl;
//         // else cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" buy "<<ins[i].buy<<" sell "<<ins[i].sell<<endl;
//         // cerr<<robots[i].now_index<<endl;
//     }

// }

// void robot_action(){
//     int col,row,is_take;
//     for(int i=0;i<robots.size();i++){
//         if(robots[i].get_type==0)is_take=0;
//         else is_take=1;
//         col = (robots[i].pos.first-0.25)/0.5;
//         row = (robots[i].pos.second-0.25)/0.5;
//         // robots[i].robot_area_type[is_take] = graphs[is_take][row][col];
//         // printPair(robots[i].virtual_pos);
//     }
//     for(int i =0;i<=7;i++)robot_get_type[i]=0;
//     for(int i = 0;i<4;i++){
//         if(robots[i].get_type != 0)robot_get_type[robots[i].get_type]++;
//         else if(robots[i].target_id != -1)robot_get_type[studios[robots[i].target_id].type]++;
//     }
//     // if(state.FrameID == 9324){
//     //     for(int i = 0;i<studios.size();i++){
//     //         cerr<<studios[i].r_id<<endl;
//     //     }
//     // }
//     robot_judge_sol(5, 0);
// }
pair<pair<int,int>,double> new_pick_point(int robot_id,int state_type){
    double max = 0;
    int studio_buy = -1,studio_send = -1;
    double dist;
    double income;
    double income_ratio;      //收益比
    int material_studio_id;
    // if(state_type == 1){
    //     // for(int i =0;i<studios.size();i++){
    //     //     if(studios[i].type<=3){
    //     //     }
    //     // }
    // }
    // else 
    if(state_type ==2){
        for(int i =0;i<studios.size();i++){
            if(studios[i].type>3){
                for(int j = 1;j<=studio_material[studios[i].type-4][0];j++){
                    if(((studios[i].bitSatus & (int)pow(2,studio_material[studios[i].type-4][j])) == 0||(check_material_full(i)&&studios[i].r_time == -1)) &&(studios_rid[i][studio_material[studios[i].type-4][j]]==-1)){
                        for(int k=0;k<studios[i].material_studios[j-1].size();k++){
                            material_studio_id = studios[i].material_studios[j-1][k];
                            if((studios[material_studio_id].pStatus == 1||(studios[material_studio_id].r_time>0)) && studios[material_studio_id].r_id < 50 ){         //
                                income = price[studios[material_studio_id].type][1]-price[studios[material_studio_id].type][0];
                                dist = dis_to_studios[material_studio_id][0][robots[robot_id].node_id];   //
                                if(eq(dist,10000))continue;
                                if(studios[material_studio_id].r_id != -1){
                                    if(lt(dist/6/0.02,product_time[studios[material_studio_id].type]))continue;
                                }
                                if(studios[material_studio_id].pStatus != 1 ){
                                    if(!check_no_send(material_studio_id))continue;
                                    if(lt(dist/6/0.02,studios[material_studio_id].r_time)){
                                        dist += (studios[material_studio_id].r_time-(dist/6/0.02))*0.02*6;
                                    }
                                }
                                dist += dis_to_studios[i][1][studios[material_studio_id].node_id];
                                income_ratio = (income/dist);
                                // if(state.FrameID>3754 &&state.FrameID<4000){
                                // cerr<<"to buy dist = "<< dis_to_studios[material_studio_id][0][robots[robot_id].node_id]<<" to send dist = "<<dis_to_studios[i][1][studios[material_studio_id].node_id]<<endl;
                                // cerr<< "robot : "<<robot_id<<" buy : "<<material_studio_id<<" type : "<<studios[material_studio_id].type<<" send : "<<i<<" type : "<< studios[i].type<<" income_ratio : "<<income_ratio<<" income = "<<income<<" dist = "<<dist<<endl;
                                if(gt(income_ratio,max)){
                                    max = income_ratio;
                                    studio_buy = material_studio_id;
                                    studio_send = i;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else if(state_type == 3){
        int item_type =robots[robot_id].get_type;
        // 只有送
        for(int i=0;i<studios.size();i++){
            for(int j=1;j<=material_send[item_type][0];j++){
                if(item_type != 7){
                    if(studios[i].type == material_send[item_type][j] && (studios_rid[i][item_type] == -1) ){
                        if(((studios[i].bitSatus & ((int)pow(2,item_type)))==0)||((check_material_full(i)&&(studios[i].pStatus != 1)&&(studios[i].r_time>0&&(checkEnough(robot_id,i,studios[i].r_time)))))){
                            dist = dis_to_studios[i][1][robots[robot_id].node_id];   //修改
                            if(eq(dist,10000))continue;
                            income = price[item_type][1]-price[item_type][0];
                            income_ratio = (income/dist);
                            if(gt(income_ratio,max)){
                                max = income_ratio;
                                studio_send = i;
                            }
                        }
                    }
                }
                else{
                    if(studios[i].type == material_send[item_type][j]){
                        dist = dis_to_studios[i][1][robots[robot_id].node_id];
                        if(eq(dist,10000))continue;
                        income = price[item_type][1]-price[item_type][0];
                        income_ratio = (income/dist);
                        if(gt(income_ratio,max)){
                            max = income_ratio;
                            studio_send = i;
                        }
                    }
                }
            }
            if(studios[i].type == 9){
                dist = dis_to_studios[i][1][robots[robot_id].node_id];
                if(eq(dist,10000))continue;
                income = price[item_type][1]-price[item_type][0];
                income_ratio = (income/dist);
                if(gt(income_ratio,max)){
                    max = income_ratio;
                    studio_send = i;
                }
            }
        }
    }
    // cerr<<" buy = "<<studio_buy<<" send = "<<studio_send<<endl;
    // cerr<<"income_ratio "<<max<<endl;
    pair<int,int> road (studio_buy,studio_send);
    return pair<pair<int,int>,double>(road,max);
}
void change_status(int robot_id,pair<pair<int,int>,double>temp){
    robots[robot_id].target_id_buy = temp.first.first;
    robots[robot_id].target_id_send = temp.first.second;
    // cerr<<" buy = "<<robots[robot_id].target_id_buy<<" send = "<<robots[robot_id].target_id_send<<endl;
    robots[robot_id].target_id = robots[robot_id].target_id_buy;
    if(robots[robot_id].target_id != -1){
        if (studios[robots[robot_id].target_id].r_id != -1 && studios[robots[robot_id].target_id].r_id != robot_id)
            studios[robots[robot_id].target_id].r_id += 50;
        else
            studios[robots[robot_id].target_id].r_id = robot_id;
        robot_get_type[studios[robots[robot_id].target_id].type]++;
        // cerr<<" studio_rid : "<<robots[robot_id].target_id_send<<" - "<<studios[robots[robot_id].target_id_buy].type<<endl;
        if(studios[robots[robot_id].target_id_send].type!=8&&studios[robots[robot_id].target_id_send].type!=9)studios_rid[robots[robot_id].target_id_send][studios[robots[robot_id].target_id_buy].type] = robot_id;
    }
}
void complete_trans(int robot_id){
    pair<pair<int,int>,double>temp;
    // cerr<<"bbb"<<endl;
    temp=new_pick_point(robot_id,2);
    // cerr<<"ccc"<<endl;
    // cerr<<" buy = "<<temp.first.first<<" send = "<<temp.first.second<<endl;
    change_status(robot_id,temp);
    // cerr<<"robot_id : "<<robot_id<<endl;
}

void charge_target(int robot_id){
    int i =robot_id;
    pair<pair<int,int>,double>temp;
    if(robots[i].get_type==0){
        temp=new_pick_point(i,2);
        double income = price[studios[robots[i].target_id_buy].type][1]-price[studios[robots[i].target_id_buy].type][0];
        double income_ratio =  income/(dis_to_studios[robots[i].target_id_buy][0][robots[i].node_id]+dis_to_studios[robots[i].target_id_send][1][studios[robots[i].target_id_buy].node_id]);
        if(lt(income_ratio,temp.second)){
            // cerr<<" robot : "<<i<<" change_target_id "<<"from "<<robots[i].target_id_buy<<" - "<<robots[i].target_id_send<<" to "<<temp.first.first<<" - "<<temp.first.second<<" income = "<<income_ratio<<" after change income = "<<temp.second<<endl;
            if (studios[robots[i].target_id].r_id >= 50)
                studios[robots[i].target_id].r_id -= 50;
            else
                studios[robots[i].target_id].r_id = -1;
            if(studios[robots[i].target_id_send].type!=8&&studios[robots[i].target_id_send].type!=9)studios_rid[robots[i].target_id_send][studios[robots[i].target_id_buy].type] = -1;
            robot_get_type[studios[robots[i].target_id].type]--;
            change_status(i,temp);
            robots[i].cnt_tar=robots[i].node_id;
        }
    }
    else{
        temp=new_pick_point(i,3);
        double income = price[robots[i].get_type][1]-price[robots[i].get_type][0];
        double income_ratio =  income/(dis_to_studios[robots[i].target_id][1][robots[robot_id].node_id]);
        if(lt(income_ratio,temp.second)){
            // cerr<<" robot : "<<i<<" change_target_id "<<"from "<<robots[i].target_id_buy<<" - "<<robots[i].target_id_send<<" to "<<temp.first.first<<" - "<<temp.first.second<<" income = "<<income_ratio<<" after change income = "<<temp.second<<endl;
            if(studios[robots[i].target_id_send].type!=8&&studios[robots[i].target_id_send].type!=9)studios_rid[robots[i].target_id_send][studios[robots[i].target_id_buy].type] = -1;
            robots[i].target_id_send = temp.first.second;
            robots[i].target_id = robots[i].target_id_send;
            robots[i].cnt_tar=robots[i].node_id;
            if(studios[robots[i].target_id_send].type!=8&&studios[robots[i].target_id_send].type!=9)studios_rid[robots[i].target_id_send][studios[robots[i].get_type].type] = i;
        }

    }
}
void new_robot_judge(){
    
    for(int i=0;i<4;i++){
        if(robots[i].loc_id == robots[i].target_id && robots[i].target_id != -1){
            if(robots[i].get_type != 0){
                //sell;
                // cerr<<"aaa"<<endl;
                ins[i].sell = 1;
                ins[i].buy = -1;
                // if(i == 1)cerr<<" ins[i].buy = "<<ins[i].buy<<" ins[i].sell = "<<ins[i].sell<<endl;
                robots[i].lastSign=0;
                robots[i].isTurn=0;
                studios[robots[i].loc_id].bitSatus += (int)pow(2,robots[i].get_type);
                studios_rid[robots[i].loc_id][robots[i].get_type] = -1;
                robots[i].get_type = 0;
                complete_trans(i);
                robots[i].cnt_tar=robots[i].node_id;
            }
            else{
                //do something buy;
                // cerr<<"bbb"<<endl;
                if(studios[robots[i].target_id].pStatus==1){
                    ins[i].buy = 1;
                    ins[i].sell = -1;
                    robots[i].lastSign=0;
                    robots[i].isTurn=0;
                    if (studios[robots[i].loc_id].r_id >= 50)
                        studios[robots[i].loc_id].r_id -= 50;
                    else
                        studios[robots[i].loc_id].r_id = -1;
                    studios[robots[i].loc_id].pStatus = 0;
                    robots[i].get_type = studios[robots[i].loc_id].type;
                    robots[i].target_id = robots[i].target_id_send;
                    robots[i].cnt_tar=robots[i].node_id;
                }
                else{
                    // cerr<<"robot: "<<i<<" wait "<<endl;
                    ins[i].buy = -1;   //wait;
                    ins[i].sell = -1;
                }
            }
        }
        else{
            // cerr<<"ccc"<<endl;
            ins[i].buy = -1;
            ins[i].sell = -1;
            
            //change_targte
            if(robots[i].target_id != -1)
                charge_target(i);
        }
        if(robots[i].target_id == -1){
            // cerr<<"ddd"<<endl;
            if(robots[i].get_type == 0){
                complete_trans(i);
                robots[i].cnt_tar=robots[i].node_id;
            }
            else{
                robots[i].target_id_send = new_pick_point(i,3).first.second;
                
                robots[i].target_id = robots[i].target_id_send;
                robots[i].cnt_tar=robots[i].node_id;
                if(studios[robots[i].target_id_send].type!=8&&studios[robots[i].target_id_send].type!=9)studios_rid[robots[i].target_id_send][studios[robots[i].get_type].type] = i;
            }
        }
        // cerr<<"robot : "<<i<<" target id = "<<robots[i].target_id<<" virtual target id = "<<robots[i].virtual_id<<" virtual target pos = "<<robots[i].virtual_pos.first<<' '<<robots[i].virtual_pos.second<<" ins[i].buy = "<<ins[i].buy<<" ins[i].sell = "<<ins[i].sell<<endl;
    }
}
void new_first_action(){
    for(int i=0;i<robots.size();i++){
        // cerr<<"aaa"<<endl;
        complete_trans(i);
        robots[i].cnt_tar=robots[i].node_id;
        // cerr<<"robot : "<<i<<" target id = "<<robots[i].target_id<<endl;
    }
}
void new_robot_action(){
    for(int i =0;i<=7;i++)robot_get_type[i]=0;
    for(int i = 0;i<4;i++){
        if(robots[i].get_type != 0)robot_get_type[robots[i].get_type]++;
        else if(robots[i].target_id != -1)robot_get_type[studios[robots[i].target_id].type]++;
    }
    // if(state.FrameID >=2200 &&state.FrameID <= 2300){
        // cerr<<" r_id "<<studios_rid[20][1]<<endl;
    // }
    new_robot_judge();
}
// void robot_action(){
//     //cerr<<"start"<<endl;
//     // print_matr();
//     // print_matr();
//     int flag = 0;
//     for(int i=0;i<4;i++){
//         if(robots[i].target_id != -1){
//             robot_area[i]=studios[robots[i].target_id].area;
//         }
//         if(robot_last_state[i][1]!=robots[i].get_type){
//             flag = 1;
//             // cerr<<"aaaa"<<endl;
//             if(robot_last_state[i][1] == 0){
//                 studios[robots[i].target_id].r_id = -1;
//             }
//             else{
//                 studios_rid[robots[i].target_id][robot_last_state[i][1]] = -1;
//             }
//             robots[i].target_id = robot_last_last_state[i][0];
//             if(robots[i].get_type == 0){
//                 studios[robot_last_last_state[i][0]].r_id = i;
//             }
//             else{
//                 studios_rid[robot_last_last_state[i][0]][robots[i].get_type]=i;
//             }
//         }
//     }
//     if (class_map == 1 || class_map == 2 || class_map ==3)
//     {
//         for(int i = 0;i<studios.size();i++){
//             if(studios[i].type>=4&&studios[i].type<=7){
//                 // if(studios[i].r_time==-1){
//                 //     studios[i].wait_time++;
//                 // }
//                 // if(studios[i].r_time>=0){
//                 //     studios[i].wait_time = 0;
//                 // }
//                 int count = 0;
//                 // cerr << studios[i].type << ' ' << studios[i].bitSatus << endl;
//                 for (int j = 1; j <= studio_material[studios[i].type - 4][0]; j++){
//                     // cerr << studio_material[studios[i].type - 4][j] << ' ' << ((int)pow(2, studio_material[studios[i].type - 4][j]))<<endl;
//                     if ((studios[i].bitSatus & ((int)pow(2, studio_material[studios[i].type - 4][j]))) == ((int)pow(2, studio_material[studios[i].type - 4][j])))
//                     {
//                         count++;
//                     }
//                 }
//                 // cerr<<count<<endl;
//                 if ((count == (studio_material[studios[i].type - 4][0] + 1)) || count == 0) studios[i].wait_time = 0;
//                 else{
//                     // studios[i].wait_time++;
//                     if (max_wait_time[studios[i].type-4] < studios[i].wait_time) max_wait_time[studios[i].type-4] = studios[i].wait_time;
//                     if (last_count[i] < count && last_count[i] >= 1)
//                         studios[i].wait_time *= 2;
//                     if (count > 0)
//                         studios[i].wait_time += count;
//                 }
//                 //  if (count > 0)studios[i].wait_time+=count;
//                 // if(studios[i].wait_time != 0)
//                     // cerr << i<<' '<<studios[i].wait_time<<endl;
//                 last_count[i]=count;
//             }
//         }
//         if(class_map==2){
//             for (int i = 0; i < studios.size(); i++)
//             {
//                 if (studios[i].wait_time > 0)
//                     studios[i].wait_time = max_wait_time[studios[i].type-4];
//             }
//         }
//     }
//     if(class_map == 4){
//         for (int i = 0; i < studios.size(); i++)
//         {
//             if (studios[i].type >= 4 && studios[i].type <= 7)
//             {
//                 if(studios[i].r_time==-1){
//                     studios[i].wait_time++;
//                 }
//                 if(studios[i].r_time>=0){
//                     studios[i].wait_time = 0;
//                 }
//                 // int count = 0;
//                 // // cerr << studios[i].type << ' ' << studios[i].bitSatus << endl;
//                 // for (int j = 1; j <= studio_material[studios[i].type - 4][0]; j++)
//                 // {
//                 //     // cerr << studio_material[studios[i].type - 4][j] << ' ' << ((int)pow(2, studio_material[studios[i].type - 4][j]))<<endl;
//                 //     if ((studios[i].bitSatus & ((int)pow(2, studio_material[studios[i].type - 4][j]))) == ((int)pow(2, studio_material[studios[i].type - 4][j])))
//                 //     {
//                 //         count++;
//                 //     }
//                 // }
//                 // // cerr<<count<<endl;
//                 // if ((count == studio_material[studios[i].type - 4][0] + 1) || count == 0)
//                 //     studios[i].wait_time = 0;
//                 // else if (count > 0)
//                 //     studios[i].wait_time++;
//                 // if(studios[i].wait_time != 0)
//                 // cerr << i<<' '<<studios[i].wait_time<<endl;
//             }
//         }
//     }
//     for(int i =0;i<=7;i++)robot_get_type[i]=0;
//     for(int i = 0;i<4;i++){
//         if(robots[i].get_type != 0)robot_get_type[robots[i].get_type]++;
//         else if(robots[i].target_id != -1)robot_get_type[studios[robots[i].target_id].type]++;
//     }
//     // for(int i =0;i<=7;i++)cerr<<"type "<<i<<" has "<<robot_get_type[i]<<endl;
//     // cerr <<endl;
//     int full = 0;
    
//     if(judge_full(2,0.01))full = 1;   //4,5,6 full threshold
//     // if (class_map == 3) full = 0;
//         // if(judge_full(3,0.2))full = 2;   //7 full threshold Higher priority
//         // cerr<<" full = "<<full<<endl;
//         // if(full!=0);
//         // robot_judge(full,1.3,4.5);
//         // cerr<<studios[0].r_id<<"aaa"<<endl;
//         robot_judge_sol(5, full);
//     for(int i=0;i<4;i++){
//         if(flag ==0){
//             robot_last_last_state[i][0]=robot_last_state[i][0];
//             robot_last_last_state[i][1]=robot_last_state[i][1];
//         }
//         robot_last_state[i][0]=robots[i].target_id;
//         robot_last_state[i][1]=robots[i].get_type;        
//     }
// }
vector<double>  get_T_limits(Robot& robot){
    double radius=0.53;
    double tmpA=robot.direction;
    auto pos=robot.pos;
    pos.first+=3*cos(tmpA);
    pos.second+=3*sin(tmpA);
    vector<double> tmp{-7,-7};
    double redundancy= 0;//冗余，避免频繁转向
    if(gt(pos.first-redundancy,0)&&lt(pos.second-redundancy,0)){//只靠近下方x轴
        tmp[0]=0;
        tmp[1]=Pi;
    }else if(lt(pos.first-redundancy,0)&&lt(pos.second-redundancy,0)){//靠近原点
        tmp[0]=0; 
        tmp[1]=Pi/2;
    }else if(lt(pos.first-redundancy,0)&&gt(pos.second-redundancy,0)){//只靠近左方的y轴
        tmp[0]=-Pi/2;
        tmp[1]=Pi/2;
    }else if(lt(pos.first-redundancy,0)&&gt(pos.second+redundancy,50)){//靠近左上角
        tmp[0]=-Pi/2;
        tmp[1]=0;
    }else if(gt(pos.first-redundancy,0)&&gt(pos.second+redundancy,50)){////靠近上方的x轴
    
        tmp[0]=-Pi;
        tmp[1]=0;
    }else if(gt(pos.first+redundancy,50)&&gt(pos.second+redundancy,50)){//靠近右上角
        tmp[0]=-Pi;
        tmp[1]=-Pi/2;
    }else if(gt(pos.first+redundancy,50)&&gt(pos.second-redundancy,0)){//靠近右边的y轴
        tmp[0]=Pi/2;
        tmp[1]=Pi;
        tmp.push_back(-Pi);
        tmp.push_back(-Pi/2);
    }else if(gt(pos.first+redundancy,50)&&lt(pos.second-redundancy,0)){//靠近右下角
        tmp[0]=Pi/2;
        tmp[1]=Pi;
    }
    return tmp;    
}
vector<double>  get_T_limits(pair<double,double>pos,const Robot& robot,int ctr,double dis){
    double radius=robot.get_type==0? 0.45:0.53;
    double tmpA=robot.direction;
    int sign1=ge(robot.xy_pos.first,0)?1:-1;
    int sign2=ge(robot.xy_pos.second,0)?1:-1;
    dis=fabs(dis);
    pos.first+=dis*cos(tmpA);
    pos.second+=dis*sin(tmpA);
    // if(state.FrameID>=591&&state.FrameID<=603&&robot.id==2){
    //     cerr<<endl<<robot.xy_pos.first<<" +  "<<robot.xy_pos.second<<endl;
    //     cerr<<endl<<pos.first<<" "<<pos.second<<endl;
    // }
    vector<double> tmp{-7,-7};
    double redundancy= (ctr==-1?0.2:dis)+radius;//冗余，避免频繁转向
    if(gt(pos.first-redundancy,0)&&lt(pos.second-redundancy,0)){//只靠近下方x轴
        tmp[0]=0;
        tmp[1]=Pi;
    }else if(lt(pos.first-redundancy,0)&&lt(pos.second-redundancy,0)){//靠近原点
        tmp[0]=0; 
        tmp[1]=Pi/2;
    }else if(lt(pos.first-redundancy,0)&&gt(pos.second-redundancy,0)){//只靠近左方的y轴
        tmp[0]=-Pi/2;
        tmp[1]=Pi/2;
    }else if(lt(pos.first-redundancy,0)&&gt(pos.second+redundancy,50)){//靠近左上角
        tmp[0]=-Pi/2;
        tmp[1]=0;
    }else if(gt(pos.first-redundancy,0)&&gt(pos.second+redundancy,50)){////靠近上方的x轴
    
        tmp[0]=-Pi;
        tmp[1]=0;
    }else if(gt(pos.first+redundancy,50)&&gt(pos.second+redundancy,50)){//靠近右上角
        tmp[0]=-Pi;
        tmp[1]=-Pi/2;
    }else if(gt(pos.first+redundancy,50)&&gt(pos.second-redundancy,0)){//靠近右边的y轴
        tmp[0]=Pi/2;
        tmp[1]=Pi;
        tmp.push_back(-Pi);
        tmp.push_back(-Pi/2);
    }else if(gt(pos.first+redundancy,50)&&lt(pos.second-redundancy,0)){//靠近右下角
        tmp[0]=Pi/2;
        tmp[1]=Pi;
    }
    return tmp;
}
bool can_stop(pair<double,double>p1,pair<double,double>p2,double angle,bool isWall){
    if(gt(angle,Pi/2))return false;
    if(lt(angle,0.08))return true;
    double dis=calcuDis(p1,p2);
    if(lt(sin(angle)*dis,0.1)){
        return true;
    }
    return false;

}
double return_cal(pair<double,double>p1,pair<double,double>p2,double angle){
    double dis=calcuDis(p1,p2);
    // cerr<<dis<<"--"<<sin(angle)*dis<<endl;
    return sin(angle)*dis-0.4;
}
bool is_range(double dire,vector<double>&tmp){
    if(tmp.size()==2){
        if(ge(dire,tmp[0])&&le(dire,tmp[1])){
            return true;
        }else{
            return false;
        }
    }else{
        if((ge(dire,tmp[0])&&le(dire,tmp[1]))||
        (ge(dire,tmp[2])&&le(dire,tmp[3]))){
            return true;
        }else{
            return false;
        } 
    }
    
}
double get_dis(pair<double, double> P, Line l) { 
    auto get_v=[&](pair<double, double> P,pair<double, double> v)->double{
        return P.first*v.second-v.first*P.second;
    };
    double distance=abs(get_v(P,l.v)-get_v(l.P,l.v))/(sqrt(l.v.first * l.v.first  + 
    l.v.second* l.v.second));
    return distance; 
}
bool can_speed_z(int stuID,pair<double,double>xy_pos,pair<double,double>pos,double acceleration ){
    Line line;
    line.v=xy_pos;
    line.P=pos;
    double totalV=sqrt(xy_pos.first*xy_pos.first+xy_pos.second*xy_pos.second);//合速度
    double dis1=get_dis(studios[stuID].pos,line);//点到直线的距离
    double dis2=calcuDis(studios[stuID].pos,pos);//点之间的距离
    double dis3=totalV*totalV/(2*acceleration);//速度减为0的滑行距离
    double dis4=sqrt(0.4*0.4-dis1*dis1);//圆截线的长度
    double dis5=sqrt(dis2*dis2-dis1*dis1);//射线的长度
    //cerr<<stuID<<" "<<dis3<<" "<<dis4<<" "<<dis5<<" "<<dis1<<endl;
    double rudi=0.08;
    if(class_map==2){
        rudi=0.1;
    }
    // if(contr_print_flag&&state.FrameID>=1840&&state.FrameID<=1870&&stuID==17&&pos.second<4){
    //             cerr<<"dis: "<<endl;
    //             cerr<<dis3<<" "<<dis5-dis4<<endl;
    // }
    if(gt(dis3,(dis5-dis4)+rudi))return true;
    return false;
}
bool isWall(int stuID){
    double i=studios[stuID].pos.first;
    double j=studios[stuID].pos.second;
    if(i-1<=0||j-2<=0||i+2>=50||j+2>=50)return true;
    return false;
}
bool isWall_r(const Robot& robot){
    double i=robot.pos.first;
    double j=robot.pos.second;
    double ridr=0.8+(robot.get_type==0?0.6:0.63);
    // if(state.FrameID>=1005&&state.FrameID<=1010&&robot.id==2){
    //     cerr<<(j)<<endl;
    // }
    if(le(i-ridr,0)||le(j-ridr,0)||ge(i+ridr,50)||ge(j+ridr,50))return true;
    return false;   
}
bool will_impact(const Robot& robot,double dis){
    vector<double> tmp=get_T_limits(robot.pos,robot,1,dis);
    if(!eq(tmp[0],-7)&&(!is_range(robot.direction,tmp)))
    {//在墙附件，并且会撞上
        return true;
    }
    return false;
}
int special_test(int i1,int i2){
    int cnt=5;
    double base=0.02;
    double radius=robots[i1].get_type==0? 0.45:0.53;
    for(int i=1;i<=cnt;i++){
        double time=i*base;
        auto p1=make_pair<double,double>(robots[i1].pos.first+robots[i1].xy_pos.first*time,
        robots[i1].pos.second+robots[i1].xy_pos.second*time
        );
        auto p2=make_pair<double,double>(robots[i2].pos.first+robots[i2].xy_pos.first*time,
        robots[i2].pos.second+robots[i2].xy_pos.second*time
        );
        double dis=calcuDis(p1,p2);   
        if(lt(dis,radius*2))return i;     
    }
    return 0;
}
double get_angle(double s1,double s2){
    Vec v1(make_pair<double ,double>(cos(s1),sin(s1)));
    Vec v2(make_pair<double ,double>(cos(s2),sin(s2)));
    return (cos_t(v1,v2));
}
double get_angle(pair<double,double> p1,pair<double,double> p2){
    Vec v1(p1);
    Vec v2(p2);
    return cos_t(v1,v2);
}
double get_angle_1(double s1,double s2){
    Vec v1(make_pair<double ,double>(cos(s1),sin(s1)));
    Vec v2(make_pair<double ,double>(cos(s2),sin(s2)));
    return acos(cos_t(v1,v2));
}
double get_angle_1(pair<double,double> p1,pair<double,double> p2){
    Vec v1(p1);
    Vec v2(p2);
    return acos(cos_t(v1,v2)) ;
}
bool is_less(int i1,int i2){
        double base=0.02;
        double time=2*base;
        auto p1=make_pair<double,double>(robots[i1].pos.first+robots[i1].xy_pos.first*time,
        robots[i1].pos.second+robots[i1].xy_pos.second*time
        );
        auto p2=make_pair<double,double>(robots[i2].pos.first+robots[i2].xy_pos.first*time,
        robots[i2].pos.second+robots[i2].xy_pos.second*time
        );
        double dis=calcuDis(p1,p2);   

        auto p3=make_pair<double,double>(robots[i1].pos.first+robots[i1].xy_pos.first,
        robots[i1].pos.second+robots[i1].xy_pos.second
        );
        auto p4=make_pair<double,double>(robots[i2].pos.first+robots[i2].xy_pos.first,
        robots[i2].pos.second+robots[i2].xy_pos.second
        );
        double dis1=calcuDis(p3,p4);   
    
        return lt(dis,dis1);
}
bool who_isFirst(int i1,int i2){
    pair<double, double> p1 = subVector(robots[i1].pos, robots[i2].pos);
    double s1=robots[i1].direction;
    pair<double, double> p2=make_pair<double ,double>(cos(s1),sin(s1));
    double tmpCos=get_angle(p1,p2);
    return gt(tmpCos,0.0);
}
double return_v(int id){
    auto xy_pos=robots[id].xy_pos;
    return sqrt(xy_pos.first*xy_pos.first+xy_pos.second*xy_pos.second);//合速度
}
double return_v(Robot rob){
    auto robot=rob;
    return calVectorSize(robot.xy_pos) * (ge(calVectorProduct(robot.xy_pos, transformVector(robot.direction)), 0.0)? 1: -1);
}
int Calculate_root(int i1,int i2){
    double tmp= get_angle(robots[i1].xy_pos,robots[i2].xy_pos);
    if(gt(tmp,0.9)&&!will_collision(i1,i2))return -1;
    else return 0;
}
bool will_collision(int i1,int i2,int ctr){
    Vec v1(robots[i1].xy_pos);
    Vec v2(robots[i2].xy_pos);
    Vec x1(robots[i1].pos);
    Vec x2(robots[i2].pos);
    Vec c_t=x1-x2;
    Vec v_t=v1-v2;
    bool f1=isWall(robots[i1].target_id);
    bool f2=isWall(robots[i2].target_id);
    double tmpDis=calcuDis(robots[i1].pos,robots[i2].pos);
    if(gt(tmpDis,2)){
            Compute_redundancy=1.0;
    }else{
            Compute_redundancy=0.0;
        }
    double r=getRobotRadius(i1)+getRobotRadius(i2)+Compute_redundancy;
    double a=v_t*v_t;
    double b=2*(v_t*c_t);
    double c=c_t*c_t-r*r;
    double cla=b*b-4*a*c;   
    if(state.FrameID>=6830&&state.FrameID<=6840&&i1==1&&i2==2){

        // cerr<<"will_c: "<<(-1*b+sqrt(cla))/(2*a)<<" "<<b<<" "<<(-1*b-sqrt(cla))/(2*a)<<" "<<cla<<endl;
        // cerr<<"dis: "<<tmpDis<<"  v "<<return_v(i1)<<" "<<return_v(i2)<<endl;
    }
    pair<double ,double>tmp(-7,-7);
    if(cla<0){
        RootFlag=-1;
        return false;
    }
    if(eq(cla,0)){
        RootFlag=0;
        tmp.first=-1*b/(2*a);
        pair<double,double>pos_tmp_r1=robots[i1].pos;
        pair<double,double>pos_tmp_r_c(pos_tmp_r1.first+robots[i1].xy_pos.first*tmp.first,
        pos_tmp_r1.second+robots[i1].xy_pos.second*tmp.first
        );
        pair<double,double>pos_tmp_s1=studios[robots[i1].target_id].pos;
        double dis1=calcuDis(pos_tmp_r1,pos_tmp_s1);
        double dis3=calcuDis(pos_tmp_r1,pos_tmp_r_c);
        Collision_point=pos_tmp_r_c;
        pair<double,double>pos_tmp_r2=robots[i2].pos;
        pair<double,double>pos_tmp_s2=studios[robots[i2].target_id].pos;
        double dis2=calcuDis(pos_tmp_r2,pos_tmp_s2);
        double dis4=calcuDis(pos_tmp_r2,pos_tmp_r_c);
        Root=tmp;
        if(!f1&&!f2){
            if(gt(tmp.first,0.0)){
                if(lt(tmp.first,0.3))
                return true;
            }else{
            auto t1=Root;
            auto t2=RootFlag;
            if(will_collision_Careful(i1,i2)){
                
                return true;
            }
            Root=t1;
            RootFlag=t2;
            }
        }
        if(gt(tmp.first,0.0)){
            if(le(dis3,dis1)&&le(dis4,dis2))return true;
        }else{
            auto t1=Root;
            auto t2=RootFlag;
            if(will_collision_Careful(i1,i2)){
                
                return true;
            }
            Root=t1;
            RootFlag=t2;
        }
        
        return false;

    }else if(gt(cla,0)){
        RootFlag=1;
        tmp.first=(-1*b+sqrt(cla))/(2*a);
        tmp.second=(-1*b-sqrt(cla))/(2*a);
        Root=tmp;
        if(!f1&&!f2){
            if(gt(tmp.first,0.0)){
                if(lt(tmp.first,0.3))
                return true;
            }else{
                auto t1=Root;
                auto t2=RootFlag;
                if(will_collision_Careful(i1,i2)){
                    
                    return true;
                }
                Root=t1;
                RootFlag=t2;
            }
            if(gt(tmp.second,0.0)){
                if(lt(tmp.second,0.3))
                return true;
            }else{
                auto t1=Root;
                auto t2=RootFlag;
                if(will_collision_Careful(i1,i2)){
                    
                    return true;
                }
                Root=t1;
                RootFlag=t2;
            }
        }
 
        pair<double,double>pos_tmp_r1=robots[i1].pos;
        pair<double,double>pos_tmp_r_c(pos_tmp_r1.first+robots[i1].xy_pos.first*tmp.first,
        pos_tmp_r1.second+robots[i1].xy_pos.second*tmp.first
        );
        pair<double,double>pos_tmp_s1=studios[robots[i1].target_id].pos;
        double dis1=calcuDis(pos_tmp_r1,pos_tmp_s1);
        double dis3=calcuDis(pos_tmp_r1,pos_tmp_r_c);

        pair<double,double>pos_tmp_r2=robots[i2].pos;
        pair<double,double>pos_tmp_s2=studios[robots[i2].target_id].pos;
        double dis2=calcuDis(pos_tmp_r2,pos_tmp_s2);
        double dis4=calcuDis(pos_tmp_r2,pos_tmp_r_c);
        // if(state.FrameID==5909&&i1==3&&i2==1){
        //     cerr<<"---\n"<<endl;
        //     cerr<<robots[i1].pos.first<<"-"<<robots[i1].pos.second<<" "
        //     <<robots[i2].pos.first<<"-"<<robots[i2].pos.second<<endl;
        //     cerr<<robots[i1].xy_pos.first<<"-"<<robots[i1].xy_pos.second<<" "
        //     <<robots[i2].xy_pos.first<<"-"<<robots[i2].xy_pos.second<<endl;
        //     cerr<<v_t.x<<"-"<<v_t.y<<endl;
        //     cerr<<c_t.x<<"-"<<c_t.y<<endl;
        //     cerr<<a<<"-"<<b<<"-"<<c<<" "<<sqrt(cla)<<" "<<(-1*b+sqrt(cla))<<" "<<
        //     (-1*b+sqrt(cla))/(2*a)<<endl;
        //     cerr<<tmp.first<<"-"<<tmp.second<<endl;
        //     cerr<<dis1<<" "<<dis2<<" "<<dis3<<" "<<dis4<<" "<<pos_tmp_r_c.first<<"-"<<pos_tmp_r_c.second<<endl;
        //     cerr<<"---\n"<<endl;
        // }
        Collision_point=pos_tmp_r_c;

        if(gt(tmp.first,0.0)){
            if(le(dis3,dis1)&&le(dis4,dis2))return true;
        }else{
            auto t1=Root;
            auto t2=RootFlag;
            if(will_collision_Careful(i1,i2)){
                
                return true;
            }
            Root=t1;
            RootFlag=t2;
        }
        

        tmp.second=(-1*b-sqrt(cla))/(2*a);
        
        pair<double,double>pos_tmp_r_c1(pos_tmp_r1.first+robots[i1].xy_pos.first*tmp.second,
        pos_tmp_r1.second+robots[i1].xy_pos.second*tmp.second
        );
        double dis11=calcuDis(pos_tmp_r1,pos_tmp_s1);
        double dis31=calcuDis(pos_tmp_r1,pos_tmp_r_c1);


        double dis21=calcuDis(pos_tmp_r2,pos_tmp_s2);
        double dis41=calcuDis(pos_tmp_r2,pos_tmp_r_c1);
        Collision_point=pos_tmp_r_c1;
        if(gt(tmp.second,0.0)){
            if(le(dis31,dis11)&&le(dis41,dis21))return true;
        }else{
            auto t1=Root;
            auto t2=RootFlag;
            if(will_collision_Careful(i1,i2)){
                
                return true;
            }
            Root=t1;
            RootFlag=t2;
        }
        
        return false;       
    }
    if(eq(a,0))return true;
    return true;
}
bool return_collision(int i1,int i2){
    return lt(robots[i1].collision_val_pre,robots[i1].collision_val)&&
    lt(robots[i2].collision_val_pre,robots[i2].collision_val);
}
pair<int,int> far_away(int i1,int i2,int base1,int base2){
    int sign1=payloads[i1].sign,sign2=payloads[i2].sign;
    if(sign1*sign2<0) return pair<int,int> (sign1,sign2);
    else{
        if(gt(fabs(payloads[i1].angle)-fabs(payloads[i2].angle) ,2)||robots[i1].get_type>(robots[i2].get_type)){
            return pair<int,int> (sign1,-1*sign2);
        }
        return pair<int,int> (-1*sign1,sign2);
    }
    double dis = calcuDis(robots[i1].pos, robots[i2].pos);
    int arr[][2]{{-1*base1,1*base2},{1*base1,1*base2},{-1*base1,-1*base2},{1*base1,-1*base2}};
    double time=0.02;
    pair<int,int>tmp(0,0);
    pair<double,double>pre_xy1=robots[i1].xy_pos;
    pair<double,double>pre_xy2=robots[i2].xy_pos;
    double mmax=0.0;
    int pos=0;
    for(int i=0;i<4;i++){
        Vec v1(make_pair<double ,double>(cos(Pi/2),sin(Pi/2)));
        Vec v2(make_pair<double ,double>(cos(Pi/2),sin(Pi/2)));
        auto p1=make_pair<double,double>(robots[i1].pos.first+robots[i1].xy_pos.first+(time*v1.x*arr[i][0]),
        robots[i1].pos.second+robots[i1].xy_pos.second+(time*v1.x*arr[i][0])
        );
        auto p2=make_pair<double,double>(robots[i2].pos.first+robots[i2].xy_pos.first+(time*v2.x*arr[i][1]),
        robots[i2].pos.second+robots[i2].xy_pos.second+(time*v2.x*arr[i][1])
        );
        double dis=calcuDis(p1,p2);    
        if(gt(dis,mmax)){
            mmax=dis;
            pos=i;
        }

    }
    tmp.first=arr[pos][0];
    tmp.second=arr[pos][1];
    return tmp;
}
double return_maxAng(int id1){
    double dis=calcuDis(robots[id1].pos,studios[robots[id1].target_id].pos);
    if(lt(dis,0.4))return Pi;
    return asin(0.4/dis);
}
bool Check_for_balls_around(int pos){
    for(int i=0;i<4;i++){
        double dis = calcuDis(robots[i].pos, robots[pos].pos);
        if(gt(dis,3))return false;
       
    }
    return true;
}
int return_line_dire(int i1,int i2,int signBase){
    
    int try_aginF=0;
    int tarId1=robots[i1].target_id==-1?0:robots[i1].target_id;
    int tarId2=robots[i2].target_id==-1?0:robots[i2].target_id;
    bool l1=can_stop(robots[i1].pos,studios[robots[i1].target_id].pos,payloads [i1].angle,isWall(tarId1));
    bool l2=can_stop(robots[i2].pos,studios[robots[i2].target_id].pos,payloads [i2].angle,isWall(tarId2));
    double tmpDis=calcuDis(robots[i1].pos,robots[i2].pos);
    // if(l1&&l2)
    // will_collision(i1,i2,0);
    // else
    will_collision(i1,i2);
    try_agin:
    int flagSign=getSign(i1,i2);
    // double canAngle=min(fabs(Root.first),fabs(Root.second))*40*0.3;
    // double stop_time= (fabs(robots[i2].angular_velocity))/(payloads[i2].angular_acceleration);
    // double subVal=stop_time*40*0.36;
    double real_time=-8;
    // if(gt(tmpDis,2))
    real_time=will_Collo_new(i1,i2);
    // else
    // real_time=min(fabs(gt(Root.first,0)?Root.first:7),fabs(gt(Root.second,0)?Root.second:7));
    // if(lt(real_time,0)){
    //     return 0;
    // }
    double canAngle_neg=get_at_v(real_time,payloads[i2].angular_acceleration
    ,robots[i2].angular_velocity,-1);
    double canAngle_pos=get_at_v(real_time,payloads[i2].angular_acceleration
    ,robots[i2].angular_velocity,1);
    auto tmp= subVector(robots[i1].pos, robots[i2].pos);
    Vec v2(robots[i2].xy_pos); 
    Vec v1(tmp);
    int sign= (lt(v1^v2,0))?-1:1;
    auto angle= return_seta(i1,i2); 
    double seta=angle.first;
    double arf=angle.second;
    double canAngle_pos_z=get_at_v_z(real_time,payloads[i2].angular_acceleration
    ,robots[i2].angular_velocity,sign)*-1;
    double canAngle_neg_z=get_at_v_z(real_time,payloads[i2].angular_acceleration
    ,robots[i2].angular_velocity,sign*-1)*-1;
    // if(lt(canAngle_neg,0.0)){
    //     cerr<<"----------+ "<<canAngle_neg<<" "<<canAngle_pos<<" "<<
    //      payloads[i2].angular_acceleration<<" "<<sign<<endl; 
    //      cerr<<robots[i2].angular_velocity<<" "<<real_time<< endl;
    // }
    // if(state.FrameID>=2048&&state.FrameID<=2500&&i1==2&&i2==1){
    //     cerr<<"Frame: "<<state.FrameID<<" "<<canAngle_neg_z<<" "<<canAngle_pos_z<<" "<<arf<<
    //     " "<<robots[i2].angular_velocity<< " "<<sign<<endl;
       
    // }
    double v_1=min(return_v(i1),ins[i1].forward);
    double v_2=min(return_v(i2),ins[i2].forward);
    int total=robots[i1].get_type+robots[i2].get_type;
    if(flagSign==1){
        bool f1=false,f2=false;
        if(gt(sign*-1==-1?canAngle_neg:canAngle_pos,seta+arf)){
            f2=true;
            // return sign*-1;
        }else if(lt(Pi-seta-arf,sign==-1?canAngle_neg:canAngle_pos)){
            f1=true;
            // return sign;
        }else if(lt(fabs(Pi-seta-arf)+canAngle_pos_z,fabs(seta+arf)+canAngle_neg_z)){
            // cerr<<"can't raote -"<<state.FrameID<<" "<<i1<<" "<<i2<<endl;
            // cerr<<"can't raote  angle -"<<canAngle_pos_z<<" "<<canAngle_neg_z<<" "<<sign<<endl;
            //ins[i1].rotate=Pi*-1*sign;
            // cerr<<"0"<<endl;
            return sign;
        }else{
            // cerr<<"can't raote -"<<state.FrameID<<" "<<i1<<" "<<i2<<endl;
            // cerr<<seta<<" "<<arf<<endl;
            // cerr<<"can't raote  angle -"<<canAngle_pos_z<<" "<<canAngle_neg_z<<" "<<sign*-1<<endl;
            // cerr<<"1"<<endl;
            return sign*-1;
        }
        if(f1&&f2){
            // return signBase;
            if(lt(fabs(Pi-seta-arf)+canAngle_pos_z,fabs(seta+arf)+canAngle_neg_z)){
                return sign;
            }else{
                return sign*-1;
            }
        }else if(f1){
            return sign;
        }else{
            return sign*-1;
        }
    }else{
        bool f1=false,f2=false;
         if(gt(sign==-1?canAngle_neg:canAngle_pos,seta-arf)){
            f1=true;
            // return sign;
        }else if(gt(sign*-1==-1?canAngle_neg:canAngle_pos,Pi-seta+arf)){
            f2=true;
            // return sign*-1;
        }else if(lt(fabs(seta-arf)+canAngle_pos_z,fabs(Pi-seta+arf)+canAngle_neg_z)){
            // cerr<<"can't raote "<<state.FrameID<<" "<<i1<<" "<<i2<<endl;
            // cerr<<"can't raote  angle "<<canAngle_pos_z<<" "<<canAngle_neg_z<<" "<<sign<<endl;
            return sign;
        }else{
            // cerr<<"can't raote "<<state.FrameID<<" "<<i1<<" "<<i2<<endl;
            // cerr<<"can't raote  angle "<<canAngle_pos_z<<" "<<canAngle_neg_z<<" "<<sign*-1<<endl;
            return sign*-1;
        }  
        if(f1&&f2){
            // return signBase;
            if(lt(fabs(seta-arf)+canAngle_pos_z,fabs(Pi-seta+arf)+canAngle_neg_z)){
                return sign;
            }else{
                return sign*-1;
            }
        }else if(f1){
            return sign;
        }else{
            return sign*-1;
        }
    } 
}
int return_line_dire(int i1,int i2){
    will_collision(i1,i2);
    auto tmp= subVector(robots[i1].pos, robots[i2].pos);
    Vec v1(tmp);
    Vec v2(robots[i2].xy_pos);
    int sign= (lt(v1^v2,0))?-1:1;
    
    return sign;
}
pair<double,bool>  return_int_dis(int base){
    vector<int>arr(2);
    int pos=0;
    for(int i=0;i<4;i++){
        if((base>>i)&1){
            arr[pos++]=i;
        }
    }
    if(base==12&&calcuDis(robots[arr[0]].pos,robots[arr[1]].pos)<2&&!will_collision(arr[0],arr[1])){
                // cerr<<"% "<<RootFlag<<" "<< Root.first<<"-"<<Root.second<<
                // " "<<Collision_point.first<<"-"<<Collision_point.second<< endl;
    }
    return pair<double,int>(calcuDis(robots[arr[0]].pos,robots[arr[1]].pos),
    will_collision(arr[0],arr[1])
    );
}




double get_rotation(int i1,int i2){
    Vec v1(pair<double,double>(cos(robots[i1].direction),sin(robots[i1].direction)));
    Vec v2(subVector(robots[i1].pos, robots[i2].pos));
    return acos(cos_t(v1,v2));
}
int addSign(int i1,int i2,int baseSign){
    Vec v1(pair<double,double>(cos(robots[i1].direction),sin(robots[i1].direction)));
    Vec v2(subVector(robots[i1].pos, robots[i2].pos)); 
    int sign= (lt(v1^v2,0))?-1:1;
    return gt(sign*baseSign,0.0)?-1:1;  
}
int getSign(int i1,int i2){
    auto tmp= subVector(robots[i1].pos, robots[i2].pos);
    Vec v1(tmp);
    Vec v2(robots[i2].xy_pos);
    int sign1=(lt(v1^v2,0))?-1:1;
    auto tmp1= subVector(robots[i2].pos, robots[i1].pos);
    Vec v3(tmp1);
    Vec v4(robots[i1].xy_pos);

    int sign2=(lt(v3^v4,0))?-1:1;
    return gt(sign1*sign2,0.0)?-1:1;
}
pair<double,double> return_seta(int i1,int i2){
    auto tmp= subVector(robots[i1].pos, robots[i2].pos);
    Vec v1(tmp);
    Vec v2(robots[i2].xy_pos);
    int sign1=(lt(v1^v2,0))?-1:1;
    auto tmp1= subVector(robots[i2].pos, robots[i1].pos);
    Vec v3(tmp1);
    Vec v4(robots[i1].xy_pos);
    double tmpAngle=acos(cos_t(v1,v2));
    double tmpAngle1=acos(cos_t(v3,v4));
    return pair<double,double>(tmpAngle,tmpAngle1);
}
bool will_collision_Careful(int i1,int i2){
    Vec v1(robots[i1].xy_pos);
    Vec v2(robots[i2].xy_pos);
    Vec x1(robots[i1].pos);
    Vec x2(robots[i2].pos);
    Vec c_t=x1-x2;
    Vec v_t=v1-v2;
    bool f1=isWall(robots[i1].target_id);
    bool f2=isWall(robots[i2].target_id);
    double tmpDis=calcuDis(robots[i1].pos,robots[i2].pos);

    double r=getRobotRadius(i1)+getRobotRadius(i2);
    double a=v_t*v_t;
    double b=2*(v_t*c_t);
    double c=c_t*c_t-r*r;
    double cla=b*b-4*a*c;   
    pair<double ,double>tmp(-7,-7);
    if(cla<0){
        RootFlag=-1;
        return false;
    }
        if(state.FrameID>=6830&&state.FrameID<=6840&&i1==1&&i2==2){

        // cerr<<"will_c_c: "<<(-1*b+sqrt(cla))/(2*a)<<" "<<b<<" "<<(-1*b-sqrt(cla))/(2*a)<<" "<<cla<<endl;
        // cerr<<"dis: "<<tmpDis<<"  v "<<return_v(i1)<<" "<<return_v(i2)<<endl;
    }
    if(eq(cla,0)){
        RootFlag=0;
        tmp.first=-1*b/(2*a);
        pair<double,double>pos_tmp_r1=robots[i1].pos;
        pair<double,double>pos_tmp_r_c(pos_tmp_r1.first+robots[i1].xy_pos.first*tmp.first,
        pos_tmp_r1.second+robots[i1].xy_pos.second*tmp.first
        );
        pair<double,double>pos_tmp_s1=studios[robots[i1].target_id].pos;
        double dis1=calcuDis(pos_tmp_r1,pos_tmp_s1);
        double dis3=calcuDis(pos_tmp_r1,pos_tmp_r_c);
        Collision_point=pos_tmp_r_c;
        pair<double,double>pos_tmp_r2=robots[i2].pos;
        pair<double,double>pos_tmp_s2=studios[robots[i2].target_id].pos;
        double dis2=calcuDis(pos_tmp_r2,pos_tmp_s2);
        double dis4=calcuDis(pos_tmp_r2,pos_tmp_r_c);
        Root=tmp;
        if(!f1&&!f2){
            if(gt(tmp.first,0.0)&&lt(tmp.first,0.3))return true;
        }
        if(gt(tmp.first,0.0))
        if(le(dis3,dis1)&&le(dis4,dis2))return true;
        return false;

    }else if(gt(cla,0)){
        RootFlag=1;
        tmp.first=(-1*b+sqrt(cla))/(2*a);
        tmp.second=(-1*b-sqrt(cla))/(2*a);
        Root=tmp;
        if(!f1&&!f2){
            if(gt(tmp.first,0.0)&&lt(tmp.first,0.3))return true;
            if(gt(tmp.second,0.0)&&lt(tmp.second,0.3))return true;
        }
        pair<double,double>pos_tmp_r1=robots[i1].pos;
        pair<double,double>pos_tmp_r_c(pos_tmp_r1.first+robots[i1].xy_pos.first*tmp.first,
        pos_tmp_r1.second+robots[i1].xy_pos.second*tmp.first
        );
        pair<double,double>pos_tmp_s1=studios[robots[i1].target_id].pos;
        double dis1=calcuDis(pos_tmp_r1,pos_tmp_s1);
        double dis3=calcuDis(pos_tmp_r1,pos_tmp_r_c);

        pair<double,double>pos_tmp_r2=robots[i2].pos;
        pair<double,double>pos_tmp_s2=studios[robots[i2].target_id].pos;
        double dis2=calcuDis(pos_tmp_r2,pos_tmp_s2);
        double dis4=calcuDis(pos_tmp_r2,pos_tmp_r_c);
        // if(state.FrameID==1354&&i1==0&&i2==3){
        //     cerr<<"---\n"<<endl;
        //     cerr<<robots[i1].pos.first<<"-"<<robots[i1].pos.second<<" "
        //     <<robots[i2].pos.first<<"-"<<robots[i2].pos.second<<endl;
        //     cerr<<robots[i1].xy_pos.first<<"-"<<robots[i1].xy_pos.second<<" "
        //     <<robots[i2].xy_pos.first<<"-"<<robots[i2].xy_pos.second<<endl;
        //     cerr<<v_t.x<<"-"<<v_t.y<<endl;
        //     cerr<<c_t.x<<"-"<<c_t.y<<endl;
        //     cerr<<a<<"-"<<b<<"-"<<c<<" "<<sqrt(cla)<<" "<<(-1*b+sqrt(cla))<<" "<<
        //     (-1*b+sqrt(cla))/(2*a)<<endl;
        //     cerr<<tmp.first<<"-"<<tmp.second<<endl;
        //     cerr<<dis1<<" "<<dis2<<" "<<dis3<<" "<<dis4<<" "<<pos_tmp_r_c.first<<"-"<<pos_tmp_r_c.second<<endl;
        //     cerr<<"---\n"<<endl;
        // }
        Collision_point=pos_tmp_r_c;

        if(gt(tmp.first,0.0))
        if(le(dis3,dis1)&&le(dis4,dis2))return true;

        tmp.second=(-1*b-sqrt(cla))/(2*a);
        
        pair<double,double>pos_tmp_r_c1(pos_tmp_r1.first+robots[i1].xy_pos.first*tmp.second,
        pos_tmp_r1.second+robots[i1].xy_pos.second*tmp.second
        );
        double dis11=calcuDis(pos_tmp_r1,pos_tmp_s1);
        double dis31=calcuDis(pos_tmp_r1,pos_tmp_r_c1);


        double dis21=calcuDis(pos_tmp_r2,pos_tmp_s2);
        double dis41=calcuDis(pos_tmp_r2,pos_tmp_r_c1);
        Collision_point=pos_tmp_r_c1;
        if(gt(tmp.second,0.0))
        if(le(dis31,dis11)&&le(dis41,dis21))return true;
        return false;       
    }
    if(eq(a,0))return true;
    return true;    
}
double return_type(int i1){
    return robots[i1].collision_val*robots[i1].time_val;
}
void change_getType(){
    // for(int i=0;i<4;i++){
    //     double val=(eq(robots[i].collision_val,0)?1:robots[i].collision_val)*(eq(robots[i].time_val,0)?1:robots[i].time_val);
    //     if(lt(val,0.8))
    //     robots[i].get_type=0;
    // }
}
double return_ac(double a,double v,double v1){
    int si1=ge(v,0)?1:-1;
    int si2=ge(v1,0)?1:-1;
    //a*=lt(si1*si2,0)||gt(si1*si2,0)&&gt(fabs(v),fabs(v1));
    // if(state.FrameID==2962){
    //     cerr<<v<<":"<<si1<<endl;
    //     cerr<<v1<<":"<<si2<<endl;
    //     cerr<<(eq(v,v1))<<endl;
    // }
    if(eq(v,v1)){
        return 0;
    }if(lt(si1*si2,0)){
        a*=si2;
    }else if(gt(si1*si2,0)&&gt(fabs(v),fabs(v1))){
        a*=si2*-1;
    }else{
        a*=si2;
    }
    return a;    
}
double get_at_v_limt(double t,double a,double v,double v1,int sign_v1){
    double lef_time=0;
    double s=0;
    int si1=ge(v,0)?1:-1;
    int si2=ge(v1,0)?1:-1;
    // a*=lt(si1*si2,0)||gt(si1*si2,0)&&gt(fabs(v),fabs(v1));
    if(lt(si1*si2,0)){
        a*=si2;
    }else if(gt(si1*si2,0)&&gt(fabs(v),fabs(v1))){
        a*=si2*-1;
    }else{
        a*=si2;
    }
   
    double tmpTime=(v1-v)/(a);
    double realTime=min(tmpTime,t);
    s=v*realTime+a*realTime*realTime;
    // if(state.FrameID==1){
    //     cerr<<a<<" ^ "<<" "<<v<<" "<<tmpTime<<" "<<realTime<<" "<<s<<endl;
    // }
    double res=(s);
    if(le(t,tmpTime)){
        if(gt(res*sign_v1,0)){
            return fabs(res);
        }
        else{
            return -1*fabs(res);
        }
            return (s);
    }
    t=t-realTime;
    res=(s+v1*t);
    if(gt(res*sign_v1,0)){
        return fabs(res);
    }else{
        return -1*fabs(res);
    }
    return (s+v1*t);    
}
double get_at_v(double t,double a,double v,int sign_v1){
    double lef_time=0;
    double s=0;
    a*=sign_v1;
    if(lt(fabs(v),Pi)){
        double tmpTime=(sign_v1*Pi-v)/(a);
        double realTime=min(tmpTime,t);
        s=v*realTime+0.5*a*realTime*realTime;
        double res=(s);
        if(le(t,tmpTime)){
            if(gt(res*sign_v1,0)){
                return fabs(res);
            }
            else{
                return -1*fabs(res);
            }
            return (s);
        }
        t=t-realTime;
    }
    double res=(s+sign_v1*Pi*t);
    if(gt(res*sign_v1,0)){
        return fabs(res);
    }else{
        return -1*fabs(res);
    }
    return (s+sign_v1*Pi*t);
}
double get_at_stop(double t,double a,double v,int sign_v1){
    double lef_time=0;
    double s=0;
    int si1=ge(v,0)?1:-1;
    int si2=ge(0,0)?1:-1;
    // a*=lt(si1*si2,0)||gt(si1*si2,0)&&gt(fabs(v),fabs(v1));
    if(lt(si1*si2,0)){
        a*=si2;
    }else if(gt(si1*si2,0)&&gt(fabs(v),fabs(0))){
        a*=si2*-1;
    }else{
        a*=si2;
    }
    // if(gt(a*sign_v1,0))a*=sign_v1;
    double tmpTime=(0-v)/(a);
    double realTime=tmpTime;
    s=v*realTime+0.5*a*realTime*realTime;
    double res=(s);
    if(gt(res*sign_v1,0)){
        return fabs(res);
    }
    else{
        return -1*fabs(res);
    }
    return (s);
}
double get_at_v_z(double t,double a,double v,int sign_v1){
    double lef_time=0;
    double s=0;
    a*=sign_v1;
    if(lt(fabs(v),Pi)){
        double tmpTime=(sign_v1*Pi-v)/(a);
        double realTime=min(tmpTime,t);
        s=v*realTime+0.5*a*realTime*realTime;
        double res=(s);
        if(le(t,tmpTime)){
            if(gt(res*sign_v1,0)){
                return fabs(res);
            }
            else{
                return -1*fabs(res);
            }
            return (s);
        }
        t=t-realTime;
    }
    double res=(s+sign_v1*Pi*t);
    if(gt(res*sign_v1,0)){
        return fabs(res);
    }else{
        return -1*fabs(res);
    }
    return (s+sign_v1*Pi*t);
}
bool is_near_tar(int id){
    double tmpDis=calcuDis(robots[id].pos,robots[robots[id].target_id].pos);
    if(lt(tmpDis,2))return true;
    return false;
}




bool checkNearBar(const pair<double,double> &a, double radius){
    int i, j;
    double x1, x2, y1, y2;
    double cross_y1, cross_y2, dis, radius2, dis1;
    int x_min, x_max, y_min, y_max;
    x1 = a.first - radius;
    x2 = a.first + radius;

    x_min = (int)(x1 / 0.5);
    x_max = (int)(x2 / 0.5);

    for(i = x_min; i <= x_max + 1; ++i) {
        dis1 = min(fabs(a.first - (i + 1) * 0.5), fabs(a.first - i * 0.5));
        // 没有交点
        if(gt(dis1, radius)) continue;
        dis = sqrt(radius2 - dis1 * dis1);
        y1 = a.second - dis;
        y2 = a.second + dis;
        y_min = (int)(y1 / 0.5);
        y_max = (int)(y2 / 0.5);

        for(j = y_min; j < y_max; ++j) {
            if(graph[i][j] == -2)
                return true;
        }
    }

    return false;
}

vector<pair<double,double>>Calculate_the_trajectory(Robot& rob,Ins ins_in, int forward_change, int rotate_change,const vector<pair<double,double>>&  tra,int cnt,int tar,double rob_dis,double pre_dis){
    //Calculate_the_trajectory_2
    // if(state.FrameID==4330&&state.FrameID==4330&&rob.id==0){
    //         cerr<<"ins ^ : "<<forward_change<<"-"<<rotate_change<<" "<<tra.size()<<endl;
    //         cerr<<state.FrameID+cnt<<endl;
    // }
    double t=0.02;
    Ins ins=contr_one_rob(rob);
    cerr<<"kkk";
    PayLoad pay=calPayload(rob,rob.virtual_pos);

    Flag_sumulate=0;
    double w_next=ins.rotate;
    double v_next=ins.forward;
    if(forward_change==1){
        v_next=ins_in.forward;
    }
    if(rotate_change==1){
        w_next=ins_in.rotate;
    }

    // 撞障碍物，返回空
    if(checkNearBar(rob.pos, rob.radius)){
        return {};
    }

    if(cnt>tar||cnt>=tra.size()){
        // if(forward_change == 0) {
        //     return {};
        // }
        return {rob.pos};
        // return {};
    }
    double tmpDis=calcuDis(rob.pos,tra[cnt]);        
    // if(state.FrameID==3487&&rob.id==3 && ins_in.forward == -2){
    //         // cerr<<"ins ^ : "<<forward_change<<"-"<<rotate_change<<endl;
    //         // cerr<<state.FrameID+cnt<<endl;
    //         cerr<<state.FrameID+cnt<<": "<<"epos1: "<<rob.pos.first<<"-"<<rob.pos.second<<" epos2:  "<<tra[cnt].first<<"-"<<tra[cnt].second<<endl;
    //         cerr<<tmpDis<<"--"<<pre_dis<<endl;
    // }
    if(gt(tmpDis,pre_dis + 0.04)){
        forward_change=0;
        rotate_change=0;
        // if(state.FrameID==849)
        // {        
        //     cerr<<"can inv: "<<state.FrameID+cnt<<endl;
        //     printRobotsDis(rob,tra[cnt]);
        // }
        // cerr<<state.FrameID+cnt<<": "<<"epos1: "<<rob.pos.first<<"-"<<rob.pos.second<<" epos2:  "<<tra[cnt].first<<"-"<<tra[cnt].second<<endl;
        // cerr<<tmpDis<<"--"<<pre_dis<<endl;
        return {rob.pos};
    }
    if(lt(tmpDis,rob_dis)){
        // if(state.FrameID>=4330&&state.FrameID>=4360&&rob.id==0){
        //     cerr<<"ins ^ : "<<forward_change<<"-"<<rotate_change<<endl;
        //     cerr<<state.FrameID+cnt<<endl;
        // }
        // cerr<<"vispos1: "<<rob.pos.first<<"-"<<rob.pos.second<<" vispos2:  "<<tra[cnt].first<<"-"<<tra[cnt].second<<endl;
        // cerr<<"ins: "<<forward_change<<"-"<<rotate_change<<endl;
        // cerr<<calcuDis(rob.pos, tra[cnt])<<" "<<rob_dis<<endl;
        // if(state.FrameID==860)
        // {        
        //     cerr<<"collo: "<<state.FrameID+cnt<<endl;
        //     printRobotsDis(rob,tra[cnt]);
        // }
        new_cllo_time=cnt*0.02;
        return {};
    }
    cnt++;
    Robot tmp=rob;
    double seta=rob.direction;
    double w=rob.angular_velocity==0?0.00001:rob.angular_velocity;
    double a=return_ac(pay.angular_acceleration,rob.angular_velocity,w_next);
    double changeAngle=get_at_v_limt(t,pay.angular_acceleration,rob.angular_velocity,w_next,pay.sign)*pay.sign;
    double v=Calculate_the_projection_speed(rob);
    double a_v=return_ac(pay.acceleration,v,v_next);
    rob.pos.first=rob.pos.first+v*cos(seta+changeAngle/2)*t;
    rob.pos.second=rob.pos.second+v*sin(seta+changeAngle/2)*t;
    int sign1=ge((rob.angular_velocity+a*t)*w_next,0)?1:-1;
    int sign2=ge((rob.angular_velocity+a*t),0)?1:-1;
    double limit_w=0.0;
    if(lt(a,0)){
        limit_w=lt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
    }else{
        limit_w=gt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
    }
    // if(state.FrameID==1){
    //     cerr<<cnt+1<<" - "<<rob.angular_velocity+a*t<<" "<<w_next<<" "
    //     <<a<<" "<<changeAngle<<endl;
    // }
    rob.angular_velocity=limit_w;
    
    // if(state.FrameID==1)cerr<<cnt-1<<" "<<changeAngle<<" "<<rob.direction<<endl;
    // rob.xy_pos=return_change_v(w,changeAngle*pay.sign,rob.xy_pos);
    int signv_1=ge((v+a_v*t)*v_next,0)?1:-1;
    int signv_2=ge((v+a_v*t),0)?1:-1;
    double limit_v=gt(fabs(v+a_v*t),fabs(6))?6*sign2:v+a_v*t;
    if(lt(a_v,0)){
        limit_v=lt(v+a_v*t,v_next)?v_next:v+a_v*t;
    }else{
        limit_v=gt(v+a_v*t,v_next)?v_next:v+a_v*t;
    }

    v=limit_v;
    // rob.direction+=changeAngle;
    // if(state.FrameID==2962&&rob.id==3){
    //     cerr<<pay.speed<<" "<<v<<" ^ "<<changeAngle<<" "<<v_next<<" "<<a_v<< endl;
    //     cerr<<rob.xy_pos.first<<"-"<<rob.xy_pos.second<<endl;
    // }
    double xy_angle=get_Angle_xy(rob);
    rob.xy_pos.first=v*cos(xy_angle);
    rob.xy_pos.second=v*sin(xy_angle);
    double xy_angle_next=get_Angle_xy(rob);
    double cal_angle=xy_angle_next-xy_angle;
    vector<vector<double>>mat(4,vector<double>(4,0));
    cal_matrix(mat,changeAngle,cal_angle);

    // if(state.FrameID==1800){
    //     cerr<<"v "<<v<<" old: "<<rob.xy_pos.first<<"-"<<rob.xy_pos.second<<" "<<xy_angle<<" "<<rob.direction<<"-"<< Calculate_the_projection_speed(rob)<< endl;
    // }
    rob.direction+=changeAngle;
    rob.direction=rob.direction>Pi?rob.direction-2*Pi:rob.direction; 
    // if(rob.direction>Pi)changeAngle=2*Pi-changeAngle;
    double t1=rob.xy_pos.first,t2=rob.xy_pos.second;
    rob.xy_pos.first=(t1*mat[0][0]+t2*mat[0][1]);
    rob.xy_pos.second=(t1*mat[1][0]+t2*mat[1][1]);
    // rob.xy_pos.first=(t1*cos(changeAngle+cal_angle)-t2*sin(changeAngle+cal_angle));
    // rob.xy_pos.second=(t1*sin(changeAngle+cal_angle)+t2*cos(changeAngle+cal_angle));
    //     cerr<<rob.xy_pos.first<<"-"<<rob.xy_pos.second<<endl;
    // }
    // rob.xy_pos.second=v_tmp.y;
    // if(Flag_sumulate){
    //     return {rob.pos};
    // }
    // if(Flag_sumulate){
    //     return {rob.pos};
    // }
    auto res=Calculate_the_trajectory(rob,ins_in,forward_change,rotate_change,tra,cnt,tar,rob_dis,tmpDis);
    if(res.size()>0)
    res.push_back(tmp.pos);
     if(cnt==1)reverse(res.begin(),res.end());
    return res;
}

vector<pair<double,double>>Calculate_the_trajectory(Robot& rob,int cnt,int tar,int ctrF){
    // cerr<<"aaaa"<<state.FrameID<<endl;
    double t=0.02;
    PayLoad  pay=calPayload_trajectory(rob,rob.target_id);
    Ins ins=contr_one_rob(rob);
    double w_next=ins.rotate;
    double v_next=ins.forward;
    if(cnt>tar){
        return {rob.pos};
    }
    // if(state.FrameID==1800){
    //     cerr<<" Framid: "<<state.FrameID+cnt<<" tarID: "<<rob.target_id<<" robId: "<<rob.id<<" w_v: "<<rob.angular_velocity<<" dirc: "<<rob.direction
    //     <<" pos_xy: "<<rob.pos.first<<"-"<<rob.pos.second<<" v_xy "<<rob.xy_pos.first<<"-"<<rob.xy_pos.second<<  endl;
    //     cerr<<"v: "<<pay.speed<<endl;
    // }
    cnt++;
    Robot tmp=rob;
    double seta=rob.direction;
    double w=rob.angular_velocity==0?0.00001:rob.angular_velocity;
    double a=return_ac(pay.angular_acceleration,rob.angular_velocity,w_next);
    double changeAngle=get_at_v_limt(t,pay.angular_acceleration,rob.angular_velocity,w_next,pay.sign)*pay.sign;
    // if(state.FrameID==1){
    //     cerr<<changeAngle<<endl;
    // }
    double v=Calculate_the_projection_speed(rob);
    double a_v=return_ac(pay.acceleration,v,v_next);
    rob.pos.first=rob.pos.first+v*cos(seta+changeAngle/2)*t;
    rob.pos.second=rob.pos.second+v*sin(seta+changeAngle/2)*t;
    int sign1=ge((rob.angular_velocity+a*t)*w_next,0)?1:-1;
    int sign2=ge((rob.angular_velocity+a*t),0)?1:-1;
    double limit_w=0.0;
    if(lt(a,0)){
        limit_w=lt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
    }else{
        limit_w=gt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
    }
    // if(state.FrameID==1){
    //     cerr<<cnt+1<<" - "<<rob.angular_velocity+a*t<<" "<<w_next<<" "
    //     <<a<<" "<<changeAngle<<endl;
    // }
    rob.angular_velocity=limit_w;
    
    // if(state.FrameID==1)cerr<<cnt-1<<" "<<changeAngle<<" "<<rob.direction<<endl;
    // rob.xy_pos=return_change_v(w,changeAngle*pay.sign,rob.xy_pos);
    int signv_1=ge((v+a_v*t)*v_next,0)?1:-1;
    int signv_2=ge((v+a_v*t),0)?1:-1;
    double limit_v=gt(fabs(v+a_v*t),fabs(6))?6*sign2:v+a_v*t;
    if(lt(a_v,0)){
        limit_v=lt(v+a_v*t,v_next)?v_next:v+a_v*t;
    }else{
        limit_v=gt(v+a_v*t,v_next)?v_next:v+a_v*t;
    }

    v=limit_v;
    // rob.direction+=changeAngle;
    // if(state.FrameID==1){
    //     cerr<<pay.speed<<" "<<v<<" ^ "<<changeAngle<<" "<<v_next<<" "<<a_v<< endl;
    //     cerr<<rob.xy_pos.first<<"-"<<rob.xy_pos.second<<endl;
    // }
    double xy_angle=get_Angle_xy(rob);
    rob.xy_pos.first=v*cos(xy_angle);
    rob.xy_pos.second=v*sin(xy_angle);
    double xy_angle_next=get_Angle_xy(rob);
    double cal_angle=xy_angle_next-xy_angle;
    vector<vector<double>>mat(4,vector<double>(4,0));
    cal_matrix(mat,changeAngle,cal_angle);

    // if(state.FrameID==1800){
    //     cerr<<"v "<<v<<" old: "<<rob.xy_pos.first<<"-"<<rob.xy_pos.second<<" "<<xy_angle<<" "<<rob.direction<<"-"<< Calculate_the_projection_speed(rob)<< endl;
    // }
    rob.direction+=changeAngle;
    rob.direction=rob.direction>Pi?rob.direction-2*Pi:rob.direction; 
    // if(rob.direction>Pi)changeAngle=2*Pi-changeAngle;
    double t1=rob.xy_pos.first,t2=rob.xy_pos.second;
    rob.xy_pos.first=(t1*mat[0][0]+t2*mat[0][1]);
    rob.xy_pos.second=(t1*mat[1][0]+t2*mat[1][1]);
    // rob.xy_pos.first=(t1*cos(changeAngle+cal_angle)-t2*sin(changeAngle+cal_angle));
    // rob.xy_pos.second=(t1*sin(changeAngle+cal_angle)+t2*cos(changeAngle+cal_angle));
    // if(state.FrameID==514&&rob.id==3){
    //     cerr<<mat[0][0]<<"-"<<mat[0][1]<<endl;
    //     cerr<<mat[1][0]<<"-"<<mat[1][1]<<endl;
    // }
    // rob.xy_pos.second=v_tmp.y;
    if (Flag_sumulate && ctrF)
    {
        return {rob.pos};
    }
    auto res=Calculate_the_trajectory(rob,cnt,tar,ctrF);
    res.push_back(tmp.pos);
    if(cnt==1)reverse(res.begin(),res.end());
    return res;
}
PayLoad calPayload_trajectory(Robot rob,int studioID){
    Robot robot = rob;
    Studio studio = studios[studioID];

    // cerr << robotID << "--"<< robot.target_id<<endl;

    double distance = calcuDis(robot.pos, studio.pos);
    double angular_acceleration = robot.get_type == 0? angular_acceleration_no :angular_acceleration_has;
    double acceleration = robot.get_type == 0? acceleration_no: acceleration_has;

    // 计算机器人与目标点构成的向量与x轴正方向夹角
    pair<double, double> robotToStudio = subVector(studio.pos, robot.pos);
    double angle1 = calAngle(robotToStudio);

    double angle2 = ge(robot.direction, 0.0) ? robot.direction: 2 * Pi + robot.direction;
    // double angle2 = calAngle(robot.xy_pos);

    double angle = angle2 - angle1;

    double speed = calVectorSize(robot.xy_pos) * (ge(calVectorProduct(robot.xy_pos, transformVector(robot.direction)), 0.0)? 1: -1);

    int sign;

    if(ge(angle, 0) && lt(angle, Pi) || lt(angle, -Pi))
        sign = -1;
    else
        sign = 1;
    angle = fabs(angle);
    angle  = gt(angle, Pi)? 2 * Pi - angle: angle;


    // cerr<<"**"<< angle1<<"**dir:"<<robot.direction<<"**"<<angle2<<endl;
    // cerr<<"**"<< angle << "**"<<distance<<"**"<<sign<<endl;

    return PayLoad((robot.get_type == 0? 0.45: 0.53), angle, angular_acceleration, acceleration, distance, speed, sign);    
}
Ins contr_one_rob(Robot& robot){
    Flag_sumulate=0;
    Ins ins_t;
    if(robot.target_id==-1){
        ins_t.forward=0;
        ins_t.rotate=Pi;
        return ins_t;
    }
    adjust_virtual_pos_total(robot);
    double tmpDis=calcuDis(robot.pos,exist_id[0][robot.node_id]);
    PayLoad payload=calPayload(robot,robot.virtual_pos);
    // if(robot.id==0&&state.FrameID>=10&&state.FrameID<=100&&contr_print_flag){
    //     cerr<<" FrameID "<< state.FrameID<<" "<<robot.virtual_pos.first<<"-"<<robot.virtual_pos.second<<endl;
    //     cerr<<check_can_arrival(robot.get_type==0?0:1,robot.node_id,robot.virtual_id)<<endl;
    //     cerr<<calcuDis(robot.pos,exist_id[0][robot.node_id])<<endl;
    //     cerr<<robot.node_id<<" "<<robot.virtual_id<<endl;
    // }
    
    auto p1=get_w_now(robot,payload);
    ins_t.rotate=p1.first;
    ins_t.forward=6;
    if(lt(payload.distance,1)){
        ins_t.forward=1;
    }else if(lt(payload.distance,0.5)){
            ins_t.forward=0.5;
    }
    if(lt(payload.distance,1)&&!p1.second){
            ins_t.forward=0;
    } 
    if(!robot.isVir&&lt(payload.distance,0.3)){
        robot.cnt_tar=ret_next(robot,robot.cnt_tar);
    }
    if(gt(payload.distance,1)&&!p1.second)
    {
        ins_t.forward=vir_v(robot);
    }
//    if(robot.id==0&&state.FrameID>=10&&state.FrameID<=100&&contr_print_flag){
//     cerr<<"forward: "<<ins_t.forward<<endl;
//     cerr<<"angle "<<payload.angle<<endl;
//     cerr<<"dis "<<payload.distance<<endl;
//     cerr<<robot.isVir<<endl;
//    }
    return ins_t;
}

pair<double,bool> get_w_now(const Robot& robot, const PayLoad& payload){
    int robStuID=robot.target_id;
    if(robStuID==-1){
         robStuID=0;
    }
    int robID=robot.id;
    double Dev_val=get_at_stop(0.02,payload.angular_acceleration
    ,robot.angular_velocity,payload.sign);
    double rateAngle_fabs=0;
    if(gt(payload.angle,0.3)){
        rateAngle_fabs=Pi;
    }else if(gt(payload.angle,0.15)){
        rateAngle_fabs=Pi/2;
    }else if(gt(payload.angle,0.075)){
        rateAngle_fabs=Pi/4;
    }else{
        rateAngle_fabs=Pi/8;
    }
    double angle=get_at_v_limt(0.02,payload.angular_acceleration
    ,robot.angular_velocity,rateAngle_fabs,payload.sign);
    double StopA=0;
    double real_angle=angle;
    int can_stop_flag=0;
    bool con1=gt(Dev_val,payload.angle);
    if(con1){
        real_angle=get_at_v_limt(0.02,payload.angular_acceleration
            ,robot.angular_velocity,0,payload.sign);
        can_stop_flag=1;
        StopA=0;
    } 
    double cmpAngle=fabs(payload.angle-real_angle);
    
    bool can_st=can_stop(robot.pos,robot.virtual_pos,cmpAngle,false);
    if(can_st){
        can_stop_flag=1;
        StopA=0;        
    }
    // if(state.FrameID>=161&&state.FrameID<=1452){
    //     cerr<<"ID: "<<state.FrameID<<" "<<robot.id<<" "<<robot.virtual_pos.first<<"-"<<robot.virtual_pos.second<<" "<<payload.distance<<endl;
    //     cerr<<can_st<<endl;
    // }
    double tmpAngle =can_stop_flag?StopA:rateAngle_fabs*payload.sign;
    return {tmpAngle,can_st} ;
}
double get_v_now(const Robot& robot, const PayLoad& payload){
    double res_v=6;
    if(lt(payload.distance,1)){
        res_v=1;
    }else if(lt(payload.distance,0.5)){
        res_v=0.5;
    }
    return res_v;
}


bool cmp_robot(Robot a, Robot b) {
    if(a.target_id == -1) 
        return true;
    if(b.target_id == -1) 
        return false;
    if(a.get_type == b.get_type) {
        return dis_to_studios[a.target_id][(a.get_type != 0)][a.node_id] <dis_to_studios[b.target_id][(b.get_type != 0)][b.node_id];
    }
    return a.get_type < b.get_type;
}


void collision_solve(int frame){
    int i, j, k, z;
    vector<pair<double,double>> trajectory[4];
    vector<pair<double,double>> tmp_tra, tra;
    vector<Robot> ro;
    double mindis;
    double dis, dis_tmp;
    int ans, tmp;
    int ins_num;
    vector<int> coll[4];
    int coll_time[4][4] = {0};
    int vis[4] = {0};
    int reachTime[4], stopID, goID;
    int choose_id = -1;
    int min_size;
    int x;
    bool flag;

    bool cerr_falg = false;


    // if(state.FrameID >= 5600 && state.FrameID <= 5630 && 999==999)
        cerr_falg = true;


    for(i = 0; i < 4; ++i)
        ro.emplace_back(robots[i]);
    sort(ro.begin(), ro.end(), cmp_robot);
    for(i = 0; i < 4; ++i) trajectory[i] = Calculate_the_trajectory(ro[i], 0, frame, 0);

    // if(state.FrameID == 2) {
    //     cerr<<"predict"<<endl;
    //     for(i = 0;i<25;++i){
    //         cerr<<i+2+1<<endl;
    //         for(j=0;j<4;++j){
    //             cerr<<ro[j].id<<":"<<trajectory[j][i].first<<","<<trajectory[j][i].second<<endl;
    //         }
    //     }
    //     cerr<<endl;
    // }

    

    for (i = 0; i < 4; i++)
    {
        for (j = i + 1; j < 4; j++)
        {
            mindis = ro[i].radius + ro[j].radius;
            tmp = checkNoCollision(trajectory[i], trajectory[j], mindis + 0.2);
            // if(state.FrameID == 1588 && ((ro[i].id == 3 && ro[j].id == 0) || (ro[i].id == 0 && ro[j].id == 3)))
            //     cerr<<"mindis:"<<mindis<<endl;
            coll_time[i][j] = tmp;
            coll_time[j][i] = tmp;
            if(ro[i].get_type != ro[i].last_get_type){
                last_solution[ro[i].id][ro[j].id] = -1;
                last_solution[ro[j].id][ro[i].id] = -1;
            }
                
            if(tmp == 9000 || gt(calcuDis(ro[i].pos, ro[j].pos), 8)) {
                //同个目标
                // if(ro[i].target_id == ro[j].target_id && ro[i].target_id != -1) {
                //     reachTime[ro[i].id] = getTimeToStudio(ro[i].id, trajectory[i]);
                //     reachTime[ro[j].id] = getTimeToStudio(ro[j].id, trajectory[j]);
                    
                //     if(reachTime[ro[i].id] == reachTime[ro[j].id] && reachTime[ro[i].id]==1000) continue;
                //     stopID = (reachTime[ro[i].id]<reachTime[ro[j].id])? ro[j].id: ro[i].id;
                //     goID = (stopID == ro[i].id)? ro[j].id: ro[i].id;
                //     ins[stopID].forward = min(payloads[stopID].distance / (reachTime[goID]*0.02 + 0.5), ins[stopID].forward);

                //     // if(cerr_falg) {
                //     //     cerr<<"time:"<<state.FrameID<<endl;
                //     //     // cerr<<ro[i].id<<"reach time:"<<reachTime[ro[i].id]<<endl;
                //     //     // cerr<<ro[j].id<<"reach time:"<<reachTime[ro[j].id]<<endl;
                //     //     cerr<<stopID<<"stop:"<<ins[stopID].forward<<endl;
                //     // }

                // }
                last_solution[ro[i].id][ro[j].id] = -1;
                last_solution[ro[j].id][ro[i].id] = -1;
                continue;
            }
            coll[i].emplace_back(j);
            coll[j].emplace_back(i);
            
            // cerr_falg=true;
            if(cerr_falg)
            {cerr<<"time:"<<state.FrameID<<endl;
            cerr<<ro[i].id<<"-"<<ro[j].id<<"collison"<<tmp<<endl;}
            // cerr_falg=false;
        }
    }

    // x = -1;
    // for(i = 0; i < 2; ++i) {
    //     choose_id = -1;
    //     //选择碰撞最多的小球改变状态
    //     for(j = 0; j < 4; ++j){
    //         if(vis[j] || coll[j].size() == 0)
    //             continue;
    //         if(choose_id == -1 || coll[j].size() - ((x == j)) > coll[choose_id].size()) {
    //             choose_id = j;
    //         }
    //     }

              
        
    //     //No collision
    //     if(choose_id == -1 || coll[choose_id].size() - (x == choose_id) == 0)
    //         break;


    //     // if(cerr_falg) {
    //     //     cerr<<state.FrameID<<endl;
    //     //     cerr<<check_wall_r(1)<<endl;
    //     //     for(j=0;j<4;++j){
    //     //         cerr<<ro[j].id<<":"<<payloads[ro[j].id].speed<<" target:"<<ro[j].target_id<<endl;
    //     //     }
    //     // }



    //     tmp = 9000;
    //     //避让最zao发生的碰撞
    //     for(j = 0; j < coll[choose_id].size(); ++j) {
    //         if(coll_time[choose_id][coll[choose_id][j]] < tmp) {
    //             x = coll[choose_id][j];
    //             tmp = coll_time[choose_id][coll[choose_id][j]];
    //         }
    //     }

    //     if(cmp_robot(ro[x], ro[choose_id]) && vis[x] != 1) {
    //         tmp = x;
    //         x = choose_id;
    //         choose_id = tmp;
    //     }

    //     vis[choose_id] = 1;

        
    //     if(x == -1) {
    //         // if(cerr_falg)
    //         // {cerr<<choose_id<<"*"<<coll[choose_id].size();
    //         // cerr<<"xx"<<x<<endl;}
    //         break;
    //     }

    //     if(cerr_falg) {
    //         cerr<<ro[choose_id].id<<"target:"<<ro[choose_id].target_id<<" angle:"<<payloads[ro[choose_id].id].speed<<endl;
    //         cerr<<ro[x].id<<"target:"<<ro[x].target_id<<" angle:"<<payloads[ro[x].id].speed<<endl;
    //         cerr<<ro[choose_id].id<<"avoid"<< ro[x].id<<endl;
    //     }


    //     ans = -1;
    //     dis = 1000;
    //     min_size = 27;
    //     mindis = ro[choose_id].radius + ro[x].radius + 0.1;
    //     ins_num = ((le(payloads[ro[choose_id].id].speed, 1) && le(payloads[ro[choose_id].id].distance, 2)) || eq(payloads[ro[choose_id].id].speed, 0) && !isNearWall(ro[choose_id].id))? 8: 7; 
    //     for(k = 0; k < ins_num; ++k) {
    //         if(k < 3) {
    //             tmp_tra = Calculate_the_trajectory(ro[choose_id], ins_set[k], 1, 1, trajectory[x], 0, 25, mindis + 0.2, 100);
    //         }
    //         else if(k < 6) {
    //             tmp_tra = Calculate_the_trajectory(ro[choose_id], ins_set[k], 0, 1, trajectory[x], 0, 25, mindis + 0.2, 100);
    //         }
    //         else {
    //             tmp_tra = Calculate_the_trajectory(ro[choose_id], ins_set[k], 1, 0, trajectory[x], 0, 25, mindis, 100);
    //         }

    //         if(cerr_falg) cerr<<k<<"-"<<tmp_tra.size()<<endl;
    //         if(tmp_tra.size() == 0) continue;
    //         flag = false;

    //         //检测是否会和其他小球发生碰撞
    //         // for(j = 0; j < 4; ++j){
    //         //     if(j == choose_id) continue;
    //         //     if(checkNoCollision(tmp_tra, trajectory[j], ro[choose_id].radius + ro[j].radius) == 9000) {
    //         //         flag =true;
    //         //         break;
    //         //     }
    //         // }
    //         //若和其他小球碰撞则更换策略
    //         // if(flag) continue;
    //         dis_tmp = calcuDis(tmp_tra[tmp_tra.size() - 1], studios[ro[choose_id].target_id].pos);
    //         if(cerr_falg) cerr<<"dis:"<<dis_tmp<<endl;

    //         if((le(dis_tmp, dis) && min_size >= tmp_tra.size()) || min_size > tmp_tra.size() ) {
    //             min_size = tmp_tra.size();
    //             dis = dis_tmp;
    //             ans = k;
    //             tra = tmp_tra;
    //         }
    //     }

    //     if(ans != -1) {
    //         if(cerr_falg) {
    //             cerr<<payloads[ro[choose_id].id].angle<<"-"<<payloads[ro[choose_id].id].sign<<endl;
    //             cerr<<"old solution:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<endl;
    //         }
    //         trajectory[choose_id] = tra;
    //         // if(cerr_falg) cerr<<"ans"<<ans<<endl;
    //         updateIns(ro[choose_id].id, ans);
    //         coll_time[x][choose_id] = 0;
    //         last_solution[ro[choose_id].id][ro[x].id] = ans;
    //         last_solution[ro[x].id][ro[choose_id].id] = -1;

    //         if(ro[choose_id].target_id == ro[x].target_id && ro[choose_id].target_id != -1) {

    //             reachTime[ro[x].id] = getTimeToStudio(ro[x].id, trajectory[x]);
                    
    //             stopID = ro[choose_id].id;
    //             goID = ro[x].id;
    //             ins[stopID].forward = min(payloads[stopID].distance / (reachTime[goID]*0.02 + 0.5), ins[stopID].forward);


    //                 if(cerr_falg) {
    //                     // cerr<<"time:"<<state.FrameID<<endl;
    //                     // cerr<<ro[i].id<<"reach time:"<<reachTime[ro[i].id]<<endl;
    //                     // cerr<<ro[j].id<<"reach time:"<<reachTime[ro[j].id]<<endl;
    //                     cerr<<stopID<<"stop:"<<ins[stopID].forward<<endl;
    //                 }
    //         }

    //         if(cerr_falg) {
    //             cerr<<ans<<endl;
    //             cerr<<ro[x].id<<"-"<<ro[choose_id].id<<":"<<last_solution[ro[x].id][ro[choose_id].id]<<endl;
    //             if(ans<3) {
    //                 cerr<<"chose solution11:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<endl;
    //             }
    //             else if(ans<6) {
    //                 cerr<<"chose solution01:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<endl;
    //             }
    //             else {
    //                 cerr<<"chose solution10:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<endl;
    //             }
                                
    //             // if(state.FrameID == 3487){
    //             //     cerr<<"-----------"<<endl;
    //             //     for(int t = 0; t<tra.size(); ++t){
    //             //         if(state.FrameID+t>3600) break;
    //             //         cerr<<state.FrameID+t;
    //             //         cerr<<"pos:("<<tra[t].first<<", "<<tra[t].second<<")--("<<trajectory[x][t].first<<", "<<trajectory[x][t].second<<") dis:"<<calcuDis(trajectory[x][t], tra[t])<<endl;
    //             //     }
    //             //     cerr<<"-----------"<<endl;
    //             // }
                
    //             // updateIns(ro[choose_id].id, 7);
    //             // int t = 1;
    //             // cerr<<"pos:("<<tra[t].first<<", "<<tra[t].second<<")--("<<trajectory[x][t].first<<", "<<trajectory[x][t].second<<") dis:"<<calcuDis(trajectory[x][t], tra[t])<<endl;
    //             // cerr<<state.FrameID;
    //             // cerr<<"pos:("<<ro[choose_id].pos.first<<", "<<ro[choose_id].pos.second<<")--("<<ro[x].pos.first<<", "<<ro[x].pos.second<<") dis:"<<calcuDis(ro[choose_id].pos, ro[x].pos)<<endl;
    //         }
    //     }
    //     else{
    //         // if(cerr_falg) updateIns(ro[choose_id].id, 4);
    //         // else
    //         // adjust_collo_new(ro[choose_id].id, ro[x].id, payloads[ro[choose_id].id].sign);
    //         // solveNoSolution(ro[choose_id].id, ro[x].id);
    //         // if(cerr_falg)
    //         if(cerr_falg) {
    //             cerr<<"~old solution:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<endl;
    //             // cerr<<ro[choose_id].id<<"-"<<ro[x].id<<":"<<"last_solution[choose_id][x]"<<last_solution[ro[choose_id].id][ro[x].id]<<endl;
    //         }
    //         // cerr<<state.FrameID<<"no solution to avoid collision"<<ro[choose_id].id<<"-"<<ro[x].id<<"*"<<coll_time[choose_id][x]<<endl;

    //         if(last_solution[ro[choose_id].id][ro[x].id] != -1) {
    //             if(le(payloads[ro[choose_id].id].speed, 0) && le(ro[choose_id].angular_velocity, 0))
    //                 updateIns(ro[choose_id].id, 7);
    //             else updateIns(ro[choose_id].id, last_solution[ro[choose_id].id][ro[x].id]);
                
    //             if(cerr_falg)
    //             {
    //                 if(last_solution[ro[choose_id].id][ro[x].id]<3) {
    //                     cerr<<"~chose solution11:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<endl;
    //                 }
    //                 else if(last_solution[ro[choose_id].id][ro[x].id]<6) {
    //                     cerr<<"~chose solution01:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<endl;
    //                 }
    //                 else {
    //                     cerr<<"~chose solution10:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<endl;
    //                 }
    //             }
    //         }
    //         else {
    //             if(vis[x] && vis[choose_id]) {
    //                 // ins[ro[choose_id].id].forward = -2;
    //                 if(cerr_falg) cerr<<"no way"<<endl;
    //                 adjust_collo_new(ro[choose_id].id, ro[x].id, payloads[ro[choose_id].id].sign);
    //                 continue;
    //             }
    //             choose_id = x;
    //             x = -1;
    //             i--;
    //         }            
    //     }
          
    // }

    // if(state.FrameID == 182 || state.FrameID == 187) {
    //     int a,b;
    //     for(i = 0; i < 4; ++i){
    //         if(ro[i].id == 2) a = i;
    //         if(ro[i].id == 3) b =i;
    //     }
    //     printPredictRobotsDis(trajectory[a], trajectory[b]);
    // }

    // if(state.FrameID >= 182 && state.FrameID <= 187) {
    //     cerr<<state.FrameID;
    //     printRobotsDis(2,3);
    // }
    updateGetType();
}




void printRobotsDis(int i, int j){
    cerr<<"&pos:("<<robots[i].pos.first<<", "<<robots[i].pos.second<<")--("<<robots[j].pos.first<<", "<<robots[j].pos.second<<") dis:"<<calcuDis(robots[i].pos, robots[j].pos)<<endl;
}

void printRobotsDis(Robot ro, pair<double,double> a){
    cerr<<"pos:("<<ro.pos.first<<", "<<ro.pos.second<<")--("<<a.first<<", "<<a.second<<") dis:"<<calcuDis(ro.pos, a)<<endl;
}

void printPredictRobotsDis(const vector<pair<double,double>> &a, const vector<pair<double,double>> &b) {
    int count = min(a.size(), b.size());
    for(int i = 0; i < count; ++i) {
        cerr<<state.FrameID+i;
        cerr<<"pos:("<<a[i].first<<", "<<a[i].second<<")--("<<b[i].first<<", "<<b[i].second<<") dis:"<<calcuDis(a[i], b[i])<<endl;
    }
    cerr<<"-----------"<<endl;
}

int getTimeToStudio(int id, const vector<pair<double,double>> &a) {
    int len = a.size();
    for(int i = 0; i < len; ++i) {
        if(lt(calcuDis(studios[robots[id].target_id].pos, a[i]), 0.4))
            return i;
    }
    return 1000;
}



void updateIns(int id, int i) {
    if(i<3) {
        ins[id].forward = ins_set[i].forward;
        ins[id].rotate = ins_set[i].rotate;
        // cerr<<"chose solution11:"<<ins[id].forward<<"**"<<ins[id].rotate<<endl;
    }
    else if(i<6) {
        ins[id].rotate = ins_set[i].rotate;
        // cerr<<"chose solution01:"<<ins[id].forward<<"**"<<ins[id].rotate<<endl;
    }
    else {
        ins[id].forward = ins_set[i].forward;
        // cerr<<"chose solution10:"<<ins[id].forward<<"**"<<ins[id].rotate<<endl;
    }
}



int checkNoCollision(const vector<pair<double,double>> &a, const vector<pair<double,double>> &b, double mindis) {
    int count = min(a.size(), b.size());
    for(int i = 0; i < count; ++i) {
        if(lt(calcuDis(a[i], b[i]), mindis))
            return i;
    }
    return 9000;
}

void updateGetType(){
    for(int i = 0; i < 4; ++i){
        robots[i].last_get_type = robots[i].get_type;
    }
}

pair<double ,double> return_change_v(double w,double changeSeta,pair<double,double>v){
    // cerr<<v.first<<' '<<v.second<<endl;

    double v_value = sqrt(v.first*v.first+v.second*v.second);
    double r = (v_value/fabs(w));
    double l = fabs(w)*r;
    // cerr<<(v.first/v_value)<<endl;
    // cerr<<(v.second/v_value)<<endl;
    double direct1 = acos(v.first/v_value);
    // cerr<<"direct1 = "<<direct1<<endl;
    double direct2;
    if(asin(v.second/v_value)<0)direct1 += Pi;
    direct2 = direct1 + changeSeta;
    // cerr<<"direct2 = "<<direct2<<endl;
    if(direct2>(2*Pi))direct2 -= (2*Pi);
    if(direct2<0)direct2 += (2*Pi);
    // cerr<<"direct2 = "<<direct2<<endl;
    double v_new = l/0.02;
    return pair<double,double>((v_new*cos(direct2)),(v_new*sin(direct2)));

}
double will_Collo_new(int i1,int i2){
    new_cllo_time=-8;
    Ins ins;
    auto res1= Calculate_the_trajectory(robots[i2],0,25,0);
    auto res2= Calculate_the_trajectory(robots[i1],ins,0,0,res1,0,25,getRobotRadius(i1)+getRobotRadius(i2)+0.2);
    
    return new_cllo_time;
}
void adjust_collo_new(int i1,int i2,int baseSign){
    double tmpDis=calcuDis(robots[i1].pos,robots[i2].pos);
    int sel=i1,sel_1=i2;
    if(lt(tmpDis,5)){
        int sign=return_line_dire(sel,sel_1,baseSign);
        //cerr<<"FrameID  "<<state.FrameID<<" collosion: "<<sel_1<<"-> "<<sel<<" "<<sign<<endl;
        if(sign==0)return;
        ins[sel_1].rotate=Pi/4*sign;    
    }
}
bool check_wall_r(int i){
    Robot robot=robots[i];
    vector<double> tmp=get_T_limits(robot);
    bool con1=robot.need_rote_wall;
    bool con2=!eq(tmp[0],-7)&&(!is_range(robot.direction,tmp))&&ge(payloads[i].angle,Pi/6);
    if(con1&&con2){
        return true;
    }    
    return false;
}
double get_Angle_xy(Robot& rob){
    if(lt(fabs(rob.xy_pos.first),0.1)&&lt(fabs(rob.xy_pos.second),0.1)){
        return rob.direction;
    }
    Vec v1;
    v1.x=1;
    v1.y=0;
    Vec v2(rob.xy_pos);
    double angle=acos(cos_t(v1,v2));
    int sign=lt(rob.xy_pos.second,0)?-1:1;
    return angle*sign;
}
double Calculate_the_projection_speed(Robot& rob){
    Vec v1;
    v1.x=cos(rob.direction);
    v1.y=sin(rob.direction);;
    Vec v2(rob.xy_pos);
    return v1*v2;  
}
void cal_matrix(vector<vector<double>>&c,double angle1_w,double angle2){
     double a[2][2];
     double b[2][2];
    a[0][0]=cos(angle1_w);
    a[0][1]=-1*sin(angle1_w);
    a[1][0]=sin(angle1_w);
    a[1][1]=cos(angle1_w);
    b[0][0]=cos(angle2);
    b[0][1]=-1*sin(angle2);
    b[1][0]=sin(angle2);
    b[1][1]=cos(angle2);
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }    
}
bool check_will_colloWithWall(const Robot& rob){
    int tarid=rob.target_id==-1?0:rob.target_id;
    double dis=calcuDis(rob.pos,studios[tarid].pos);
    double stop_dis=(rob.xy_pos.first*rob.xy_pos.first+rob.xy_pos.second*rob.xy_pos.second)
        /(2*payloads[rob.id].acceleration);
    if(isWall(tarid)&&gt(stop_dis+0.2,dis)){
        return true;
    }
    return false;
}
bool check_fall_into_scope(double k1,double b1,double b2,double k2,double b3,double b4,pair<double,double> pos){
    if(gt((k1*pos.first+b1),pos.second)){
        if (lt((k1 * pos.first + b2), pos.second)){
            if (gt((k2 * pos.first + b3), pos.second)){
                if (gt((k2 * pos.first + b4), pos.second)){
                    return true;
                }
            }
        }
    }
    return false;
}
bool check_barrier(int start,int end,int carry){
    double r, k, b, offset,k1,b1,b2,k2,b3,b4;
    if(carry==1)r = 0.45;
    else r = 0.53;
    if (!eq((panes[end].pos.first - panes[start].pos.first),0 )){
        k =(double) (panes[end].pos.second - panes[start].pos.second) / (panes[end].pos.first - panes[start].pos.first);
    }
    else if (gt((panes[end].pos.second - panes[start].pos.second),0)){
        k = 100;
    }
    else if (lt((panes[end].pos.second - panes[start].pos.second), 0)){
        k = -100;
    }
    else k = 0;
    b = (panes[start].pos.second - k * panes[start].pos.first);
    k1 = k;
    offset =(double)r/(cos(Pi-atan(k)));
    if(gt(offset,0)){
        b1 = b + offset;
        b2 = b - offset;
    }
    else{
        b1 = b - offset;
        b2 = b + offset;
    }
    k2 = (double)(-1/k1);
    if (gt(panes[end].pos.second, panes[start].pos.second)){
        b1 = panes[end].pos.second - k2 * (panes[end].pos.first);
        b2 = panes[start].pos.second - k2 * (panes[start].pos.first);
    }
    else{
        b2 = panes[end].pos.second - k2 * (panes[end].pos.first);
        b1 = panes[start].pos.second - k2 * (panes[start].pos.first);
    }
    // for(int i = 0; i<101; i++){
    //     for(int j=0; j<101; j++){
    //         if (check_fall_into_scope(k1,b1,b2,k2,b3,b4,pair<double,double>(i*0.5,j*0.5))){
    //             if(wail[i][j]==-2){
    //                 return false;
    //             }
    //         }
    //     }
    // }
    return true;

}
// void init_dis(){
//     int i,j;
//     for(i=0;i<10000;i++){
//         for(j=i;j<10000;j++){
//             if (i == j)
//                 dis[j][i] = 0;
//             else{
//                 if (check_barrier(i,j,0)){
//             //         // cerr<<"cc"<<endl;
//                     dis[i][j] = calcuDis(panes[i].pos, panes[j].pos);
//                     dis[j][i] = dis[i][j];
//                     // if(j==9999){
//                     //     cerr<<i<<' '<<j<<' '<<dis[i][j]<<endl;
//                     //     dis[j][i] = dis[i][j];
//                     // }
//                     target_sequence[i][j] = j;

//                     target_sequence[j][i] = i;
//                 }
//                 else{
//                     dis[i][j] = 1000;
//                     dis[j][i] = dis[i][j];
//                     target_sequence[i][j] = -1;
//                     target_sequence[j][i] = -1;
//                 }
//             //     // cerr<<"bb"<<endl;
//             }
//             // if(i>=3728)
//             // cerr<<"j = "<<j<<endl;
//         }
//         cerr<<i<<endl;
//     }
// }

// void floyd(){
//     // init_dis();
//     cerr<<"AA"<<endl;
//     // for(int i = 0;i<10000;i++){
//     //     for(int j = 0;j<10000;j++){
//     //         for(int k =j+1 ;k<10000;k++){
//     //             if (gt(dis[j][k],dis[j][i]+dis[i][k])){
//     //                 dis[j][k] = dis[j][i]+dis[i][k];
//     //                 dis[k][j] = dis[j][k];
//     //                 target_sequence[j][k] = target_sequence[j][i];
//     //                 target_sequence[k][j] = target_sequence[k][i];
//     //             }
//     //         }
//     //     }
//     // }

// }
// void print_queue(){
//     for(int i=0;i<10000;i++){
//         for(int j = 0; j<10000;j++){
//             cerr<<i<<' '<<j<<' '<<dis[i][j]<<endl;
//             cerr<<target_sequence[i][j];
//         }
//         cerr<<endl;
//     }
// }
 bool check_side(int i,int j,int flag,int is_take){
    int count1,count2,k;
    j = j + flag;
    count1=99-j+1;
    count2=99-j+1;
    for(k=j;k<99;k++){
        if (count1 == (99-j+1) && graph[i - 1][k] != graph[i - 1][k+1]){
            count1 = k-j+1;
        }
        if (count2 == (99-j+1) && graph[i][k] != graph[i][k + 1])
        {
            count2 = k - j + 1;
        }
        if(count1!=(99-j+1)&&count2!=(99-j+1))break;
    }
    // if(graph[i-1][j]==21)
        // cerr<<" abs(count1-count2) "<<i<<' '<<j<<' '<<abs(count1-count2)<<' '<<count1<<' '<<count2<<endl;
    if(count1>(is_take+3)&&count2>(is_take+3)){
        if(abs(count1-count2)>(is_take+2)){
            // if(graph[i-1][j]==21)
                // cerr<<"kkk"<<' '<<i<<' '<<j<<endl;
            return false;
        }
    }
    else{
        if(abs(count1-count2)>0){
            // if(graph[i-1][j]==21)
                // cerr<<"kkk"<<' '<<i<<' '<<j<<endl;
            return false;
        }
    }
    // cerr<<"flag = "<<flag<<endl;
    if(flag == 0){
        count1 = j;
        for (k = j; k >=1; k--)
        {
            if (count1 == j && graph[i - 1][k-1] != graph[i - 1][k])
            {
                count1 = j - k + 1;
                break;
            }
        }
        // if(graph[i-1][j]==21)
            // cerr<<" count1 "<<i<<' '<<count1<<endl;
        if(count2>(is_take+3)){
            if(count1>(is_take+2))return false;
        }
        else{
            if(count1>0)return false;
        }
    }
    return true;
 }
 pair<double, double> check_wail_change(int i, int j, int type)
 {
    double x, y;
    int flag=0;
    if (graph[i][j] != type)
    {
        for(int k = j;k<99;k++){
            if(graph[i][k]!=-2){
                if(graph[i][k]!=type){
                    break;
                }
                else{
                    j= k+1;
                    flag = 1;
                    break;
                }
            }
        }
        if(flag == 0){
            for(int k = j;k>0;k--){
                if(graph[i][k]!=-2){
                    if(graph[i][k]!=type){
                        break;
                    }
                    else{
                        j= k-1;
                        flag = 1;
                        break;
                    }
                }
            }
        }
    }
    else flag = 1;
    // if(flag == 0)cerr<<"can not find in this level"<<endl;
    x = j * 0.5 + 0.25;
    y = i * 0.5 + 0.25;
    // cerr<<"virtual_target_type "<<i<<' '<<j<<' '<<x<<' '<<y<<' '<<graph[i][j]<<endl;
    return pair<double, double>(x, y);
 }

// void deal_graph(){
//     int count1,count2;
//     for(int i = 0;i<2;i++){
//         for(int j=0;j<100;j++){
//             for(int k =0 ;k<100;k++){
//                 graphs[i][j][k]=graph[j][k];
//             }
//         }
//     }
//     for(int i=0;i<100;i++){
//         count1 = 0;
//         count2 = 0;
//         for(int j=0;j<100;j++){
//             if(graphs[0][i][j]!=-2){
//                 count1++;
//             }
//             if(count1>0 && graphs[0][i][j] == -2){
//                 if(count1 < 2 )
//                     graphs[0][i][j-1]=-2;
//                     else count1 = 0;
//             }
//             if(graphs[1][i][j]!=-2){
//                 count2++;
//             }
//             if(count2>0 && graphs[1][i][j] == -2){
//                 if(count2 < 3 ){
//                     graphs[1][i][j-1]=-2;
//                     graphs[1][i][j-2]=-2;
//                 }
//                 else count2 = 0;
//             }
//         }
//     }
//     for(int j=0;j<100;j++){
//         count1 = 0;
//         count2 = 0;
//         for(int i=0;i<100;i++){
//             if(graphs[0][i][j]!=-2){
//                 count1++;
//             }
//             if(count1>0 && graphs[0][i][j] == -2){
//                 if(count1 < 2 )
//                     graphs[0][i-1][j]=-2;
//                     else count1 = 0;
//             }
//             if(graphs[1][i][j]!=-2){
//                 count2++;
//             }
//             if(count2>0 && graphs[1][i][j] == -2){
//                 if(count2 < 3 ){
//                     graphs[1][i-1][j]=-2;
//                     graphs[1][i-2][j]=-2;
//                 }
//                 else count2 = 0;
//             }
//         }
//     }
//     divide_space(0);
//     divide_space(1);
//     analyze_space(0);
//     analyze_space(1);
// }

// void divide_space(int is_take){
//     int i,j,k,type = 0,count1,count2,count;
//     int flag =0;
//     double first,second;
//     pair<double,double>temp;
//     for(i=0;i<100;i++){
//         for(j=0;j<100;j++){
//             // cerr << "ffff" << endl;
//             if(graphs[is_take][i][j]==-2)continue;
//             flag = 0;
//             // cerr<<"aaaa"<<endl;
//             if(i==0||graphs[is_take][i-1][j]==-2){
//                 // cerr<<"bbbb"<<endl;
//                 if(j==0||graphs[is_take][i][j-1]==-2){
//                     if (graphs[is_take][i][j - 1] == -2 &&j<98){
//                         if (graphs[is_take][i - 1][j + 1] != -2 ){ 
//                             flag = 1;
//                             if(check_side(i,j,1,is_take)){
//                                 graphs[is_take][i][j] = graphs[is_take][i - 1][j + 1];
//                                 continue;
//                             }
//                         }
//                         else if (graphs[is_take][i-1][j +2] != -2 ){
//                             flag = 2;
//                             if (check_side(i, j, 2,is_take))
//                             {
//                                 graphs[is_take][i][j] = graphs[is_take][i - 1][j + 2];
//                                 continue;
//                             }
//                         }
//                     }
//                     // cerr<<"111"<<endl;
//                     graphs[is_take][i][j] = type;
//                     // cerr<<type<<endl;
//                     type_area temp;
//                     temp.type = type;
//                     temp.height=1;
//                     types[is_take].push_back(temp);
//                     type ++;
//                 }
//                 else{
//                     graphs[is_take][i][j]=graphs[is_take][i][j-1];
//                 }
//             }
//             else{
//                 if(j==0||graphs[is_take][i][j-1]==-2){
//                     if(graphs[is_take][i-1][j] != -2){
//                         if(check_side(i,j,0,is_take)){
//                             graphs[is_take][i][j] = graphs[is_take][i - 1][j];
//                             types[is_take][graphs[is_take][i][j]].height++;
//                         }
//                         else{
//                             graphs[is_take][i][j]=type;
//                             type_area temp;
//                             temp.type = type;
//                             temp.height=1;
//                             types[is_take].push_back(temp);
//                                 // cerr<<type<<endl;
//                             type ++;
//                         }
//                     }
//                 }
//                 else{
//                     if(graphs[is_take][i][j-1]!=graphs[is_take][i-1][j] && graphs[is_take][i][j-1] != -2){
//                         graphs[is_take][i][j]=graphs[is_take][i][j-1];
//                     }
//                     else graphs[is_take][i][j]=graphs[is_take][i-1][j];
//                 }
//             }
//         }
//         // for(j=0;j<100;j++){
//         //     fprintf(stderr,"%4d",graph[i][j]);
//         // }
//         // cerr<<endl;
//     }
//     for(i=99;i>=0;i--){
//         for(j=0;j<100;j++){
//             fprintf(stderr,"%4d",graphs[is_take][i][j]);
//             // cerr<<graph[i][j]<<' ';
//         }
//         cerr<<endl;
//     }
//     cerr<<endl;
// }

// void analyze_space(int is_take){
//     int i,j,k,type = 0,count1,count2,count;
//     int flag =0;
//     double first,second;
//     pair<double,double>temp;
//     for(i=0;i<100;i++){
//         for(j=0;j<100;j++){
//             if(graphs[is_take][i][j]==-2)continue;
//             if(i>0&& graphs[is_take][i][j]!=graphs[is_take][i-1][j]&& graphs[is_take][i-1][j]!=-2){
//                 if(j==0 || graphs[is_take][i][j] !=graphs[is_take][i][j-1]|| graphs[is_take][i-1][j] !=graphs[is_take][i-1][j-1]){
//                     count = 1;
//                     for(k=j+1;k<100;k++){
//                         // cerr<<graph[i][k]<<' '<<graph[i][k-1]<<' '<<graph[i-1][k]<<' '<<graph[i-1][k-1]<<endl;
//                         if(graphs[is_take][i][k] !=graphs[is_take][i][k-1]|| graphs[is_take][i-1][k] != graphs[is_take][i-1][k-1]) break;
//                         count ++;
//                     }
//                     // cerr<<"count = "<<count<<endl;
//                     if(count>(is_take+2)){
//                         first = (j+count/2)*0.5+0.25;
//                         // cerr<<"type = "<<graph[i][j]<<endl;
//                         // cerr<<"type = "<<graph[i-1][j]<<endl;
//                         if(types[is_take][graphs[is_take][i][j]].height>2){
//                             second = ((i+1)*0.5+0.25);
//                             temp = check_wail_change(i+1,(j + count / 2), graphs[is_take][i][j]);
//                         }
//                         else {
//                             second = ((i)*0.5+0.25);
//                             temp = check_wail_change(i, (j + count / 2), graphs[is_take][i][j]);
//                         }
//                         if (types[is_take][graphs[is_take][i][j]].entrance.count(graphs[is_take][i-1][j]) == 0)
//                         {
//                             types[is_take][graphs[is_take][i][j]].entrance.insert({graphs[is_take][i-1][j],temp});
//                         }
//                         if(types[is_take][graphs[is_take][i-1][j]].height>2){
//                             second = ((i-2)*0.5+0.25);
//                             temp = check_wail_change(i - 2, (j + count / 2), graphs[is_take][i-1][j]);
//                         }
//                         else{
//                             second = ((i-1)*0.5+0.25);
//                             temp = check_wail_change(i - 1, (j + count / 2), graphs[is_take][i-1][j]);
//                         }
//                         if(types[is_take][graphs[is_take][i-1][j]].entrance.count(graphs[is_take][i][j])==0){
//                             types[is_take][graphs[is_take][i-1][j]].entrance.insert({graphs[is_take][i][j],temp});
//                         }
//                     }
//                     else{
//                         // cerr<<"graph[i][j] type = "<<graph[i][j]<<"graph[i-1][j] = " <<graph[i-1][j]<<endl;
//                         types[is_take][graphs[is_take][i-1][j]].entrance.erase(graphs[is_take][i][j]);
//                         types[is_take][graphs[is_take][i][j]].entrance.erase(graphs[is_take][i-1][j]);
//                     }
//                     // cerr<<"count = "<<count<<endl;
//                 }
//                 // if(types[graph[i][j]].entrance.count(graph[i-1][j])==0){
//                 //     types[graph[i][j]].entrance.insert({graph[i-1][j],pair<double,double>(0,0)});
//                 // }
//                 // if(types[graph[i-1][j]].entrance.count(graph[i][j])==0){
//                 //     types[graph[i-1][j]].entrance.insert({graph[i][j],pair<double,double>(0,0)});
//                 // }
//             }
//         }
//         // cerr<<i<<endl;
//     }
//     // for(i=0;i<types.size();i++){
//     //     cerr<<"type : "<<types[i].type<<' '<<types[i].entrance.size()<<endl;
//     //     for (auto iter = types[i].entrance.begin(); iter != types[i].entrance.end(); ++iter) {
//     //         cerr << iter->first << ' '<<iter->second.first<<' '<<iter->second.second<<' ';
//     //     }
//     //     cerr<<endl;
//     // }
//     floyd_area(is_take);
//     // for(int i=0;i<types.size();i++){
//     //     for(int j=0;j<types.size();j++){
//     //         cerr<<i<<' '<<j<<' '<< "dis = "<<dis_area[i][j]<<endl;
//     //     }
//     // }
//     // print_target(0, 93);
//     studio_distance(is_take);
// }

// void print_target(int i, int j,int is_take) {
//     int k;
//     int start = i;
//     double dist=0;
//     cerr<<i;
//     if(eq(dis_area[is_take][i][j], 1000)) return;
//     while(i != j) {
//         k = target_sequence[is_take][i][j];
//         cerr<<"->"<<k;
//         if(k==j) dist += dis_area[is_take][i][k];
//         else
//             dist += dis_area[is_take][i][k] + calcuDis(types[is_take][k].entrance[i], types[is_take][k].entrance[target_sequence[is_take][k][j]]);
//         // cerr<<"dist "<<start<<"->"<<k<<" = "<<dist<<endl;
//         i = k;
//     }
//     // cerr<<endl;
//     // cerr<<dist<<"*"<<dis_area[start][j]<<endl;
// }
// void init_area(int is_take){
//     for(int i=0;i<types[is_take].size();i++){
//         for(int j=i+1;j<types[is_take].size();j++){
//             if(types[is_take][i].entrance.count(types[is_take][j].type)!=0){
//                 dis_area[is_take][i][j]=calcuDis(types[is_take][i].entrance[types[is_take][j].type],types[is_take][types[is_take][j].type].entrance[i]);
//                 dis_area[is_take][j][i]=dis_area[is_take][i][j];
//                 target_sequence[is_take][i][j]=j;
//                 target_sequence[is_take][j][i]=i;
//             }
//             else{
//                 dis_area[is_take][i][j]=1000;
//                 dis_area[is_take][j][i]=dis_area[is_take][i][j];
//                 target_sequence[is_take][i][j]=-1;
//                 target_sequence[is_take][j][i]=-1;
//             }
//         }
//     }
// }
// void floyd_area(int is_take){
//     double dist;
//     init_area(is_take);
//     for(int k=0;k<types[is_take].size();k++){
//         for(int j= 0;j<types[is_take].size();j++){
//             for(int i=0;i<types[is_take].size();i++){
//                 if(j==k || i==k || i==j)continue;
//                 if(lt(dis_area[is_take][i][k], 1000) && lt(dis_area[is_take][k][j], 1000)){
//                     // cerr<<j<<endl;
//                     // printPair(types[i].entrance[j]);
//                     // cerr<<k<<endl;
//                     // printPair(types[i].entrance[k]);
                    
//                     dist = dis_area[is_take][i][k] + dis_area[is_take][k][j] + calcuDis(types[is_take][k].entrance[target_sequence[is_take][k][i]], types[is_take][k].entrance[target_sequence[is_take][k][j]]);
//                     if(gt(dis_area[is_take][i][j], dist)) {
//                         dis_area[is_take][i][j] = dis_area[is_take][j][i] = dist;
//                         target_sequence[is_take][i][j] = target_sequence[is_take][i][k];
//                         target_sequence[is_take][j][i] = target_sequence[is_take][j][k];
//                     }


//                     // dist = calcuDis(types[i].entrance[j],types[i].entrance[k]);
//                     // // cerr<<"dist = "<<dist<<endl;
//                     // if(gt(dis_area[j][k],dis_area[j][i]+dis_area[i][k]+dist)){
//                     //     dis_area[j][k] = dis_area[j][i]+dis_area[i][k]+dist;
//                     //     target_sequence[j][k]=target_sequence[j][i];
//                     // }
//                 }
//             }
//         }
//     }
// }
// void studio_distance(int is_take){
//     int row,col;
//     int x, y;
//     for(int i=0;i<studios.size();i++){
//         col = (studios[i].pos.first-0.25)/0.5;
//         row = (studios[i].pos.second-0.25)/0.5;
//         studios[i].studio_area_type[is_take] = graphs[is_take][row][col];
//         // cerr <<"kkk"<<studios[i].pos.first << ' ' << studios[i].pos.second << ' ' << col<<' '<<row<<' '<<studios[i].studio_area_type << endl;
//     }
//     for(int i=0;i<robots.size();i++){
//         col = (robots[i].pos.first-0.25)/0.5;
//         row = (robots[i].pos.second-0.25)/0.5;
//         // cerr<<col<<' '<<row<<endl;
//         robots[i].robot_area_type[0] = graphs[0][row][col];
//     }
//     for(int i=0;i<studios.size();i++){
//         x = studios[i].studio_area_type[is_take];
//         cerr << studios.size()<<endl;
//         for (int j = i + 1; j < studios.size(); j++)
//         {
//             y = studios[j].studio_area_type[is_take];
//             // cerr<<"a"<<endl;
//             if(lt(dis_area[is_take][studios[i].studio_area_type[is_take]][studios[j].studio_area_type[is_take]],1000)){
//                 // cerr << i << ' ' << j << endl;
//                 // cerr<<x<<' '<<y<<endl;
//                 if(studios[i].studio_area_type[is_take]!=studios[j].studio_area_type[is_take]){
//                     studio_dis[is_take][i][j] = calcuDis(studios[i].pos, types[is_take][x].entrance[target_sequence[is_take][x][y]]) + dis_area[is_take][x][y] + calcuDis(studios[j].pos, types[is_take][y].entrance[target_sequence[is_take][y][x]]);
//                 }
//                 else{
//                     studio_dis[is_take][i][j]=calcuDis(studios[i].pos,studios[j].pos);
//                 }
//                 // cerr << "b" << endl;
//             }
//             else{
//                 // cerr<<i<<' '<<j<<endl;
//                 studio_dis[is_take][i][j] = 1000;
//                 // cerr << "c" << endl;
//             }
//             studio_dis[is_take][j][i] = studio_dis[is_take][i][j];
//             // cerr << j << endl;
//         }
//     }
//     for(int i=0;i<robots.size();i++){
//         x = robots[i].robot_area_type[0];
//         for(int j=0;j<studios.size();j++){
//             y = studios[j].studio_area_type[is_take];
//             if(lt(dis_area[is_take][x][y],1000)){
//                 if(x != y){
//                     init_robot_dis[i][j] = calcuDis(robots[i].pos, types[is_take][x].entrance[target_sequence[is_take][x][y]]) + dis_area[is_take][x][y] + calcuDis(studios[j].pos, types[is_take][y].entrance[target_sequence[is_take][y][x]]);
//                 }
//                 else{
//                     init_robot_dis[i][j]=calcuDis(robots[i].pos,studios[j].pos);
//                 }
//             }
//             else{
//                 init_robot_dis[i][j] = 1000;
//             }
//         }
//     }
//     // for (int i = 0; i < studios.size(); i++)
//     // {
//     //     for(int j=0;j<studios.size();j++){
//     //         cerr<<i<<' '<<j<<' '<< "studio_distance = "<<studio_dis[i][j]<<endl;
//     //     }
//     // }

//     // cerr<<"type"<<robots[0].robot_area_type<<endl;

// }

void init_trans(){
    for(int i=0;i<100;i++){
        for(int j=0;j<100;j++){
           graph_trans[i][j]=( graph[i][j]==-2?-2:0);
        }
    }
}//将原来的地图中不是-2的部分全部更改为0
double Angle_conversion(double angle){
    return fabs(angle)/Pi;
}//将角度转换为距离
bool check_4(int i,int j){
    if(i<0||j<0||i>99||j>99)return false;
    if(i+1>99||j-1<0)return false;
    return graph_trans[i][j]!=-2&&graph_trans[i][j-1]!=-2&&graph_trans[i+1][j-1]!=-2&&graph_trans[i+1][j]!=-2;
}//检查坐标i,j是否是一个四个格子的合法点
pair<int,pair<double,double>> check_8(int i,int j){
    if(check_4(i,j)&&check_4(i,j+1)&&check_4(i-1,j)&&check_4(i-1,j+1)){
        return {1,make_pair<double,double>(0.5*j+0.25,0.5*i+0.25)};
    }else if((!check_4(i,j))&&check_4(i,j+1)&&check_4(i-1,j)&&check_4(i-1,j+1)){
        return {2,make_pair<double,double>(0.5*j+0.47,0.5*i+0.03)};
    }else if(check_4(i,j)&&check_4(i,j+1)&&(!check_4(i-1,j))&&check_4(i-1,j+1)){
        return {3,make_pair<double,double>(0.5*j+0.47,0.5*i+0.47)};
    }else if(check_4(i,j)&&(!check_4(i,j+1))&&check_4(i-1,j)&&check_4(i-1,j+1)){
        return {4,make_pair<double,double>(0.5*j+0.03,0.5*i+0.03)};
    }else if(check_4(i,j)&&check_4(i,j+1)&&check_4(i-1,j)&&(!check_4(i-1,j+1))){
        return {5,make_pair<double,double>(0.5*j+0.03,0.5*i+0.47)};
    }else{
        return {0,make_pair<double,double>(0,0)};
    }
    
}//检查坐标i,j是否是一个八个格子的合法点
void Translation_graph_no(){
    for(int i=0;i<100;i++){
        for(int j=0;j<100;j++){
            if(check_4(i,j)){
                int id=100*i+j;
                auto pos=make_pair<double,double>(0.5*j,0.5*i+0.5);
                exist_id[0][id]=pos;
                for(int t=0;t<studios.size();t++){
                    double tmpDis=calcuDis(studios[t].pos,pos);
                    if(le(tmpDis,0.4)&&id!=studios[t].node_id){
                        double dis= (abs(i-(studios[t].node_id/100))+abs(j-(studios[t].node_id%100))==2)?pow(2,0.5):1;
                        graph_edge[0][id].push_back(Graph_node(studios[t].node_id,dis,id));
                        graph_edge[0][studios[t].node_id].push_back(Graph_node(id,dis,studios[t].node_id));
                        // if(t==0)cerr<<graph_edge[0][studios[t].node_id].size()<<endl;
                    }
                }
                for(int t=0;t<4;t++){
                    double tmpDis=calcuDis(robots[t].pos,pos);
                    if(le(tmpDis,0.4)&&id!=robots[t].node_id){
                        double dis= (abs(i-(robots[t].node_id/100))+abs(j-(robots[t].node_id%100))==2)?pow(2,0.5):1;
                        graph_edge[0][id].push_back(Graph_node(robots[t].node_id,dis,id));
                        graph_edge[0][robots[t].node_id].push_back(Graph_node(id,dis,robots[t].node_id));
                    }
                }
            }
        }
    }
}
void Translation_graph_has(){
    for(int i=0;i<100;i++){
        for(int j=0;j<100;j++){
            auto tmp=check_8(i,j);
            auto pos=tmp.second;
            int id=100*i+j;
            if(tmp.first!=0){
                exist_id_type[1][id]=tmp.first;
                exist_id[1][id]=tmp.second;
                for(int t=0;t<studios.size();t++){
                    double tmpDis=calcuDis(studios[t].pos,pos);
                    if(!is_corner(studios[t].node_id)&&((le(tmpDis,0.71)&&tmp.first==1)||le(tmpDis,0.4))&&id!=studios[t].node_id){
                        double dis= (abs(i-(studios[t].node_id/100))+abs(j-(studios[t].node_id%100))==2)?pow(2,0.5):1;
                        graph_edge[1][id].push_back(Graph_node(studios[t].node_id,dis,id));
                        graph_edge[1][studios[t].node_id].push_back(Graph_node(id,dis,studios[t].node_id));
                    }
                }
                for(int t=0;t<4;t++){
                    double tmpDis=calcuDis(robots[t].pos,pos);
                    if(!is_corner(studios[t].node_id)&&((le(tmpDis,0.71)&&tmp.first==1)||le(tmpDis,0.4))&&id!=robots[t].node_id){
                        double dis= (abs(i-(robots[t].node_id/100))+abs(j-(robots[t].node_id%100))==2)?pow(2,0.5):1;
                        graph_edge[1][id].push_back(Graph_node(robots[t].node_id,dis,id));
                        graph_edge[1][robots[t].node_id].push_back(Graph_node(id,dis,robots[t].node_id));
                    }
                }
            }
        }
    }    
}//转换机器人带物品的原始图
void getEdgeRalative(){
    for(auto& it:exist_id[0]){
        int idi=it.first/100;
        int idj=it.first-idi*100;
        for(int i=idi-1;i<=idi+1;i++){
            for(int j=idj-1;j<=idj+1;j++){
                if(i==idi&&j==idj)continue;
                int tmpId=i*100+j;
                int ckeck_id=idi*100+j;
                // if(it.first==studios[0].node_id){
                //     cerr<<graph_edge[0][it.first].size()<<" "<<exist_id[0].count(tmpId)<<" "<<exist_id[0].count(ckeck_id)<<endl;
                //     cerr<<idi<<"-"<<idj<<" "<<i<<"-"<<j<<endl;
                // }
                // if((it.first==4656&&tmpId==4557) ||(it.first==4557&&tmpId==4656) ){
                //     cerr<<"****"<<endl;
                //     cerr<<check_slope(tmpId,it.first)<<endl;
                //     cerr<<idi<<" "<<j<<endl;
                //     cerr<<graph_trans[idi][j+1]<<endl;
                //     cerr<<"****"<<endl;
                // }
                bool isSlope=  (fabs(i-idi)+fabs(j-idj)==2)?true:false;
                int SlopeCheckId1=idi*100+j;
                int SlopeCheckId2=i*100+idj;
                //不带货物时，不考虑是否在墙角。
                if(exist_id[0].count(tmpId)&&check_slope(tmpId,it.first) &&it.first!=tmpId){
                    double dis= (abs(i-idi)+abs(j-idj)==2)?pow(2,0.5):1;
                    graph_edge[0][it.first].push_back(Graph_node(tmpId,dis,it.first));
                }
            }
        }
    }
    for(auto& it:exist_id[1]){
        int idi=it.first/100;
        int idj=it.first-idi*100;
        for(int i=idi-1;i<=idi+1;i++){
            for(int j=idj-1;j<=idj+1;j++){
                if(i==idi&&j==idj)continue;
                int tmpId=i*100+j;
                if(is_corner(it.first)||is_corner(tmpId)){
                    continue;
                }
                int ckeck_id=idi*100+j;
                bool isSlope= (fabs(i-idi)+fabs(j-idj)==2)?true:false;
                bool isInCorner=(exist_id_type[1][tmpId]>1||exist_id_type[1][it.first]>1);
                int SlopeCheckId1=idi*100+j;
                int SlopeCheckId2=i*100+idj;
                //带货物时，考虑是否在墙角。
                if(isInCorner&&isSlope&&(exist_id[1].count(SlopeCheckId1)||exist_id[1].count(SlopeCheckId2))){

                }else if(exist_id[1].count(tmpId)&&it.first!=tmpId&&check_slope(tmpId,it.first)){
                    double dis= (abs(i-idi)+abs(j-idj)==2)?pow(2,0.5):1;
                    graph_edge[1][it.first].push_back(Graph_node(tmpId,dis,it.first));
                }
            }
        }
    }
}

void init_vector() {
    // vector<int> next_node[50][2];//next_node[node_id][studio_id][2]:node_id去往studio_id工作台的下一个点
    // vector<double> dis_to_studios[50][2];
    for(int i = 0; i < studios.size(); ++i) {
        next_node[i][0].resize(10000, -1);
        next_node[i][1].resize(10000, -1);
        dis_to_studios[i][0].resize(10000, 0);
        dis_to_studios[i][1].resize(10000, 0);
    }
}

double calAngleToDis(int x, int y, int z) {
    if(x == y) return 0;
    if(y == z) return 0;

    Vec vec1 = Vec((y / 100) - (x / 100), (y % 100) - (x % 100));
    Vec vec2 = Vec((z / 100) - (y / 100), (z % 100) - (y % 100));

    double angle = acos(cos_t(vec1, vec2));
    return Angle_conversion(angle);
}

int transID(int from_id, int is_robot, int to_id) {
    return (from_id + is_robot * 50) * 54 + to_id;
}

void Dijkstra(int studio_id, int is_take) {
    priority_queue<Graph_node, vector<Graph_node>, cmp_Graph_node> q;
    int from, pre_id, num, i, to;
    double dis, new_dis, angle_sum;
    int s;
    int vis_node[10000];
    double angle_node[10000];

    s = studios[studio_id].node_id;

    bool cerr_flag = false;
    // if(studio_id == 0 && is_take == 1) cerr_flag = true;

    if(cerr_flag) {
        cerr<<"start-studio:"<<studio_id<<endl;
    }

    for(i = 0; i < 10000; ++i) {
        vis_node[i] = 0;
        dis_to_studios[studio_id][is_take][i] = 10000;
    }

    q.push(Graph_node(s, 0, s));
    dis_to_studios[studio_id][is_take][s] = 0;
    next_node[studio_id][is_take][s] = s;
    
    while(!q.empty()) {
        Graph_node now_node = q.top();
        q.pop();
        if(vis_node[now_node.id]) continue;
        from = now_node.id;
        dis = now_node.dis;
        pre_id = now_node.pre_id;
        vis_node[now_node.id] = 1;

        if(cerr_flag) 
            cerr<<"node_id:"<<from<<" dis:"<<dis<<" pre_id:"<<pre_id<<endl;


        num = graph_edge[is_take][from].size();
        if(cerr_flag) 
            cerr<<"edge-num:"<<num<<endl;
        for(i = 0; i < num; ++i) {
            to = graph_edge[is_take][from][i].id;
            if(vis_node[to] || to == pre_id) continue;

            angle_sum = now_node.angle_sum + calAngleToDis(pre_id, from, to);
            new_dis = dis + graph_edge[is_take][from][i].dis;
            // cerr<<"to_id:"<<to<<" new-dis:"<<dis<<" old-dis:"<<dis_node[to]<<endl;
            if(lt(new_dis, dis_to_studios[studio_id][is_take][to]) || (eq(new_dis, dis_to_studios[studio_id][is_take][to]) && lt(angle_sum, angle_node[to]))) {
                q.push(Graph_node{to, new_dis, from, angle_sum});
                if(cerr_flag) cerr<<"update-to_id:"<<to<<" new-dis:"<<new_dis<<" old-dis:"<<dis_to_studios[studio_id][is_take][to]<<endl;
                dis_to_studios[studio_id][is_take][to] = new_dis;
                next_node[studio_id][is_take][to] = from;
                angle_node[to] = angle_sum;
            }
            // else if(is_take && eq(new_dis, dis_to_studios[to][studio_id][is_take]) && eq(angle_sum, angle_node[to])) {
            //     q.push(Graph_node{to, new_dis, from, angle_sum});
            //     if(cerr_flag) cerr<<"update-to_id:"<<to<<" new-dis:"<<new_dis<<" old-dis:"<<dis_to_studios[to][studio_id][is_take]<<endl;
            //     dis_to_studios[to][studio_id][is_take] = new_dis;
            //     next_node[to][studio_id][is_take] = from;
            //     angle_node[to] = angle_sum;
            // }
        }
    }
}

int trans_pos_to_nodeID(pair<double, double> pos) {
    return (int)(pos.second / 0.5) * 100 + (int)(pos.first / 0.5);
}

int trans_pos_to_nodeID(int robot_id) {
    return trans_pos_to_nodeID(robots[robot_id].pos);
}

bool is_connected(int node_id_a, int node_id_b, int is_take) {
    return get_bar_num(node_id_a, node_id_b, is_take) == 0;
}

int get_bar_num(int node_id_a, int node_id_b, int is_take) {
    int x1, x2, y1, y2;
    x1 = min(node_id_a / 100, node_id_b / 100);
    x2 = max(node_id_a / 100, node_id_b / 100);
    y1 = min(node_id_a % 100, node_id_b % 100);
    y2 = max(node_id_a % 100, node_id_b % 100);
    if(x2 == 0 && y2 == 0)
        return bar_sum[x2][y2][is_take];
    if(x2 == 0)
        return bar_sum[x2][y2][is_take] - bar_sum[x2][y1][is_take];
    if(y2 == 0)
        return bar_sum[x2][y2][is_take] - bar_sum[x1][y2][is_take];
    return bar_sum[x2][y2][is_take] - bar_sum[x2][y1][is_take] - bar_sum[x1][y2][is_take] + bar_sum[x1][y1][is_take];
}

void init_bar_sum() {
    for(int stu_id = 0; stu_id < studios.size(); ++stu_id) {
        for(int i = 0; i < 100; ++i) {
            // if(stu_id != 0 && )
            for(int j = 0; j < 100; ++j) {
                if(stu_id == 0) {
                    bar_sum[i][j][0] = (next_node[stu_id][0][i * 100 + j] == -1);
                    bar_sum[i][j][1] = (next_node[stu_id][1][i * 100 + j] == -1);
                }
                else {
                    bar_sum[i][j][0] = min(bar_sum[i][j][0], (int)(next_node[stu_id][0][i * 100 + j] == -1));
                    bar_sum[i][j][0] = min(bar_sum[i][j][0], (int)(next_node[stu_id][0][i * 100 + j] == -1));
                }
                if(i > 0) {
                    bar_sum[i][j][0] += bar_sum[i - 1][j][0];
                    bar_sum[i][j][1] += bar_sum[i - 1][j][1];
                }
                if(j > 0) {
                    bar_sum[i][j][0] += bar_sum[i][j - 1][0] - bar_sum[i - 1][j - 1][0];
                }
            }
        }
    }
}


void print_dijkstra(int studio_id, int is_take, int is_path) {
    for(int i = 0; i < 100; ++i) {
        for(int j = 0; j< 100; ++j) {
            if(is_path) {
                if(next_node[studio_id][is_take][(99-i) * 100 +j] == -1)
                    cerr<<"- ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-i) * 100 +j + 1)
                    cerr<<"→ ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-i) * 100 +j -1)
                    cerr<<"← ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-(i+1)) * 100 +j)
                    cerr<<"↓ ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-(i-1)) * 100 +j)
                    cerr<<"↑ ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-(i+1)) * 100 +j-1)
                    cerr<<"↙️ ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-(i+1)) * 100 +j+1) 
                    cerr<<"↘️ ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-(i-1)) * 100 +j+1) 
                    cerr<<"↗️ ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-(i-1)) * 100 +j-1) 
                    cerr<<"↖️ ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-i) * 100 +j)
                    cerr<<"* ";
                else cerr<<(99-i) * 100 +j<<"-"<<next_node[studio_id][is_take][(99-i) * 100 +j]<<" ";
                    
            }
            else 
                cerr<<setw(10)<<dis_to_studios[studio_id][is_take][(99-i) * 100 +j]<<" ";
        }
        cerr<<endl;
    }
}

bool is_corner(int id){
    int i=id/100;
    int j=id-i*100;
    bool leg1= (j==0||graph_trans[i][j-1]==-2)?true:false;
    bool leg2= (i==99||graph_trans[i+1][j]==-2)?true:false;
    bool leg3= (j==99||graph_trans[i][j+1]==-2)?true:false;
    bool leg4= (i==0||graph_trans[i-1][j]==-2)?true:false;
    return (leg1&&leg2) || (leg2&&leg3) || (leg3&&leg4) || (leg4&&leg1);
}//判断工作台是不是在墙角
void init_data(){
    init_trans();
    Translation_graph_no();
    Translation_graph_has();
    getEdgeRalative();
    // trans_studio_rob_toID();
    for(int j=0;j<100;j++){
        for(int i=1;i<100;i++){
            int id=i*100+j;
            sum_matrix[0][i][j]=sum_matrix[0][i-1][j]+ (exist_id[0].count(id));
            sum_matrix[1][i][j]=sum_matrix[1][i-1][j]+ (exist_id[1].count(id));
        }
    }
}
void printMap(int f){
    // for(int i=0;i<100;i++){
    //     cerr<<i<<" ";
    // }
    cerr<<endl;
        for(int i=100;i>=0;i--){
        for(int j=0;j<100;j++){
            int id=i*100+j;
            if(exist_id[f].count(id)){
                if(f==1)
                cerr<<check_8(i,j).first<<" ";
                else
                cerr<<1<<" ";
            }else{
                cerr<<"-"<<" ";
            }
        }
        cerr<<endl;
    }
}
void printEdge(int id){
    vector<int> vis_rob_edge(100*100,-1);
    // cerr<<studios[4].node_id/100<<"-"<<(studios[4].node_id-studios[4].node_id/100*100)<<endl;
    cerr<<studios[5].node_id<<endl;
    //unordered_map<int,vector<Graph_node>> graph_edge[2];
    queue<int>Q;
    Q.push(robots[id].node_id);
    int cnt=0;
    vis_rob_edge[robots[id].node_id]=0;
    while(!Q.empty()){
        int tmpId=Q.front();
        Q.pop();
        for(auto it:graph_edge[0][tmpId]){
            if(vis_rob_edge[it.id]==-1){
                 Q.push(it.id);
                 vis_rob_edge[it.id]=(vis_rob_edge[tmpId]+1)%10;
            }
            
        }
    }
    int tarStu=4;
    // vector<Graph_node> path = road[0][transID(3, 0, tarStu)];
    // for(auto it:path){
    //     int tmpId=it.id;
    //     if(vis_rob_edge[tmpId]==-1){
    //         cerr<<" 错误 "<<endl;
    //     }
    //     vis_rob_edge[tmpId]=-7;
       
        
    // }
    cerr<<"edge-print "<<endl;
    // for(int i=0;i<100;i++){
    //     cerr<<i<<" ";
    // }
    // cerr<<endl;
    for(int i=99;i>=0;i--){
    for(int j=0;j<100;j++){
            int id=i*100+j;
            if(vis_rob_edge[id]!=-1&&vis_rob_edge[id]!=-7&&id!=studios[tarStu].node_id){
                cerr<<vis_rob_edge[id]<<" ";
            }else if(vis_rob_edge[id]==-7&&id!=studios[tarStu].node_id){
                cerr<<"+"<<" ";
            }else if(id==studios[tarStu].node_id){
                cerr<<"^"<<" ";
            }else {
                cerr<<"-"<<" ";
            }
        }
        cerr<<endl;
    }
}
bool check_slope(int id1,int id2){
    int y1=id1/100,y2=id2/100;
    int x1=id1-y1*100,x2=id2-y2*100;
    id1=y2*100+x1;
    id2=y1*100+x2;
    return exist_id[0].count(id1)&&exist_id[0].count(id2);
}

void printPath(int from_id, int is_robot, int to_id, int is_take) {
    int id, x, y;
    // vector<Graph_node> path = road[is_take][transID(from_id, is_robot, to_id)];
    // for(int i =0;i< path.size(); ++i) {
    //     id = path[i].id;
    //     x = id / 100;
    //     y = id % 100;
    //     cerr << "to:(" << x <<", "<<y<<")"<<endl;
    // }
    // cerr<<endl;
}

vector<int> get_future_node(int robot_id) {
    vector<int> v;
    int t = 2;
    int node_id = robots[robot_id].node_id;
    while(t--) {
        node_id = next_node[robots[robot_id].target_id][(robots[robot_id].get_type != 0)][node_id];
        v.emplace_back(node_id);
    }
    // cerr<<"kkk"<<v.size()<<endl;
    return v;
}
bool is_need_slow(Robot& robot,pair<double,double> pos,pair<double,double> pos1){
    if(lt(pos1.first,0))return false;
    auto p1=subVector(robot.virtual_pos,robot.pos);
    auto p2=subVector(pos,robot.pos);
    auto p3=subVector(pos1,robot.pos);
    auto v=robot.xy_pos;
    Vec v_p1(p1);
    Vec v_p2(p2);
    Vec v_p3(p3);
    Vec v_v(v);
    double cmpNums1= cos_t(v_v,v_p1),cmpNums2= cos_t(v_v,v_p2),cmpNums3= cos_t(v_v,v_p3);
    // if(contr_print_flag&&state.FrameID>=1354&&state.FrameID<=1450&&robot.id==0){
    //     cerr<<"cmpAngle "<<cmpNums1<<" "<<cmpNums2<<endl;
    // }
    if(ge(cmpNums1,0.9)&&ge(cmpNums2,0.8)&&ge(cmpNums3,0.8))return false;
    return true;
}
void adjust_virtual_pos(Robot& robot){
    if(robot.get_type==0|| exist_id_type[1][robot.virtual_id]!=1){
        return;
    }
    int now_j= robot.pos.first/0.5;
    int now_i= robot.pos.second/0.5;
    int tar_i=robot.virtual_id/100;
    int tar_j=robot.virtual_id-100*tar_i;
    if(fabs(now_i-tar_i)+fabs(now_j-tar_j)!=2)return;
    int tmpID1=now_i*100+tar_j,tmpID2=tar_i*100+now_j;
    double arr[][2]={{0,0},{0,0},{0,-0.2},{0.2,0},{0,-0.2},{-0.2,0}};
    if(exist_id_type[1][tmpID1]!=1&&exist_id_type[1][tmpID2]!=1){
        return;
    }else if(exist_id_type[1][tmpID1]!=1){
        int type=exist_id_type[1][tmpID1];
        robot.virtual_pos.first=exist_id[1][robot.virtual_id].first+arr[type][0];
        robot.virtual_pos.second=exist_id[1][robot.virtual_id].second+arr[type][1];
    }else if(exist_id_type[1][tmpID2]!=1){
        int type=exist_id_type[1][tmpID2];
        robot.virtual_pos.first=exist_id[1][robot.virtual_id].first+arr[type][0];
        robot.virtual_pos.second=exist_id[1][robot.virtual_id].second+arr[type][1];
    }
    return ;
}
void adjust_virtual_pos_total(Robot& rob){
    setVirPos(rob);
    adjust_virtual_pos(rob);
}
bool check_can_arrival(int istake,int id1,int id2){
    if(exist_id[1].count(id1)==0||exist_id[1].count(id2)==0)return false;
    int i1=min(id1/100,id2/100),i2=max(id1/100,id2/100);
    int j1=min(id1-id1/100*100,id2-id2/100*100),j2=max(id1-id1/100*100,id2-id2/100*100);
    // if(!eq(i1-i2,0)&&!eq(j1-j2,0)){
    //     i1=max(i1-1,0);
    //     i2=min(i2+1,99);
    //     j1=max(j1-1,0);
    //     j2=min(j2+1,99);
    // }
    if(i1<0||j1<0||i2<0||j2<0){
        // cerr<<" 错误"<<endl;
        return false;
    }
    for(int j=j1;j<=j2;j++){
        int tmp=sum_matrix[1][i2][j]-(i1>0?sum_matrix[1][i1-1][j]:0);
        if(tmp<(i2-i1+1)){
            
            return false;
        }
    }

    return true;

}
set<int> getEqID(int istake,int id1) {
    int i1=id1/100;
    int j1=id1-i1*100;
    set<int> ans;
    for(int i=i1-2;i<=i1+2;i++){
        for(int j=j1-2;j<=j1+2;j++){
            int tmpID=i*100+j;
            if(check_can_arrival(istake,id1,tmpID)){
                ans.insert(tmpID);
            }
        }
    }
    return ans;
}
void setVirPos(Robot& robot){
    int tarID=robot.target_id;
    int istake=robot.get_type==0?0:1;

    int now_id=robot.node_id;
    int cnt=robot.cnt_tar;
   
    for(int i=ret_next(robot,cnt);i!=ret_next(robot,i) && i!=-1;i=ret_next(robot,i)){
        int tmpId=i;
 
        // if(i<0)cerr<<"i<0错误"<<endl;
        if(check_can_arrival(1,now_id,tmpId)){
            if(robot.id==0){
                // cerr<<" 合并的tar "<<robot.cnt_tar<<" ";
                // printPair(exist_id[istake][robot.cnt_tar]);
            }
            robot.cnt_tar=tmpId;

        }else{
            if(robot.id==0){
                // cerr<<endl<<state.FrameID<<" 更新后的tar "<<robot.cnt_tar<<endl;
                // printPair(exist_id[istake][robot.cnt_tar]);
            }
            
            break;
        }
        
    }
  
   
    int tar1=robot.cnt_tar;

    auto virPos=exist_id[istake][tar1];
    auto virID=getPosID(virPos);
    robot.isVir=false;
    bool con1=false;
    if(at_least_three(robot,tar1)){
        con1=true;
        int tar2=next_node[tarID][istake][robot.cnt_tar];
        int tar3=next_node[tarID][istake][tar2];
        auto set1=getEqID(istake,tar1);
        auto set2=getEqID(istake,tar2);
        auto set3=getEqID(istake,tar3);
        set<int> tmpSet;
        vector<int> ansSet;
        set_intersection(set1.begin(),set1.end(),set2.begin(),set2.end(),inserter(tmpSet,tmpSet.begin()));
        set_intersection(tmpSet.begin(),tmpSet.end(),set3.begin(),set3.end(),inserter(ansSet,ansSet.begin()));
        virPos=select_visPos(robot,ansSet,tar3);
    }
    robot.virtual_pos=virPos;
    robot.virtual_id=virID;
}
pair<double,double>select_visPos(Robot& robot,vector<int> range,int tar3){
    int now_j= robot.pos.first/0.5;
    int now_i= robot.pos.second/0.5;
    int now_id=robot.node_id;
    int tarID=robot.target_id;
    int istake=robot.get_type==0?0:1;
    int tar1=robot.cnt_tar;
    if(exist_id[istake].count(tar1)==0){
        cerr<<"路径选点错误 "<<endl;
    }
    auto virPos=exist_id[istake][tar1];
    for(auto it: range){
        if(check_can_arrival(istake,now_id,it)){
            robot.isVir=true;
            robot.virtual_id=it;
            return exist_id[istake][it];;
        }
    }
    return virPos;
}
int ret_next(Robot& robot,int tar_cnt){
    int tarID=robot.target_id;
    if(tarID==-1)cerr<<"tarID==-1错误"<<endl;
    if(robot.target_id==-1)cerr<<"robot.target_id==-1错误"<<endl;
    int istake=robot.get_type==0?0:1;
    return next_node[tarID][istake][tar_cnt];
}
bool at_least_three(Robot& robot,int tar_cnt){
    int cnt1=3;
    int cnt=robot.cnt_tar;
    
    for(int i=ret_next(robot,cnt);i!=ret_next(robot,i);i=ret_next(robot,i)){
        cnt1--;
        if(cnt1==0)return true;
    }
    return false;
}
bool calMinAngle(Robot& robot,pair<double,double>pos){
    int now_j= robot.pos.first/0.5;
    int now_i= robot.pos.second/0.5;
    int tar_j=pos.first/0.5;
    int tar_i=pos.second/0.5; 
    auto p1=make_pair<double,double>(0,tar_j-now_j);
    auto p2=make_pair<double,double>(tar_i-now_i,0);
    auto p3=make_pair<double,double>(cos(robot.direction),sin(robot.direction));
    Vec v1(p1);
    Vec v2(p2);
    Vec v3(p3);
    if(lt(cos_t(v3,v1),0)|| lt(cos_t(v2,v1),0))return false;
    return true;;
}
double vir_v(Robot rob){
   for(int v=5;v>=0;v--){
    if(can_trajectory_virpos(rob,v,50))return v;
   }
   return 0.5;
}
bool can_trajectory_virpos(Robot rob,double v,int cnt){
    double t=0.02;
    double v_next=v;
    for(int i=0;i<cnt;i++){
        auto pay=calPayload(rob,rob.virtual_pos);
        if(checkNearBar(rob.pos,pay.radius))return false;
        auto tmpPair=get_w_now(rob,pay);
        double w_next=tmpPair.first;
        if(tmpPair.second|| lt(pay.angle,0.3)){
            int tarID=getPosID(rob.virtual_pos);
            int now_id=getPosID(rob.pos);
            int istake=rob.get_type==0?0:1;
            if(check_can_arrival(istake,now_id,tarID)){
                if(rob.id==0){
                    // cerr<<now_id<<" "<<tarID<<endl;
                    // cerr<<"检查到的对齐点: \n";
                    // printPair(rob.pos);
                    // printPair(rob.virtual_pos);
                    // cerr<<state.FrameID+ i<<endl;
                }

                return true;
            }
            else
                return false;
        }
        double seta=rob.direction;
        double w=rob.angular_velocity==0?0.00001:rob.angular_velocity;
        double a=return_ac(pay.angular_acceleration,rob.angular_velocity,w_next);
        double changeAngle=get_at_v_limt(t,pay.angular_acceleration,rob.angular_velocity,w_next,pay.sign)*pay.sign;
        double v=Calculate_the_projection_speed(rob);
        double a_v=return_ac(pay.acceleration,v,v_next);
        rob.pos.first=rob.pos.first+v*cos(seta+changeAngle/2)*t;
        rob.pos.second=rob.pos.second+v*sin(seta+changeAngle/2)*t;
        int sign1=ge((rob.angular_velocity+a*t)*w_next,0)?1:-1;
        int sign2=ge((rob.angular_velocity+a*t),0)?1:-1;
        double limit_w=0.0;
        if(lt(a,0)){
            limit_w=lt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
        }else{
            limit_w=gt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
        }
        rob.angular_velocity=limit_w;
        int signv_1=ge((v+a_v*t)*v_next,0)?1:-1;
        int signv_2=ge((v+a_v*t),0)?1:-1;
        double limit_v=gt(fabs(v+a_v*t),fabs(6))?6*sign2:v+a_v*t;
        if(lt(a_v,0)){
            limit_v=lt(v+a_v*t,v_next)?v_next:v+a_v*t;
        }else{
            limit_v=gt(v+a_v*t,v_next)?v_next:v+a_v*t;
        }
        v=limit_v;
        double xy_angle=get_Angle_xy(rob);
        rob.xy_pos.first=v*cos(xy_angle);
        rob.xy_pos.second=v*sin(xy_angle);
        double xy_angle_next=get_Angle_xy(rob);
        double cal_angle=xy_angle_next-xy_angle;
        vector<vector<double>>mat(4,vector<double>(4,0));
        cal_matrix(mat,changeAngle,cal_angle);
        rob.direction+=changeAngle;
        rob.direction=rob.direction>Pi?rob.direction-2*Pi:rob.direction; 
        double t1=rob.xy_pos.first,t2=rob.xy_pos.second;
        rob.xy_pos.first=(t1*mat[0][0]+t2*mat[0][1]);
        rob.xy_pos.second=(t1*mat[1][0]+t2*mat[1][1]);
    }  
    int tarID=getPosID(rob.virtual_pos);
    int now_id=getPosID(rob.pos);
    int istake=rob.get_type==0?0:1;
    if(check_can_arrival(istake,now_id,tarID)){
        // if(rob.id==0){
        // cerr<<"未检查到对齐"<<endl;
        // }
        return true;
    }    
    else{
        return false;
    }
        
}
int getPosID(pair<double,double>pos){
    return pos.second/0.5*100+pos.first/0.5;
}