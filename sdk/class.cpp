#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include<algorithm>
#include"vec.h"
#include "class.h"
// #include "line.h"

using namespace std;
vector<vector<double>> dis(50, vector<double>(50, 0));
vector<Studio> studios;
vector<Robot> robots;
State state;//当前帧数，全局可见
vector<Ins> ins(4);
vector<int> material[8];
vector<int> product[8];
vector<int> full_product;
double EPS=1e-7;
double acceleration_no;
double acceleration_has;
double angular_acceleration_no;
double angular_acceleration_has;
vector<bool> need_stop(4,false);
int robot_get_type[8];
int studios_rid[50][8];
int studio_material[4][4];
int studio_level[5][2];
int material_send[8][3];
int RootFlag=-2;
int Adjust_the_same_direction[4][2];
pair<double ,double> Root;
pair<double ,double> Collision_point;
vector<PayLoad> pl_g;
double Compute_redundancy=0;
void initRobortInfo() {
    double weightMin = 0.45 * 0.45 * Pi * 20.0;
    double weightMax = 0.53 * 0.53 * Pi * 20.0;
    double inertiaMin = weightMin * 0.45 * 0.45;
    double inertiaMax = weightMax * 0.53 * 0.53;

    acceleration_no = 250.0 / weightMin;
    acceleration_has = 250.0 / weightMax;

    angular_acceleration_no = 50.0 / inertiaMin;
    angular_acceleration_has = 50.0 /inertiaMax;


}
void init_studio_parameter(){
    for(int i=0;i<50;i++){
        for(int j=0;j<8;j++)studios_rid[i][j]=-1;
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


}
bool readMapUntilOK() {
    char line[1024];
    int count = 0;
    int count_robot = 0, count_studio = 0;
    double x,y;
    int i;
    while (cin.getline(line,sizeof(line))) {
        if (line[0] == 'O' && line[1] == 'K') {
            calcuStudioDis();
            return true;
        }
        //do something
        for(i=0;i<100;i++){
            if(line[i] == 'A'){
                x = i*0.5+0.25;
                y = (100-count)*0.5-0.25;
                pair<double,double>pos_robot(x,y);
                pair<double,double>xy_pos_robot(0,0);
                // cout<<x<<" "<<y<<endl;
                Robot  robot(count_robot,0,0,0,1,1,xy_pos_robot,0,pos_robot,-1);
                robots.push_back(robot);
                count_robot++;
            }
            else if(line[i] >= '1' && line[i] <= '9'){
                x = i*0.5+0.25;
                y = (100-count)*0.5-0.25;
                pair<double,double>pos_studio(x,y);
                // cout<<x<<" "<<y<<endl;
                Studio studio(count_studio,0,-1,pos_studio,0,0,0);
                studio.type = (int)line[i]-48;
                studios.push_back(studio);
                count_studio++;
            }
        }
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
    for(int i=0;i<8;++i) {
        material[i].clear();
        product[i].clear();
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
        }
        if(studios[studio_id].type > 3){
            if(studios[studio_id].type < 8){
                for(int i = 0;i < 4;i++){
                    if(studios[studio_id].type == i+4){
                        for(int j = 0;j<studio_material[i][0];j++){
                            if((studios[studio_id].bitSatus & (int)pow(2,studio_material[i][j+1])) == 0){
                                // if(studios_rid[studio_id][studio_material[i][j+1]] == -1)material[studio_material[i][j+1]].push_back(studio_id);
                                material[studio_material[i][j+1]].push_back(studio_id);
                            }
                        }
                    }
                }
            }
            if(studios[studio_id].type == 8){
                material[7].push_back(studio_id);
            }
            if(studios[studio_id].type == 9){
                for(int h = 1;h <=7;h++){
                    material[h].push_back(studio_id);
                }
            }
        }
        studio_id++;
    }
    for(int i=0;i<4;i++){
        vector<double> tmp(10,0);
        for(int i=0;i<tmp.size();i++){
            cin>>tmp[i];
        }
        robots[rob_id].collision_val_pre=robots[rob_id].collision_val;
        robots[rob_id].set(rob_id,tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],pair<double,double>(tmp[5],tmp[6]),tmp[7],
        pair<double,double>(tmp[8],tmp[9]));
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





void calcuStudioDis()
{
    int num = studios.size();
    int i, j;
    for (i = 0; i < num; i++)
    {
        for (j = 0; j < i; j++)
        {
            dis[j][i] = dis[i][j] = calcuDis(studios[i].pos, studios[j].pos);
        }
    }
}

void print_matr(){
    int i = 0;
    int j;
    for(i = 1 ; i <= 7; i++){
        cerr << "kkkkkkk"<<material[i].size()<<endl;
        // for(j=0;j<material[i].size();j++) 
        //     cerr<<"mater "<<i<<"studio "<<material[i][j]<<endl;
    }
    // for(i = 1 ; i <= 7; i++){
    //     cerr<<"product i"<<i<<" size = "<<product[i].size()<<endl;
    //     // for(j=0;j<product[i].size();j++) 
    //     //     cerr<<"mater "<<i<<"studio "<<product[i][j]<<endl;
    // }
}

double calAngle(pair<double, double> a, pair<double, double> b) {
    return acos(calVectorProduct(a, b) / calVectorSize(a) / calVectorSize(b));
}

double calAngle(pair<double, double> a) {

    double angle = acos(a.first / calVectorSize(a));
    return lt(a.second, 0.0) ? 2 * Pi- angle: angle;
}


PayLoad calPayload(int robortID,int studioID) {
    
    //int target = rand() % ((int)studios.size());
    //robots[robortID].target_id = target;

    //cerr << robortID << target<<endl;

    Robot robort = robots[robortID];
    Studio studio = studios[studioID];

    // cerr << robortID << "--"<< robort.target_id<<endl;

    double distance = calcuDis(robort.pos, studio.pos);
    double angular_acceleration = robort.get_type == 0? angular_acceleration_no :angular_acceleration_has;
    double acceleration = robort.get_type == 0? acceleration_no: acceleration_has;

    // 计算机器人与目标点构成的向量与x轴正方向夹角
    pair<double, double> robortToStudio = subVector(studio.pos, robort.pos);
    double angle1 = calAngle(robortToStudio);

    double angle2 = ge(robort.direction, 0.0) ? robort.direction: 2 * Pi + robort.direction;
    double angle = angle2 - angle1;

    int sign;

    if(ge(angle, 0) && lt(angle, Pi) || lt(angle, -Pi))
        sign = -1;
    else
        sign = 1;
    angle = fabs(angle);
    angle  = gt(angle, Pi)? 2 * Pi - angle: angle;


    // cerr<<"**"<< angle1<<"**dir:"<<robort.direction<<"**"<<angle2<<endl;
    // cerr<<"**"<< angle << "**"<<distance<<"**"<<sign<<endl;

    return PayLoad(angle, angular_acceleration, acceleration, distance, sign);
}

bool eq(double a, double b) { return abs(a - b) < EPS; } // ==
bool gt(double a, double b) { return a - b > EPS; }      // >
bool lt(double a, double b) { return a - b < -EPS; }     // <
bool ge(double a, double b) { return a - b > -EPS; }     // >=
bool le(double a, double b) { return a - b < EPS; }      // <=

double getRobotRadius(int robort_id) {
    return robots[robort_id].get_type == 0? 0.45: 0.53;
}


bool checkRobortsCollison(int robotA_id, int robotB_id, double k) {
    Robot robotA = robots[robotA_id];
    Robot robotB = robots[robotB_id];
    pair<double, double> next_posA = getNextPos(robotA_id);
    pair<double, double> next_posB = getNextPos(robotB_id);

    // if ((state.FrameID >= 1555 && state.FrameID <= 1557) && robotA_id == 2 && robotB_id == 3)
    // {
    //     cerr << "time" << state.FrameID << endl;
    //     cerr << "robot" << robotA_id << " ";
    //     printPair(robotA.pos);
    //     printPair(next_posA);
    //     cerr << "robot" << robotB_id << " ";
    //     printPair(robotB.pos);
    //     printPair(next_posB);
    //     cerr << "dis:" << calcuDis(robotA.pos, robotB.pos) << "**" << calcuDis(next_posA, next_posB) << endl;
    //     cerr << "ra+" << getRobotRadius(robotA_id) + getRobotRadius(robotB_id) << endl
    //          << endl;
    // }

    return le(calcuDis(next_posA, next_posB), getRobotRadius(robotA_id) + getRobotRadius(robotB_id) + k);
}

bool checkRobortsCollison(int robotA_id, pair<double, double> next_pos, int robotB_id) {
    Robot robotA = robots[robotA_id];
    Robot robortB = robots[robotB_id];
    return ge(getRobotRadius(robotA_id) + getRobotRadius(robotB_id), calcuDis(next_pos, robortB.pos));
}

bool checkIsTrySeparate(int robotA_id, int robotB_id) {
    Robot robotA = robots[robotA_id];
    Robot robortB = robots[robotB_id];
    pair<double, double> next_posA = getNextPos(robotA_id);
    pair<double, double> next_posB = getNextPos(robotB_id);
    return gt(getRobotRadius(robotA_id) + getRobotRadius(robotB_id), calcuDis(next_posA, next_posB));
}

double calNextTimeDistance(double speed, double time, double  acceleration) {
    double speed_max = min(speed + time * acceleration, 6.0);
    double time_rest = max(time - (speed_max - speed) / acceleration, 0.0);
    return (speed_max * speed_max - speed * speed) / 2 / acceleration + speed_max * time_rest;
}

bool checkTimeEnough(int robot_id, int target_id, int frame) {
    double dis = calcuDis(robots[robot_id].pos, studios[target_id].pos);
    double time = frame * 0.02;//剩余秒数
    double speed = calVectorSize(robots[robot_id].xy_pos);
    double acceleration = robots[robot_id].get_type == 0? acceleration_no: acceleration_has;
    // if((state.FrameID > 8500 )){
    //     cerr<<"FrameID "<<state.FrameID<<endl;
    //     cerr<<robot_id<<"-"<<target_id<<endl;
    //     cerr<<"dis:"<<dis<<" speed:"<<speed<<endl;
    //     cerr<<calNextTimeDistance(speed, time, acceleration)<<endl;
    // }
    if(lt(calNextTimeDistance(speed, time, acceleration), dis*1.2))
        return false;
        
    return true;
}

bool checkEnough(int robot_id, int target_id, int frame)
{
    double dis = calcuDis(robots[robot_id].pos, studios[target_id].pos) - 0.4;
    if (dis > 0)
    {
        double time = (dis / 6.0) / 0.02; // 剩余秒数
        // cerr<<"time = "<<time<<" least time = "<<frame<<endl;
        if (time > frame)
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

bool getAvoidDirection(int goID, int stopID)
{
    double angle1 = calAngle(subVector(robots[stopID].pos, robots[goID].pos));
    double angle2 = ge(robots[goID].direction, 0.0) ? robots[goID].direction : 2 * Pi + robots[goID].direction;
    double angle3 = angle2 - angle1;

    double included_angle = fabs(angle3);
    included_angle = gt(angle3, Pi) ? 2 * Pi - angle3 : angle3;

    // if (state.FrameID == 965)
    //     cerr << "angle" << included_angle << "angle3:" << angle3 << endl;

    // 如果stopID-goID方向与goTD前进方向是锐角，go旋转
    if (lt(fabs(included_angle), Pi / 2))
    {
        if (ge(angle3, 0) && lt(angle3, Pi) || lt(angle3, -Pi))
            ins[goID].rotate = Pi;
        else
            ins[goID].rotate = -Pi;
        // if(lt(included_angle, robots[goID].angular_velocity * 0.05)) {
        //     ins[goID].rotate = gt(robots[goID].angular_velocity, 0)? Pi: -Pi;
        // }
        return true;
    }
    return false;
}

bool isAcuteAngle(pair<double, double> a, pair<double, double> b)
{
    return gt(calVectorProduct(a, b), 0);
}

bool isAcuteAngle(pair<double, double> a, double x)
{
    return gt(calVectorProduct(a, make_pair(cos(x), sin(x))), 0);
}

void printPair(pair<double,double> a) {
    //cerr<<"pos:("<<a.first<<", "<<a.second<<")"<<endl;
}

void printRobotInfo(int i)
{
    Robot r = robots[i];

    // cerr << "id:" << r.id << " dir:" << r.direction
    //      << "speed:" << calVectorSize(r.xy_pos)
    //      << "/n cv:" << r.collision_val << " tv:" << r.time_val;
    printPair(r.pos);
    printPair(r.xy_pos);
}

void solveRobortsCollision()
{
    int stopID, goID;
    double speed_go, speed_stop;
    bool flag_go, flag_stop;

    for (int i = 0; i < 4; i++)
    {
        for (int j = i + 1; j < 4; j++)
        {
            if (checkIsTrySeparate(i, j))
                continue;

            stopID = robots[i] < robots[j] ? i : j;
            goID = robots[i] < robots[j] ? j : i;
            speed_go = calVectorSize(robots[goID].xy_pos);
            speed_stop = calVectorSize(robots[stopID].xy_pos);

            double angle = fabs(robots[i].direction - robots[j].direction);
            angle = gt(angle, Pi) ? 2 * Pi - angle : angle;

            if ((state.FrameID >= 1555 && state.FrameID <= 1557) && i == 2 && j == 3)
            {
                // cerr << "time" << state.FrameID << endl;
                // cerr << "angle:" << angle << endl;
            }

            // 如果两小球方向为锐角
            if (lt(angle, Pi * 0.5))
            {
                continue;
                if (!checkRobortsCollison(i, j, 1.0))
                    continue;

                flag_go = isAcuteAngle(subVector(robots[stopID].pos, robots[goID].pos), robots[goID].direction);
                flag_stop = isAcuteAngle(subVector(robots[goID].pos, robots[stopID].pos), robots[stopID].direction);
                if (flag_go && flag_stop)
                { // 并排运动
                    // 优先级低的先减速
                    ins[stopID].forward = 0;
                }
                else
                { // 追赶ing，且后面的速度大于前面，锐角在后
                    if (flag_go && gt(speed_go, speed_stop))
                    {                                    // goID小球在后面
                        getAvoidDirection(goID, stopID); // go转弯
                    }
                    else if (flag_stop && lt(speed_go, speed_stop))
                    {
                        getAvoidDirection(stopID, goID);
                    }
                }
            }
            else
            { // 夹角为钝角
                if (!checkRobortsCollison(i, j, max(10 * 0.02 * (speed_go + speed_stop), 0.5)))
                    continue;
                // 优先级小的先
                // 如果goID需要变换方向
                if (getAvoidDirection(goID, stopID))
                {
                    ins[stopID].rotate = ins[goID].rotate;
                    if (lt(speed_stop, 1.0))
                        ins[stopID].forward = 2;
                }
                else
                {
                    getAvoidDirection(stopID, goID);
                }
            }

            if (true)
            {
                // cerr << "time:" << state.FrameID << endl;
                // cerr << angle << endl;
                // cerr << "stopID:" << stopID << "-" << robots[stopID].target_id << endl;
                // // cerr<<"**"<<robots[stopID].get_type<<endl;
                // cerr << "speed" << calVectorSize(robots[stopID].xy_pos) << " dir:" << robots[stopID].direction << endl;
                // cerr << "a_speed" << robots[stopID].angular_velocity << endl;
                // cerr << "rate:" << ins[stopID].rotate << endl;
                // cerr << "**" << endl;
                // cerr << "goID:" << goID << "-" << robots[goID].target_id << endl;
                // // cerr<<"**"<<robots[goID].get_type<<endl;
                // cerr << "speed" << calVectorSize(robots[goID].xy_pos) << " dir:" << robots[goID].direction << endl;
                // cerr << "a_speed" << robots[goID].angular_velocity << endl;
                // cerr << "rate:" << ins[goID].rotate << endl
                //      << endl;
            }
        }
    }
}

void updateLastRate()
{
    for (int i = 0; i < 4; ++i)
    {
        robots[i].lastRate = ins[i].rotate;
    }
}



void control(vector<PayLoad> payLoad){
    const double time=0.04;//预测的时间。
    const double rateLim=0.24434609528;//14度
    const double Dec_val=0.003;//减速系数
    const double Dec_val_ra=1;//角速度减速系数
    const double p1=1;//机器人距离多近时开始减速
    const int max_dis=5;
    vector<int>arr{0,1,2,3};
    auto cmp=[&](int i1,int i2){
        if(robots[i1].get_type!=robots[i2].get_type)
        return robots[i1].get_type>robots[i2].get_type;
        else
        return who_isFirst(i1,i2);
    };
    auto check=[&](int rid)->bool{
        double radius=robots[rid].get_type==0? 0.45:0.53;
        double n_x=robots[rid].pos.first+robots[rid].xy_pos.first*time,n_y=robots[rid].pos.second+robots[rid].xy_pos.second*time;
        if(lt(n_x-radius,0)||lt(n_y-radius,0)||gt(n_x+radius,50)||gt(n_y+radius,50))
        return true;
        return false;
    };//判断是否有可能撞墙

    for(int i=0;i<4;i++){
        int robID=robots[i].id;
        ins[i].robID=robots[i].id;
        int isSame=robots[i].lastSign!=0?robots[i].lastSign*payLoad[i].sign:1;
        if(isSame==-1){
            robots[i].isTurn=-1;
        }
        int isTurn=robots[i].isTurn;
        robots[i].lastSign=payLoad[i].sign;
        double lastRate=fabs(robots[i].lastRate);
        double Dev_val=robots[i].angular_velocity*robots[i].angular_velocity/2*payLoad[i].angular_acceleration;
        bool can_st=can_stop(robots[i].pos,studios[robots[i].target_id].pos,payLoad[i].angle);
        // if(state.FrameID>=470&&i==3){
        //     cerr<<"---"<<endl;
        //     cerr<<state.FrameID<<endl;
        //     cerr<<check(robID)<<endl;
        //     cerr<<robots[i].target_id<<" "<<i<<endl;
        //     cerr<<robots[i].collision_val<<"-"<<i<<" "<<robots[i].pos.first<<" "<<robots[i].pos.second<<endl; 
        //     cerr<<" "<<robots[i].xy_pos.first<<" "<<robots[i].xy_pos.second<<endl;
        //     cerr<<can_st<<" angle "<<"--"<<payLoad[i].angle<<"--"<<payLoad[i].sign<<
        //     " direction "<<robots[i].direction <<endl;
        //     cerr<<return_cal(robots[i].pos,studios[robots[i].target_id].pos,fabs(payLoad[i].angle))<<endl;
            
        //     cerr<<"tar pos"<<studios[robots[i].target_id].pos.first<<" "<<studios[robots[i].target_id].pos.second<<endl;
        //     cerr<<"rob pos"<<robots[i].pos.first<<" "<<robots[i].pos.second<<endl;
        //     cerr<<"---"<<endl; 
        // }
        // if(state.FrameID>=610&&robots[i].collision_val!=0&&lt(robots[i].collision_val,1)&&i==3){
        //     // cerr<<"~~~~~"<<endl;
        //     // cerr<<state.FrameID<<endl;
        //     // cerr<<robots[i].collision_val<<"-"<<i<<" "<<robots[i].pos.first<<" "<<robots[i].pos.second<<endl;
        //     // cerr<<"~~~~~"<<endl;
            
        // }
        vector<double> tmp=get_T_limits(robots[i].pos,i);
        if(!eq(tmp[0],-7)&&(!is_range(robots[i].direction,tmp))){
            // if(i==2)
            // cerr<<"~"<<payLoad[i].angle<<" "<<robots[i].direction<<" "<<robots[i].lastRate
            // <<"~"<<robots[i].target_id<<endl;

            ins[i].rotate=((isSame==1)?Pi*payLoad[i].sign:max(0.5,Dec_val_ra*lastRate)*payLoad[i].sign);
            robots[i].lastRate=ins[i].rotate;
            ins[i].forward=0;
            continue;
        }
      
        double dis=calcuDis(robots[i].pos,studios[robots[i].target_id].pos);
        if(lt(dis,(getRobotRadius(i)+2))&&!can_st){
                ins[i].rotate=((isSame==1&&isTurn==0)?Pi*payLoad[i].sign:max(0.8,Dec_val_ra*lastRate)*payLoad[i].sign);
                // if(i==0)
                // cerr<<"~"<<ins[i].rotate<<" "<<isSame<<"+"<<payLoad[i].angle<<"+" <<Dec_val_ra*lastRate*payLoad[i].sign<<endl;
                // if(robots[i]..forward>=3)
                // ins[i].forward=-2;
                ins[i].forward=0;
                robots[i].lastRate=ins[i].rotate;   
                continue;         
        }
        if(isWall_r(i,payLoad[i].angle)){
                ins[i].rotate=((isSame==1&&isTurn==0)?Pi*payLoad[i].sign:max(0.8,Dec_val_ra*lastRate)*payLoad[i].sign);
                // if(i==0)
                // cerr<<"~"<<ins[i].rotate<<" "<<isSame<<"+"<<payLoad[i].angle<<"+" <<Dec_val_ra*lastRate*payLoad[i].sign<<endl;
                // if(robots[i]..forward>=3)
                // ins[i].forward=-2;
                ins[i].forward=1;
                robots[i].lastRate=ins[i].rotate;   
                continue;     
        }
        // int can_st_flag=1;
        double stop_dis=(robots[i].xy_pos.first*robots[i].xy_pos.first+robots[i].xy_pos.second*robots[i].xy_pos.second)
        /(2*payLoad[i].acceleration);
        
        if( isWall(robots[i].target_id)&&can_st&&ins[i].rotate==0){
            if(can_speed_z(robots[i].target_id,robots[i].xy_pos,robots[i].pos,payLoad[i].acceleration)){
                ins[i].forward=0;
                // can_st_flag=0;
            }else{
                ins[i].forward=6;
            }
        }else if(check(robID)){
            if(can_st)
            ins[i].forward=4;
            else
            ins[i].forward=1;
        }else if(will_impact(robID,stop_dis)&&can_st&&robots[i].get_type!=0){
            // cerr<<stop_dis<<"~"<<endl;
            ins[i].forward=0;
        }
        else{
            ins[i].forward=6;
        }
        if(can_st){
            // if(i==0)
            // cerr<<"----"<<endl;
            ins[i].rotate=0;
            robots[i].isTurn=0;
            robots[i].lastRate=ins[i].rotate;
        }else{
            ins[i].rotate=((isSame==1&&isTurn==0)?Pi*payLoad[i].sign:max(0.8,Dec_val_ra*lastRate)*payLoad[i].sign);
                            // if(i==0)
                // if(i==0)
                // cerr<<"+"<<ins[i].rotate<<" "<<isSame<<"+"<<payLoad[i].angle<<"+" <<Dec_val_ra*lastRate*payLoad[i].sign<<endl;
            robots[i].lastRate=ins[i].rotate;
        }
        
    }
    // solveRobortsCollision();
    Collision_detection(payLoad);
    updateLastRate();
    
    
//     vector<bool>vis(4,false);
//     for(int i=0;i<4;i++){
        
//         if(robots[arr[i]].get_type==0)break;
//         for(int j=i+1;j<4;j++){
//             int tmp=special_test(arr[i],arr[j]);
//             if(vis[j])continue;
//             if(tmp){
//                 ins[arr[j]].forward*=0.5;
//                 vis[j]=true;
                
//             }
//         }
//     }

// if(state.FrameID>=150&&state.FrameID<=450){
//     cerr<<"---";
//     cerr<<state.FrameID<<endl;
//  for(int i=0;i<4;i++)cerr<<arr[i]<<" ";
//  cerr<<endl;
//   cerr<<"---";
// }
// if(state.FrameID>=15){
//     cerr<<"-----"<<endl;;
//     cerr<<"id:"<<state.FrameID<<endl;
//     for(int i=0;i<4;i++) {
//         int id1=arr[i];
//     int Flag_line1=can_stop(robots[id1].pos,studios[robots[id1].target_id].pos,payLoad[id1].angle);
//         cerr<<arr[i]<<" "<<ins[arr[i]].forward<<" "<<ins[arr[i]].rotate
//         <<" tar "<<robots[arr[i]].target_id<<" "<<Flag_line1<<" "<<
//         payLoad[arr[i]].angle<<" "<<payLoad[arr[i]].sign<<endl;
       

//     }   
      
// }


// if(state.FrameID>=15){
//     cerr<<state.FrameID<<endl;
//     for(int i=0;i<4;i++) {
//                 double adjustAng1=fabs(return_maxAng(i));
//            bool f= can_stop(robots[i].pos,studios[robots[i].target_id].pos,payLoad[i].angle);
//         cerr<<arr[i]<<" "<<ins[arr[i]].forward<<" "<<ins[arr[i]].rotate
//         <<" tar "<<robots[arr[i]].target_id<<" ang "<<payLoad[i].angle
//         <<" "<<f<< endl;
       

//     }   
//     for(int i=0;i<4;i++){
//         for(int j=i+1;j<4;j++){
//             double tmpDis=calcuDis(robots[i].pos,robots[j].pos);
//             bool Flag_line1=lt(fabs(payLoad[i].angle),(double)Pi/tmpDis)||can_stop(robots[i].pos,studios[robots[i].target_id].pos,payLoad[i].angle);
//             bool Flag_line2=lt(fabs(payLoad[j].angle),(double)Pi/tmpDis)||can_stop(robots[j].pos,studios[robots[j].target_id].pos,payLoad[j].angle);
//             cerr<<i<<"-"<<j<<" "<<tmpDis<<" "<<will_collision(i,j)<< " "<<Calculate_root(i,j)
//             <<" can join "<<(Flag_line1&&Flag_line2)<<endl;
//         }
//     }
//     cerr<<"~~~~"<<endl;;     
// }

  
    out_put();
}
void Collision_detection(vector<PayLoad> payLoad){
    int selct1=3;
    double minDis=200;
    
    for(int i=1;i<(1<<4);i++){
        if(__builtin_popcount(i)==2){
            pair<double,bool> tmpF=return_int_dis(i);
            // cerr<<"-- "<<i<<" "<<tmpF.first<<" "<<tmpF.second<<endl;

            if(lt(tmpF.first,minDis)&&tmpF.second){
                selct1=i;
                minDis=tmpF.first;
            }
        }
    } 
    
    vector<vector<int>>arr{return_int_pos(selct1),return_int_pos(((1<<4)-1)^selct1)};
    //cerr<<arr.size()<<" "<<arr[0][0]<<"-"<<arr[0][1]<<" "<<arr[1][0]<<"-"<<arr[1][1] <<endl;
    for(int i=0;i<arr.size();i++){
        int id1=arr[i][0],id2=arr[i][1];
        double tmpDis=calcuDis(robots[id1].pos,robots[id2].pos);
        bool Flag_line1=lt(fabs(payLoad[id1].angle),0.2)||can_stop(robots[id1].pos,studios[robots[id1].target_id].pos,payLoad[id1].angle);
        bool Flag_line2=lt(fabs(payLoad[id2].angle),0.2)||can_stop(robots[id2].pos,studios[robots[id2].target_id].pos,payLoad[id2].angle);
        // cerr<<"id: "<<state.FrameID<<" "<<id1<<" "<<id2<<" "
        // <<will_collision(id1,id2)<< " "<<Flag_line1<<" "<<Flag_line2<<
        // " "<<tmpDis<<" "<<endl;
        // cerr<<" angle "<<payLoad[id1].angle<<" "<<payLoad[id2].angle<<endl;
        // cerr<<" tar "<<robots[id1].target_id<<" "<<robots[id2].target_id<<endl;
        // cerr<<"pos "<<RootFlag<<" "<<Root.first<<" "<<Root.second <<endl;
        int sel=robots[id1].get_type>robots[id2].get_type?id1:id2;
        int sel_1=robots[id1].get_type>robots[id2].get_type?id2:id1;
        if(Flag_line1&&!Flag_line2){
            sel=id1;
            sel_1=id2;
        }
        if(Flag_line2&&!Flag_line1){
            sel=id2;
            sel_1=id1;            
        }
        if(lt(tmpDis,5)&&will_collision(sel,sel_1)){
            int sign=return_line_dire(sel,sel_1,payLoad[sel_1].sign);
            int sign1=return_line_dire(sel_1,sel,payLoad[sel].sign);
            if(gt(sign*payLoad[sel_1].sign,0)){
                ins[sel_1].rotate=Pi*sign;
                // ins[sel].rotate/=2;
                cerr<<"sel: "<<sel_1<<" 0 "<<Pi*sign<< endl;
            }else if(gt(sign1*payLoad[sel].sign,0)){
                ins[sel].rotate=Pi*sign1;
                // ins[sel_1].rotate/=2;
                cerr<<"sel: "<<sel_1<<" 1 "<<Pi*sign<<endl;
            }else{
                ins[sel_1].rotate=Pi*sign;
                // ins[sel].rotate/=2;
                cerr<<"sel: "<<sel_1<<" 0 "<<Pi*sign<<endl;
            }
        }
        double v1=return_v(sel);
        double v2=return_v(sel_1);
        if(lt(min(v1,v2),3)&&is_same_direction(sel,sel_1)){
            int F=who_isFirst(sel,sel_1)?sel:sel_1;
            int S=who_isFirst(sel,sel_1)?sel_1:sel;
            ins[F].forward=6;
            continue;
            }
        
    }
    
    Detect_codirection();
}
double anger_to_length(int robot_id,int studio_id){
    double length;
    double anger = calPayload(robot_id, studio_id).angle;
    length = anger/Pi*6;
    // cerr<<"length = "<<length<<endl;
    return length;
}

double close_threshold(int robot_id,int target_id,int close_threshold){
    int count = 0;
    int dist;
    for(int i=0;i<4;i++){
        if(i!=robot_id){
            if(robots[i].target_id != -1){
                dist = calcuDis(studios[robots[i].target_id].pos,studios[i].pos);
                if(dist < close_threshold) count++;
            }
        }
    }
    return 1+count*0.5;
}

double close_threshold2(int robot_id,int target_id,int close_threshold){
    int count = 0;
    double dist,k,b,y1,y2;
    k= (studios[target_id].pos.second-robots[robot_id].pos.second)/((studios[target_id].pos.first-robots[robot_id].pos.first));
    b= robots[robot_id].pos.second-k*robots[robot_id].pos.first;
    //cerr<<" studios[target_id].pos.second-robots[robot_id].pos.second = "<<studios[target_id].pos.second-robots[robot_id].pos.second<<endl;
    for(int i=0;i<4;i++){
        if(i!=robot_id){
            if(robots[i].target_id != -1){
                if((studios[target_id].pos.second-robots[robot_id].pos.second)>0){
                    if(studios[robots[i].target_id].pos.second<studios[target_id].pos.second && robots[i].pos.second>robots[robot_id].pos.second){
                        y1=k*(studios[robots[i].target_id].pos.first-close_threshold)+b;
                        y2=k*(studios[robots[i].target_id].pos.first+close_threshold)+b;
                        if(studios[robots[i].target_id].pos.second<=y2 && studios[robots[i].target_id].pos.second>=y1){
                            cerr<<" robots[i].direction-robots[robot_id].direction- = "<<robots[i].direction-robots[robot_id].direction<<endl;
                            if(((robots[i].direction-robots[robot_id].direction)>(Pi*0.6))&&((robots[i].direction-robots[robot_id].direction)<(Pi*1.4))){
                                count++;
                            }
                        }
                    }
                }
            }
            else{ 
                if(studios[robots[i].target_id].pos.second>studios[target_id].pos.second && robots[i].pos.second<robots[robot_id].pos.second){
                    y1=k*(studios[robots[i].target_id].pos.first-close_threshold)+b;
                    y2=k*(studios[robots[i].target_id].pos.first+close_threshold)+b;
                    if(studios[robots[i].target_id].pos.second<=y2 && studios[robots[i].target_id].pos.second>=y1){
                        cerr<<" robots[i].direction-robots[robot_id].direction = "<<robots[i].direction-robots[robot_id].direction<<endl;
                        if(((robots[i].direction-robots[robot_id].direction<(-Pi*0.6))&&((robots[i].direction-robots[robot_id].direction)>(-Pi*1.4)))){
                            count++;
                        }
                    }
                }
            }
        }
    }
    cerr<<"robot "<<robot_id<<" count = "<<count<<" studio_id = "<<target_id<<endl;
    return 1+count*2;
}
double wait_dis(int robot_id ,int studio_id){
    double dis;
    double dist = calcuDis(robots[robot_id].pos,studios[studio_id].pos);
    if(studios[studio_id].pStatus==1||checkEnough(robot_id,studio_id,studios[studio_id].r_time))return 0;
    else{
        // cerr<<" studios[studio_id].r_time = "<<studios[studio_id].r_time<<" (dist/6.0/0.02) "<<(dist/6.0/0.02)<<endl;
        dis = (studios[studio_id].r_time-(dist/6.0/0.02))*6*0.02 ;
        cerr<<" wait dis = "<<dis<<endl;
    }
    return dis;  
}
/*
  control target_id
*/
pair<int,double> pick_point(int robot_id, int state){
    pair<double,double> pos = robots[robot_id].pos;
    double min = 1000;
    int min_subscript = -1;
    int i,j;
    int studio_id;
    double dist;
    int item_type = robots[robot_id].get_type;
    // if(state == 0){
    //     for(i=0;i<studios.size();i++){
    //         if(studios[i].type >= 1 && studios[i].type <= 3 && (studios[i].r_id==-1 ) ){  //123 and no robot choose ,get
    //             // if(studios[i].pStatus == 1||(studios[i].r_time>0&&(checkEnough(robot_id,i,studios[i].r_time)))){
    //             if(studios[i].pStatus == 1||(studios[i].r_time>0)){
    //                 if(material[studios[i].type].size()>0 && robot_get_type[studios[i].type]< material[studios[i].type].size()){
    //                     // dist=(calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i))*close_threshold2(robot_id,i,1.0);
    //                     // dist=(calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i));
    //                     // dist=calcuDis(robots[robot_id].pos,studios[i].pos);
    //                     dist=(calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i))+wait_dis(robot_id,i);
    //                     if(dist<min){
    //                         min=dist;
    //                         min_subscript=i;
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }
    // else if(state == 1){
    //     for(i=0;i<full_product.size();i++){
    //         //cerr<<studios.size()<<endl;
    //         // cerr<<" full_product[i] = "<<full_product[i]<<endl;
    //         if(studios[full_product[i]].r_id==-1){  //123 and no robot choose ,first choose ,get
    //             // if(studios[full_product[i]].pStatus == 1 ||(studios[full_product[i]].r_time>0&&(checkEnough(robot_id,full_product[i],studios[i].r_time)))){
    //             if(studios[full_product[i]].pStatus == 1||(studios[full_product[i]].r_time>0)){
    //                 // dist=(calcuDis(robots[robot_id].pos,studios[full_product[i]].pos)+anger_to_length(robot_id,full_product[i]))*close_threshold2(robot_id,full_product[i],1.0);
    //                 // dist=(calcuDis(robots[robot_id].pos,studios[full_product[i]].pos)+anger_to_length(robot_id,full_product[i]));
    //                 // dist=calcuDis(robots[robot_id].pos,studios[full_product[i]].pos);
    //                 dist=(calcuDis(robots[robot_id].pos,studios[full_product[i]].pos)+anger_to_length(robot_id,full_product[i]))+wait_dis(robot_id,full_product[i]);
    //                 if(dist<min){
    //                     min=dist;
    //                     min_subscript=full_product[i];
    //                 }
    //             }
    //         }
    //     }
    // }
    // else 
    
    if(state == 2){
        for(i=0;i<studios.size();i++){
            if(studios[i].type >= 1 && studios[i].type <= 3 && (studios[i].r_id==-1 ) ){  //123 and no robot choose ,get
                // if(studios[i].pStatus == 1||(studios[i].r_time>0&&(checkEnough(robot_id,i,studios[i].r_time)))){
                if(studios[i].pStatus == 1||(studios[i].r_time>0)){
                    if(robot_get_type[studios[i].type]< material[studios[i].type].size()){
                    // if(material[studios[i].type].size()>0){
                        // dist=(calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i))*close_threshold2(robot_id,i,1.0);
                        // dist=(calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i));
                        dist=(calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i))+wait_dis(robot_id,i);
                        // cerr<<calcuDis(robots[robot_id].pos,studios[i].pos)<<' '<<anger_to_length(robot_id,i)<<' '<<wait_dis(robot_id,i)<<endl;
                        
                        // dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                        if(dist<min){
                            min=dist;
                            min_subscript=i;
                        }
                        // cerr<<state<< ' '<<dist<< ' '<<min<<' '<<min_subscript<<' '<<i<<endl;
                    }
                }
            }
        }
    }
    else if(state == 3){
        for(i=0;i<studios.size();i++){
            if(studios[i].type >= 4 && studios[i].type <= 6 && (studios[i].r_id==-1 )){  //456 and no robot choose ,get
                // if(studios[i].pStatus == 1 ||(studios[i].r_time>0&&(checkEnough(robot_id,i,studios[i].r_time)))){
                if(studios[i].pStatus == 1||(studios[i].r_time>0)){
                    //||(studios[i].r_time>0&&(checkEnough(robot_id,i,studios[i].r_time)))
                    if(robot_get_type[studios[i].type]< material[studios[i].type].size()){
                    // if(material[studios[i].type].size()>0){
                        // dist=(calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i))*close_threshold2(robot_id,i,1.0);
                        // dist=calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i);
                        dist=(calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i))+wait_dis(robot_id,i);
                        // cerr<<"close_threshold = "<<close_threshold2(robot_id,i,1)<<endl;
                        // dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                        if(dist<min){
                            min=dist;
                            min_subscript=i;
                        }
                        // cerr<<state<< ' '<<dist<< ' '<<min<<' '<<min_subscript<<endl;
                    }
                }
            }
        }
    }
    else if(state == 4){
        for(i=0;i<studios.size();i++){
            if(studios[i].type ==7 && (studios[i].r_id==-1)){  //7 and no robot choose ,get
                // if(studios[i].pStatus == 1 ||(studios[i].r_time>0&&(checkEnough(robot_id,i,studios[i].r_time)))){
                if(studios[i].pStatus == 1||(studios[i].r_time>0)){
                    if(robot_get_type[studios[i].type]< material[studios[i].type].size()){
                    // if(material[studios[i].type].size()>0){
                        // dist=(calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i))*close_threshold2(robot_id,i,1.0);
                        // cerr<<"close_threshold = "<<close_threshold2(robot_id,i,1)<<endl;
                        // dist=calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i);
                        dist=(calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i))+wait_dis(robot_id,i);
                        // dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                        if(dist < min){
                            min=dist;
                            min_subscript=i;
                        }
                        // cerr<<state<< ' '<<dist<< ' '<<min<<' '<<min_subscript<<endl;
                    }
                }
            }
        }
    }
    else if(state == 5){             
            //send
        for(i=0;i<studios.size();i++){
            for(j=1;j<=material_send[item_type][0];j++){
                if(item_type != 7){
                    if(studios[i].type == material_send[item_type][j] && (studios_rid[i][item_type] == -1) ){
                        if(((studios[i].bitSatus & ((int)pow(2,item_type)))==0)||((check_material_full(i)&&(studios[i].pStatus != 1)&&(studios[i].r_time>0&&(checkEnough(robot_id,i,studios[i].r_time)))))){
                            // dist=(calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i))*close_threshold(robot_id,i,1.0);
                            dist=calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i);
                            // dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                            if(dist<min){
                                min=dist;
                                min_subscript=i;
                            }
                        }
                    }
                }
                else{
                    if(studios[i].type == material_send[item_type][j]){
                        // dist=(calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i))*close_threshold(robot_id,i,1.0);
                        dist=calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i);
                        // dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                        if(dist<min){
                            min=dist;
                            min_subscript=i;
                        }
                    }
                }
            }
            if(studios[i].type == 9){
                // dist=(calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i))*close_threshold(robot_id,i,1.0);
                dist=calcuDis(robots[robot_id].pos,studios[i].pos)+anger_to_length(robot_id,i);
                // dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                if(dist<min){
                    min=dist;
                    min_subscript=i;
                }
            }
        }
    }
    return pair<int,double>(min_subscript,min);
}
pair<int,double> choose_lack(int studio_id ,int threshold){
    int dist ;
    int min =100;
    int min_subscript = -1;
    if(studios[studio_id].type >3 &&studios[studio_id].type < 8){
        for(int i = 0;i < 4;i++){
            if(studios[studio_id].type == i+4){
                for(int j = 0;j<studio_material[i][0];j++){
                    if((studios[studio_id].bitSatus & (int)pow(2,studio_material[i][j+1])) == 0){
                        if(studios_rid[studio_id][studio_material[i][j+1]] == -1){
                            //cerr<<" product[studio_material[i][j+1]].size() "<<product[studio_material[i][j+1]].size()<<endl;
                            for(int k = 0;k<product[studio_material[i][j+1]].size();k++){
                                //cerr<<" studios[studio_id].pos = "<<studios[studio_id].pos.first<<' '<<studios[studio_id].pos.second<<endl;
                                //cerr<<" studios[product[studio_material[i][j+1]][k]].pos"<<studios[product[studio_material[i][j+1]][k]].pos.first<<' '<<studios[product[studio_material[i][j+1]][k]].pos.second<<endl;
                                dist=calcuDis(studios[studio_id].pos,studios[product[studio_material[i][j+1]][k]].pos);
                                if(dist<min){
                                    min=dist;
                                    min_subscript=product[studio_material[i][j+1]][k];
                                }
                            }
                        }
                        //cerr<<" dist = "<<dist<<" threshold = "<<threshold<<endl;
                    }
                }
            }
        }
    }
    if(min < threshold)return pair<int,double>(min_subscript,min);
    return pair<int,double>(-1,-1);
}
void first_action(){
    int i,j; 
    pair <int,double> p;
    pair <int,double> f;
    for(i = 0; i < robots.size(); i++){
        robots[i].target_id = pick_point(i, 2).first;
        cerr<<" first target "<<robots[i].target_id<<endl;
        //cerr<<"aaa"<<robots[i].target_id<<endl;
        //studios[robots[i].target_id].r_id = i;
    }
    // if(robots[i].get_type==0)cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<studios[robots[i].target_id].type<<" buy "<<ins[i].buy<<" sell "<<ins[i].sell<<endl;
    // else cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" buy "<<ins[i].buy<<" sell "<<ins[i].sell<<endl;
    for(i = 0; i < robots.size(); i++)
    {
        studios[robots[i].target_id].r_id = i;
        for(j = i + 1; j < robots.size(); j++){
            if(robots[i].target_id == robots[j].target_id){
                p = pick_point(i,2);
                f = pick_point(j,2);
                cerr<<p.first<<' '<<f.first<<' '<<p.second<<' '<<f.second<<endl;
                if(gt(p.second,f.second)){
                    studios[robots[i].target_id].r_id = i;
                    robots[j].target_id = f.first;
                    studios[robots[j].target_id].r_id = j;
                }else {
                    studios[robots[j].target_id].r_id = j;
                    robots[i].target_id = p.first;
                    cerr<<" robots[i].target_id = "<<robots[i].target_id<<endl;
                    studios[robots[i].target_id].r_id = i;
                }
            }
        }
        //studios[robots[i].target_id].r_id = i;
        cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
    }
}

bool judge_full(int level, double threshold){
    int i,j;
    int count = 0;
    int full_count = 0;
    double v;
    full_product.clear();
    if(level == 2){
        for(i = 0;i < studios.size();i++){
            if(studios[i].type >= 4 && studios[i].type <= 7){
                count++;
                if(studios[i].pStatus == 1 && studios[i].r_id == -1 ){
                    for(j = 0;j<studio_material[studios[i].type-4][0];j++){
                        if((studios[i].bitSatus & (int)pow(2,studio_material[studios[i].type-4][j+1])) == 0) break;
                    }
                    if(j==studio_material[studios[i].type-4][0]){
                        full_count++;
                        full_product.push_back(i);
                        // cerr<<" stuidio_id = "<<i<<" studios[i].type = "<<studios[i].type<<" studios[i].pStatus = "<<studios[i].pStatus<<" studios[i].bitSatus = "<<studios[i].bitSatus<<endl;
                    }
                }
            }
        }
        v = (double)full_count/(double)count;
        cerr<<" full_count = "<<full_count<<" count = "<<count<<" bilu "<<v<<endl;
        if(v >= threshold){
            // print_matr();
            return true;
        }
    }
    if(level == 3){
        for(i = 0;i < studios.size();i++){
            if(studios[i].type == 7){
                count++;
                if(studios[i].pStatus == 1 && studios[i].r_id == -1){
                    for(j = 0;j<studio_material[studios[i].type-4][0];j++){
                        if((studios[i].bitSatus & (int)pow(2,studio_material[studios[i].type-4][j+1])) == 0) break;
                    }
                    if(j==studio_material[studios[i].type-4][0]){
                        // cerr<<" stuidio_id = "<<i<<" studios[i].type = "<<studios[i].type<<" studios[i].pStatus = "<<studios[i].pStatus<<" studios[i].bitSatus = "<<studios[i].bitSatus<<endl;
                        full_product.push_back(i);
                        full_count++;
                    }
                }
            }
        }
        v = (double)full_count/(double)count;
        cerr<<" full_count = "<<full_count<<" count = "<<count<<" bilu "<<v<<endl;
        if(v >= threshold){
            // print_matr();
            return true;
        }
    }
    return false;
}
bool check_material_full(int studio_id){
    int j;
    for(j = 0;j<studio_material[studios[studio_id].type-4][0];j++){
        if((studios[studio_id].bitSatus & (int)pow(2,studio_material[studios[studio_id].type-4][j+1])) == 0) break;
    }
    if(j==studio_material[studios[studio_id].type-4][0]){
        return true;
    }
    return false;
}
void robot_judge(int full,int threshold_near,int threshold_lack){
    int i;
    int target;
    pair<int,double> temp1;
    pair<int,double> temp2;
    //cerr<<robots.size()<<endl;
    for(i = 0; i < robots.size(); i++){
        //cerr<<i<<robots[i].target_id<<endl;
        if(robots[i].loc_id == robots[i].target_id && robots[i].target_id != -1){
            if(robots[i].get_type == 0){
                //dosomething buy ,next send
                robots[i].lastSign=0;
                robots[i].isTurn=0;
                robots[i].get_type = studios[robots[i].loc_id].type;
                //cerr<<"robots "<< i<<" buy "<<studios[robots[i].target_id].type<<endl;
                studios[robots[i].loc_id].r_id = -1;
                robots[i].target_id = pick_point(i,5).first;
                if(robots[i].target_id!= -1){
                    //studios[robots[i].target_id].r_id = i;
                    if(studios[robots[i].target_id].type!=8&&studios[robots[i].target_id].type!=9)studios_rid[robots[i].target_id][robots[i].get_type] = i;
                    //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<" flag "<<studios_rid[robots[i].target_id][robots[i].get_type]<<endl;
                }
                ins[i].buy = 1;
                ins[i].sell = -1;
                if(state.FrameID>8500){
                    // cerr <<"***";
                    if(!checkTimeEnough(i,robots[i].target_id,9000-state.FrameID))ins[i].buy = -1;
                }
            }
            else{
                //dosomething sell
                ins[i].sell = 1;
                ins[i].buy = -1;
                robots[i].lastSign=0;
                robots[i].isTurn=0;
                ////cerr<<"robots "<< i<<" sell "<<robots[i].get_type<<endl;
                studios[robots[i].loc_id].bitSatus += (int)pow(2,robots[i].get_type);
                //studios[robots[i].loc_id].r_id = -1;
                studios_rid[robots[i].loc_id][robots[i].get_type] = -1;
                robots[i].get_type = 0;
                target = choose_lack(robots[i].loc_id,threshold_lack).first;
                //cerr<<"robots[i].loc_id "<<robots[i].loc_id<<"target = "<<target<<endl;
                if(target != -1){
                    robots[i].target_id = target ;
                     studios[robots[i].target_id].r_id = i;
                }
                else{
                if(full == 1){
                    robots[i].target_id = pick_point(i,3).first; //find near 456
                    if(robots[i].target_id!= -1){
                        //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
                        studios[robots[i].target_id].r_id = i;
                    }
                }
                else if(full == 2){
                    temp1=pick_point(i,3);
                    temp2=pick_point(i,4);
                    if((temp1.second*threshold_near) <= temp2.second)robots[i].target_id = temp1.first;
                    else robots[i].target_id = temp2.first;
                    //robots[i].target_id = pick_point(i,4).first; //find near 7
                    if(robots[i].target_id!= -1){
                        //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
                        studios[robots[i].target_id].r_id = i;
                    }
                }
                else{
                    robots[i].target_id = pick_point(i,2).first; //find near 123
                    if(robots[i].target_id!= -1){
                        //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
                        studios[robots[i].target_id].r_id = i;
                        }
                    }
                }
            }
        }
        else{
            ins[i].buy = -1;
            ins[i].sell = -1;
            ins[i].destroy = -1;
        }
        if(robots[i].target_id == -1){
            if(robots[i].get_type ==0){
                robots[i].target_id = pick_point(i,2).first; //no target
                if(robots[i].target_id!= -1){
                        //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
                    studios[robots[i].target_id].r_id = i;
                    //cerr<< "kkkkk"<<endl;
                }
            }
            else{
                robots[i].target_id = pick_point(i,5).first;
                if(robots[i].target_id!= -1){
                    //studios[robots[i].target_id].r_id = i;
                    if(studios[robots[i].target_id].type!=8&&studios[robots[i].target_id].type!=9)studios_rid[robots[i].target_id][robots[i].get_type] = i;
                    //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
                }
            }
        }
        // if(robots[i].get_type==0)cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<studios[robots[i].target_id].type<<" buy "<<ins[i].buy<<" sell "<<ins[i].sell<<endl;
        // else cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" buy "<<ins[i].buy<<" sell "<<ins[i].sell<<endl;
        // if(i==2 && robots[i].target_id != -1){
        //     cerr<<" robot 2 dis "<<calcuDis(robots[i].pos,studios[robots[i].target_id].pos)<<endl;
        // }
    }
}
void robot_judge_sol(int threshold_lack,int full){
    int i;
    int target,min_subscript;
    int min_dist,dist;
    pair<int,double> temp1;
    pair<int,double> temp2;
    //cerr<<robots.size()<<endl;
    for(i = 0; i < robots.size(); i++){
        //cerr<<i<<robots[i].target_id<<endl;
        if(robots[i].loc_id == robots[i].target_id && robots[i].target_id != -1){
            if(robots[i].get_type == 0){
                if(studios[robots[i].target_id].pStatus == 1){
                //dosomething buy ,next send
                    robots[i].lastSign=0;
                    robots[i].isTurn=0;
                    robots[i].get_type = studios[robots[i].loc_id].type;
                    //cerr<<"robots "<< i<<" buy "<<studios[robots[i].target_id].type<<endl;
                    studios[robots[i].loc_id].r_id = -1;
                    //cerr<<"dddd"<<endl;
                    robots[i].target_id = pick_point(i,5).first;
                    //cerr<<"kkkk"<<endl;
                    if(robots[i].target_id!= -1){
                        //studios[robots[i].target_id].r_id = i;
                        if(studios[robots[i].target_id].type!=8&&studios[robots[i].target_id].type!=9)studios_rid[robots[i].target_id][robots[i].get_type] = i;
                        //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<" flag "<<studios_rid[robots[i].target_id][robots[i].get_type]<<endl;
                    }
                    ins[i].buy = 1;
                    ins[i].sell = -1;
                    if(state.FrameID>8000){
                        // cerr <<"***";
                        if(!checkTimeEnough(i,robots[i].target_id,9000-state.FrameID))ins[i].buy = -1;
                    }
                }
                else{
                    ins[i].buy = -1;
                    ins[i].sell = -1;
                    cerr<<" robot "<<i<<" can not buy "<<studios[robots[i].target_id].type<<endl;
                }
            }
            else{
                //dosomething sell
                ins[i].sell = 1;
                ins[i].buy = -1;
                robots[i].lastSign=0;
                robots[i].isTurn=0;
                //cerr<<"robots "<< i<<" sell "<<robots[i].get_type<<endl;
                studios[robots[i].loc_id].bitSatus += (int)pow(2,robots[i].get_type);
                //studios[robots[i].loc_id].r_id = -1;
                studios_rid[robots[i].loc_id][robots[i].get_type] = -1;
                robots[i].get_type = 0;
                // target = -1;
                target = choose_lack(robots[i].loc_id,threshold_lack).first;
                //cerr<<"robots[i].loc_id "<<robots[i].loc_id<<"target = "<<target<<endl;
                if(target != -1){
                    robots[i].target_id = target ;
                     studios[robots[i].target_id].r_id = i;
                }
                else{
                    min_subscript = -1;
                    min_dist = 100;
                    // cerr<<"dddd "<<full<<endl;
                    if(full !=0){
                        //cerr<<"fdfdf "<<endl;
                        temp1=pick_point(i,1);
                        //cerr<<"temp1 = "<<temp1.first<<' '<<temp1.second<<endl;
                        for(int j=2;j<=4;j++){
                            temp2=pick_point(i,j);
                            dist = temp2.second;
                            if(min_dist>dist){
                                min_dist=dist;
                                min_subscript=temp2.first;
                            }
                        }
                        //cerr<<"min = "<<min_subscript<<' '<<min_dist<<endl;
                        if(temp1.second<min_dist*1.5){
                            min_dist=temp1.second;
                            min_subscript=temp1.first;
                        }
                    }
                    else{
                        for(int j=2;j<=4;j++){
                            temp1=pick_point(i,j);
                            dist = temp1.second;
                            if(min_dist>dist){
                                min_dist=dist;
                                min_subscript=temp1.first;
                            }
                        }
                    }
                    robots[i].target_id = min_subscript;
                    if(min_subscript != -1){
                        studios[robots[i].target_id].r_id = i;
                    }
                }
            }
        }
        else{
            ins[i].buy = -1;
            ins[i].sell = -1;
            ins[i].destroy = -1;
        }
        if(robots[i].target_id == -1){
            if(robots[i].get_type ==0){
                robots[i].target_id = pick_point(i,2).first; //no target
                if(robots[i].target_id!= -1){
                        //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
                    studios[robots[i].target_id].r_id = i;
                    //cerr<< "kkkkk"<<endl;
                }
            }
            else{
                robots[i].target_id = pick_point(i,5).first;
                if(robots[i].target_id!= -1){
                    //studios[robots[i].target_id].r_id = i;
                    if(studios[robots[i].target_id].type!=8&&studios[robots[i].target_id].type!=9)studios_rid[robots[i].target_id][robots[i].get_type] = i;
                    //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
                }
            }
        }
        if(robots[i].get_type==0)cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<studios[robots[i].target_id].type<<"studios r_id = "<<studios[robots[i].target_id].r_id<<" buy "<<ins[i].buy<<" sell "<<ins[i].sell<<endl;
        else cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" buy "<<ins[i].buy<<" sell "<<ins[i].sell<<endl;

        // if(robots[i].target_id != -1 && robots[i].get_type !=0){
        //     if((studios[robots[i].target_id].pStatus ==1 ||(studios[robots[i].target_id].r_time>0&&(checkEnough(i,robots[i].target_id,studios[i].r_time))))&& studios[robots[i].target_id].r_id == -1){
        //         studios[robots[i].target_id].r_id =i;
        //     }
        // }
    }

}
void robot_action(){
    //cerr<<"start"<<endl;
    //print_matr();
    // print_matr();
    for(int i =0;i<=7;i++)robot_get_type[i]=0;
    for(int i = 0;i<4;i++){
        if(robots[i].get_type != 0)robot_get_type[robots[i].get_type]++;
        else if(robots[i].target_id != -1)robot_get_type[studios[robots[i].target_id].type]++;
    }
    for(int i =0;i<=7;i++)cerr<<"type "<<i<<" has "<<robot_get_type[i];
    // cerr <<endl;
    int full = 0;
    // if(judge_full(2,0.5))full = 1;   //4,5,6 full threshold
    // // if(judge_full(3,0.2))full = 2;   //7 full threshold Higher priority
    // cerr<<" full = "<<full<<endl;
    // if(full!=0);
    // robot_judge(full,1.3,4.5);
    robot_judge_sol(5,full);
}

vector<double>  get_T_limits(pair<double,double>pos,int id,int ctr,double dis){
    double radius=robots[id].get_type==0? 0.45:0.53;
    double tmpA=robots[id].direction;
    double x_t=robots[id].pos.first+dis*cos(tmpA),y_t=robots[id].pos.second+dis*sin(tmpA);
    if(x_t<0||x_t>50||y_t<0||y_t>50){
        dis=50;
    }else{
        dis=0.0;
    }
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
bool can_stop(pair<double,double>p1,pair<double,double>p2,double angle){
    if(gt(angle,Pi/2))return false;
    double dis=calcuDis(p1,p2);
    if(lt(sin(angle)*dis,0.4)){
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
    if(gt(dis3,dis5-dis4))return true;
    return false;
}
bool isWall(int stuID){
    int i=studios[stuID].pos.first;
    int j=studios[stuID].pos.second;
    if(i-1<=0||j-2<=0||i+2>=50||j+2>=50)return true;
    return false;
}
bool isWall_r(int robID,double angle){
    if(lt(angle,(double)Pi/6.0))return false;
    int i=robots[robID].pos.first;
    int j=robots[robID].pos.second;
    if(i-1<=0||j-2<=0||i+2>=50||j+2>=50)return true;
    return false;   
}
bool will_impact(int robID,double dis){
    vector<double> tmp=get_T_limits(robots[robID].pos,robID,1,dis);
    // if(state.FrameID>=610 &&robID==3){
    // //     cerr<<state.FrameID<<endl;
    // //     cerr<<"__"<<is_range(robots[robID].direction,tmp)<<" "<<(fabs(robots[robID].xy_pos.first)>1||fabs(robots[robID].xy_pos.second)>1)<<endl;
    // // cerr<<tmp[0]<<" "<<tmp[1]<<endl;
    // // cerr<<robots[robID].direction<<endl;
    
    // }
    if(!eq(tmp[0],-7)&&(!is_range(robots[robID].direction,tmp)))
    {//在墙附件，并且会撞上
        return true;
    }
    return false;
}
int special_test(int i1,int i2){
    int cnt=5;
    int base=0.02;
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
        int base=0.02;
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
int Calculate_root(int i1,int i2){
    Vec v1(robots[i1].xy_pos);
    Vec v2(robots[i2].xy_pos);
    Vec x1(robots[i1].pos);
    Vec x2(robots[i2].pos);
    Vec x_t=x1-x2;
    Vec v_t=v1-v2;
    double r=1.4;
    double a=v_t*v_t;
    double b=2*v_t*x_t;
    double c=x_t*x_t-r*r;
    double cla=b*b-4*a*c;
    if(eq(a,0))return -1;
    else if(gt(cla,0)) return 1;
    else return 0;
}
bool will_collision(int i1,int i2){
    Vec v1(robots[i1].xy_pos);
    Vec v2(robots[i2].xy_pos);
    Vec x1(robots[i1].pos);
    Vec x2(robots[i2].pos);
    Vec c_t=x1-x2;
    Vec v_t=v1-v2;
    bool f1=isWall(robots[i1].target_id);
    bool f2=isWall(robots[i2].target_id);
    double tmpDis=calcuDis(robots[i1].pos,robots[i2].pos);
    if(gt(tmpDis,3)){
            Compute_redundancy=1.0;
    }else{
            Compute_redundancy=0.0;
        }
    double r=getRobotRadius(i1)+getRobotRadius(i2)+Compute_redundancy;
    double a=v_t*v_t;
    double b=2*(v_t*c_t);
    double c=c_t*c_t-r*r;
    double cla=b*b-4*a*c;   
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
bool return_collision(int i1,int i2){
    return lt(robots[i1].collision_val_pre,robots[i1].collision_val)&&
    lt(robots[i2].collision_val_pre,robots[i2].collision_val);
}
pair<int,int> far_away(int i1,int i2,int base1,int base2){
    int sign1=pl_g[i1].sign,sign2=pl_g[i2].sign;
    if(sign1*sign2<0) return pair<int,int> (sign1,sign2);
    else{
        if(gt(fabs(pl_g[i1].angle)-fabs(pl_g[i2].angle) ,2)||robots[i1].get_type>(robots[i2].get_type)){
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
    will_collision(i1,i2);
    double canAngle=min(fabs(Root.first),fabs(Root.second))*40*0.3;
    double need_angle_1=min(fabs(Root.first),fabs(Root.second))*fabs(robots[i1].angular_velocity);
    double need_angle=get_rotation(i1,i2);
    auto tmp= subVector(robots[i1].pos, robots[i2].pos);
    int sa=addSign(i1,i2,lt(robots[i1].angular_velocity,0.0)?-1:1);
    Vec v1(tmp);
    Vec v2(robots[i2].xy_pos);
    double tmpAngle=acos(cos_t(v1,v2));
    auto tmp1= subVector(robots[i2].pos, robots[i1].pos);
    Vec v3(tmp1);
    Vec v4(robots[i1].xy_pos);
    double tmpAngle_1=acos(cos_t(v3,v4));
    cerr<<"id"<<i1<<" "<<i2<<endl;
    cerr<<tmpAngle<<"-"<<need_angle<<endl;
    int sign= (lt(v1^v2,0))?-1:1;
    if(sign*signBase==-1&&gt(canAngle,Pi-tmpAngle_1+tmpAngle+need_angle_1*sa)){
        sign*=-1;
    }
    return sign;
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
vector<int> return_int_pos(int base){
    vector<int>arr(2);
    int pos=0;
    for(int i=0;i<4;i++){
        if((base>>i)&1){
            arr[pos++]=i;
        }
    }
    return arr;
}
void Detect_codirection(){
    for(int i=1;i<(1<<4);i++){
        if(__builtin_popcount(i)==2){
            return_int_neg(i);
        }
    } 
}
int return_int_neg(int base){
    vector<int>arr(2);
    int pos=0;
    for(int i=0;i<4;i++){
        if((base>>i)&1){
            arr[pos++]=i;
        }
    }
    int tmp=Calculate_root(arr[0],arr[1]);
    if(Adjust_the_same_direction[arr[0]][0]>0){
        Adjust_the_same_direction[arr[0]][0]--;
        ins[arr[0]].rotate=Adjust_the_same_direction[arr[0]][1];
    }
    if(Adjust_the_same_direction[arr[1]][0]>0){
        Adjust_the_same_direction[arr[1]][0]--;
        ins[arr[1]].rotate=Adjust_the_same_direction[arr[1]][1];
    }
    int id1=arr[0],id2=arr[1];
    double dis=calcuDis(robots[id1].pos,robots[id2].pos);
    if(tmp==-1&&lt(dis,4)){
        
        int sel=robots[id1].get_type>robots[id2].get_type?id1:id2;
        int sel_1=robots[id1].get_type>robots[id2].get_type?id2:id1;
        int sign1=return_line_dire(sel,sel_1);
        ins[sel_1].rotate=Pi*sign1;
        Adjust_the_same_direction[arr[1]][1]=Pi*sign1;
        Adjust_the_same_direction[arr[1]][0]=5;
    }
    return  1;
}
bool is_same_direction(int i1,int i2){
    Vec v1(pair<double,double>(cos(robots[i1].direction),sin(robots[i1].direction)));
    Vec v2(pair<double,double>(cos(robots[i2].direction),sin(robots[i2].direction)));
    return gt(cos_t(v1,v2),0.0);
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