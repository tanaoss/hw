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
double EPS=1e-10;
double acceleration_no;
double acceleration_has;
double angular_acceleration_no;
double angular_acceleration_has;
vector<bool> need_stop(4,false);
int robot_get_type[8];
int studios_rid[50][8];
int studio_material[4][4];
int keep_state[4] = {0};
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
    for(int i=0;i<7;++i) {
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
        if(studios[studio_id].pStatus == 1){
            product[studios[studio_id].type].push_back(studio_id);
        }
        if(studios[studio_id].type > 3){
            if(studios[studio_id].type < 8){
                for(int i = 0;i < 4;i++){
                    if(studios[studio_id].type == i+4){
                        for(int j = 0;j<studio_material[i][0];j++){
                            if((studios[studio_id].bitSatus & (int)pow(2,studio_material[i][j+1])) == 0){
                                if(studios_rid[studio_id][studio_material[i][j+1]] == -1)material[studio_material[i][j+1]].push_back(studio_id);
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
    // for(i = 1 ; i <= 7; i++){
    //     cerr << "kkkkkkk"<<material[i].size()<<endl;
    //     for(j=0;j<material[i].size();j++) 
    //         cerr<<"mater "<<i<<"studio "<<material[i][j]<<endl;
    // }
}

double calAngle(pair<double, double> a, pair<double, double> b) {
    return acos(calVectorProduct(a, b) / calVectorSize(a) / calVectorSize(b));
}

double calAngle(pair<double, double> a) {

    double angle = acos(a.first / calVectorSize(a));
    return lt(a.second, 0.0) ? 2 * Pi- angle: angle;
}


PayLoad calPayload(int robortID) {
    
    //int target = rand() % ((int)studios.size());
    //robots[robortID].target_id = target;

    //cerr << robortID << target<<endl;

    Robot robort = robots[robortID];
    Studio studio = studios[robort.target_id];

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


bool checkRobortsCollison(int robotA_id, int robotB_id) {
    Robot robotA = robots[robotA_id];
    Robot robortB = robots[robotB_id];
    // if((state.FrameID == 1360 || state.FrameID == 1359)&& robotA_id == 0 && robotB_id == 3) {
    //     cerr<<"time:"<<state.FrameID<<endl;
    //     cerr<< getRobotRadius(robotA_id) + getRobotRadius(robotB_id) <<endl;
    //     cerr<<"dis:"<<calcuDis(robotA.pos, robortB.pos)<<endl;
    //     cerr <<robotA.collision_val<<"-"<<robortB.collision_val<<endl;
    // }
    //todo
    return le(calcuDis(robotA.pos, robortB.pos), getRobotRadius(robotA_id) + getRobotRadius(robotB_id) + 0.5);
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
    double speed_max = max(speed + time * acceleration, 36.0);
    double time_rest = time - (speed_max - speed) / acceleration;
    return (speed_max * speed_max - speed * speed) / 2 / acceleration + speed_max * time_rest;
}

bool checkeTimeEnough(int robot_id, int target_id, int frame) {
    double dis = calcuDis(robots[robot_id].pos, studios[target_id].pos);
    double time = (9000.0 - frame) * 0.02;//剩余秒数
    double speed = calVectorSize(robots[robot_id].xy_pos);
    double acceleration = robots[robot_id].get_type == 0? acceleration_no: acceleration_has;
    
    if(lt(calNextTimeDistance(speed, time, acceleration), dis*1.3))
        return false;
        
    return true;
}


pair<double, double> transformVector(double direction) {
    direction = gt(direction, 0)? direction: Pi - direction;
    return make_pair(cos(direction), sin(direction));
}

pair<double, double> getNextSpeed(double direction, pair<double, double> speed) {
    direction = gt(direction, 0)? direction: Pi - direction;
    double val = calVectorSize(speed);
    return make_pair(cos(direction) * val, sin(direction) * val);
}

pair<double, double> getNextPos(int robot_id) {
    return addVector(robots[robot_id].pos, calVectorProduct(robots[robot_id].xy_pos, 0.02));
}

pair<double, double> getNextPos(pair<double, double> pos, pair<double, double> speed) {
    return addVector(pos, calVectorProduct(speed, 0.02));
}

pair<double, double> getNextTimePos(int time, pair<double, double> pos, pair<double, double> speed, double direction, double angular_velocity) {
    while(time--) {
        pos = getNextPos(pos, speed);
        speed = getNextSpeed(direction, speed);
        direction = direction + angular_velocity * 0.02;
    }
    return pos;
}


void predictCollision(int a, int b) {
    Robot robotA = robots[a];
    Robot robotB = robots[b];

    if(robotA.target_id == 0 && robotB.target_id == 0)
        return;
    
    // LLine lineA = LLine(robotA.pos, studios[robotA.target_id].pos);
    // LLine lineB = LLine(robotB.pos, studios[robotB.target_id].pos);
    // pair<int,Point> corss = lineA&lineB;
    // int stopID = robots[a] < robots[b] ? a: b;
    // int goID = robots[a] < robots[b] ? a: b;
    // if(corss.first == 0 | corss.first == 1) {
    //     if(checkRobortsCollison(a, b)){
            
//     }
// }
    // else {
    //     // if(robotA.target_id != robotB.target_id)
    // }
}

void getAvoidDirection(int goID, int stopID) {
    double angle1 = calAngle(subVector(robots[stopID].pos , robots[goID].pos));
    double angle2 = ge(robots[goID].direction, 0.0) ? robots[goID].direction: 2 * Pi + robots[goID].direction;
    double angle3 = angle2 - angle1;

    double included_angle = fabs(angle3);
    included_angle = gt(angle3, Pi)? 2 * Pi - angle3: angle3;

    //如果stopID-goID方向与goTD前进方向是锐角，go旋转
    if(lt(fabs(included_angle), Pi / 2)){
        if(ge(angle3, 0) && lt(angle3, Pi) || lt(angle3, -Pi))
            ins[goID].rotate = Pi;
        else
            ins[goID].rotate = -Pi;
    }
}


void solveRobortsCollision() {
    int stopID, goID;
    pair<double, double> next_pos;
    bool flag;
    for(int i = 0; i < 4; i++) {
        flag = true;
        for(int j = i + 1; j < 4; j++) {
            if(checkRobortsCollison(i, j)) {

                if(robots[i].get_type == 0 & robots[j].get_type == 0)
                    continue;

                //优先级小的先
                stopID = robots[i] < robots[j] ? i: j;
                goID = robots[i] < robots[j] ? j: i;



                if(checkIsTrySeparate(i, j))
                    continue;
                double angle = fabs(robots[i].direction - robots[j].direction);
                angle = gt(angle, Pi)? 2 * Pi - angle: angle;
                if(lt(angle, Pi * 0.5)) continue;

                // ins[stopID].rotate = gt(ins[goID].rotate, 0)? Pi: -Pi;
                // ins[goID].rotate = gt(ins[goID].rotate, 0)? Pi: -Pi;

                // ins[stopID].rotate = gt(robots[goID].angular_velocity, 0)? Pi: -Pi;
                // ins[goID].rotate = gt(robots[goID].angular_velocity, 0)? Pi: -Pi;

                getAvoidDirection(goID, stopID);
                ins[stopID].rotate = ins[goID].rotate;

                flag =false;
                keep_state[i] = keep_state[i]? keep_state[i]: 10;
                keep_state[j] = keep_state[j]? keep_state[j]: 10;

                // ins[stopID].forward = -2;

                // getAvoidDirection(stopID, goID);
                // getAvoidDirection(j, i);
                
                // todo

                if(true) {
                    cerr<<"time:"<<state.FrameID<<endl;
                    cerr<<angle<<endl;
                    cerr << "stopID:" <<stopID <<"-"<<robots[stopID].target_id<<endl;
                    // cerr<<"**"<<robots[stopID].get_type<<endl;
                    cerr << "speed"<<calVectorSize(robots[stopID].xy_pos)<<" dir:"<<robots[stopID].direction<<endl;
                    cerr <<"a_speed"<<robots[stopID].angular_velocity<<endl;
                    cerr << "rate:"<<ins[stopID].rotate<<endl;
                    cerr<<"**"<<endl;
                    cerr<<" goID:"<<goID<<"-"<<robots[goID].target_id<<endl;
                    // cerr<<"**"<<robots[goID].get_type<<endl;
                    cerr << "speed"<<calVectorSize(robots[goID].xy_pos)<<" dir:"<<robots[goID].direction<<endl;
                    cerr <<"a_speed"<<robots[goID].angular_velocity<<endl;
                    cerr << "rate:"<<ins[goID].rotate<<endl<<endl;
                }
                
                //判断停下来的球是否会阻挡路线
                // next_pos = getNextPos(goID);
                // if(checkRobortsCollison(goID, next_pos, stopID)) {

                //     ins[goID].rotate = - Pi * robots[goID].lastSign;

                //     if(lt(robots[stopID].lastSign * robots[goID].lastSign, 0)) {
                //         ins[stopID].rotate = - Pi * robots[stopID].lastSign;
                //     }
                //     else {
                //         ins[stopID].rotate = Pi * robots[stopID].lastSign;
                //     }
                // }
            }
        }
        if(flag && keep_state[i]) {
            ins[i].rotate = gt(robots[i].angular_velocity, 0)? Pi: -Pi;
            keep_state[i]--;
        }
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
        if(robots[i1].get_type!=robots[i1].get_type)
        return robots[i1].get_type>robots[i1].get_type;
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
        if(dis<3&&!can_st){
                ins[i].rotate=((isSame==1&&isTurn==0)?Pi*payLoad[i].sign:max(0.5,Dec_val_ra*lastRate)*payLoad[i].sign);
                // if(i==0)
                // cerr<<"~"<<ins[i].rotate<<" "<<isSame<<"+"<<payLoad[i].angle<<"+" <<Dec_val_ra*lastRate*payLoad[i].sign<<endl;
                // if(robots[i]..forward>=3)
                // ins[i].forward=-2;
                ins[i].forward=0;
                robots[i].lastRate=ins[i].rotate;   
                continue;         
        }
        if(isWall_r(i,payLoad[i].angle)){
                ins[i].rotate=((isSame==1&&isTurn==0)?Pi*payLoad[i].sign:max(0.5,Dec_val_ra*lastRate)*payLoad[i].sign);
                // if(i==0)
                // cerr<<"~"<<ins[i].rotate<<" "<<isSame<<"+"<<payLoad[i].angle<<"+" <<Dec_val_ra*lastRate*payLoad[i].sign<<endl;
                // if(robots[i]..forward>=3)
                // ins[i].forward=-2;
                ins[i].forward=0.5;
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
            ins[i].forward=0.5;
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
            ins[i].rotate=((isSame==1&&isTurn==0)?Pi*payLoad[i].sign:max(0.5,Dec_val_ra*lastRate)*payLoad[i].sign);
                            // if(i==0)
                // if(i==0)
                // cerr<<"+"<<ins[i].rotate<<" "<<isSame<<"+"<<payLoad[i].angle<<"+" <<Dec_val_ra*lastRate*payLoad[i].sign<<endl;
            robots[i].lastRate=ins[i].rotate;
        }
        
    }
    
    
    // sort(arr.begin(),arr.end(),cmp);
    // vector<bool>vis(4,false);
    // for(int i=0;i<4;i++){
        
    //     if(robots[arr[i]].get_type==0)break;
    //     for(int j=i+1;j<4;j++){
    //         int tmp=special_test(arr[i],arr[j]);
    //         if(vis[j])continue;
    //         if(tmp){
    //             ins[arr[j]].forward*=0.5;
    //             vis[j]=true;
                
    //         }
    //     }
    // }
    // sort(arr.begin(),arr.end(),cmp);
// if(state.FrameID>=150&&state.FrameID<=450){
//     cerr<<"---";
//     cerr<<state.FrameID<<endl;
//  for(int i=0;i<4;i++)cerr<<arr[i]<<" ";
//  cerr<<endl;
//   cerr<<"---";
// }
   
        // for(int i=0;i<4;i++){
        
        //     Studio studio=studios[robots[i].target_id];
        //     Robot robort=robots[i];
        //     pair<double, double> robortToStudio = subVector(studio.pos, robort.pos);
        // for(int j=i+1;j<4;j++){
           
        //     Studio studio1=studios[robots[i].target_id];
        //     Robot robort1=robots[i];
        //     pair<double, double> robortToStudio1 = subVector(studio1.pos, robort1.pos);
        //     double tmpAngle1=get_angle(robort.direction,robort1.direction);
        //     double tmpAngle2=get_angle(robortToStudio,robortToStudio1);
        //     if(state.FrameID>=150&&state.FrameID<=450)
        //     // cerr<<i<<"-"<<j<<" "<<tmpAngle1<<" "<<tmpAngle2<<endl;
        //     double dis_stop=ins[arr[i]].forward*ins[arr[i]].forward/2*payLoad[arr[i]].acceleration;
        //     double tmpDis=calcuDis(robots[arr[i]].pos,robots[arr[j]].pos);
        //     // if(lt(tmpDis,5)&&
        //     // (gt(tmpAngle1,0.7)||gt(tmpAngle2,0.7))){
        //     //     if(lt(tmpDis,1.2)){
        //     //         ins[arr[j]].forward=0.5;
        //     //         ins[arr[j]].rotate=payLoad[j].sign*Pi;
        //     //     }else
        //     //         ins[arr[j]].forward=ins[i].forward*0.5;
        //     // }
        //     if(lt(calcuDis(robots[arr[i]].pos,robots[arr[j]].pos),5)&&is_less(i,j)&&(lt(tmpAngle1,-0.7)&&lt(tmpAngle2,-0.7))
        //     ){
        //         ins[arr[j]].forward=0;
        //         // ins[arr[j]].rotate=payLoad[j].sign*Pi;
        //     }
        // }
//     if(state.FrameID>=150&&state.FrameID<=450){
//         cerr<<ins[arr[i]].forward<<" "<<ins[arr[i]].rotate<<endl;
//         cerr<<"~~~~";
// }
    // }

    solveRobortsCollision();
   
    out_put();
}


/*
  control target_id
*/
pair<int,double> pick_point(int robot_id, int state){
    pair<double,double> pos = robots[robot_id].pos;
    int min = 10000;
    int min_subscript = -1;
    int i;
    int dist;
    int item_type = robots[robot_id].get_type;
    if(state == 1){
        for(i=0;i<=studios.size();i++){
            //cerr<<studios.size()<<endl;
            if(studios[i].type >= 1 && studios[i].type <= 3 && studios[i].r_id==-1){  //123 and no robot choose ,first choose ,get
                //cerr<<i<<endl;
                dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                //cerr<<"dist="<<dist<<endl;
                //cerr<<"studio_id"<<i<<endl;
                if(dist<min){
                    min=dist;
                    min_subscript=i;
                }
            }
        }
    }
    else if(state == 2){
        for(i=0;i<studios.size();i++){
            if(studios[i].type >= 1 && studios[i].type <= 3 && studios[i].r_id==-1 && studios[i].pStatus == 1){  //123 and no robot choose ,get
                if(material[studios[i].type].size()>0 && robot_get_type[studios[i].type]< material[studios[i].type].size()){
                    dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                    if(dist<min){
                        min=dist;
                        min_subscript=i;
                    }
                }
            }
        }
    }
    else if(state == 3){
        for(i=0;i<studios.size();i++){
            if(studios[i].type >= 4 && studios[i].type <= 6 && studios[i].r_id==-1 && studios[i].pStatus == 1){  //456 and no robot choose ,get
                if(material[studios[i].type].size()>0&& robot_get_type[studios[i].type]< material[studios[i].type].size()){
                    //&& robot_get_type[studios[i].type]< material[studios[i].type].size()
                    dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                    if(dist<min){
                        min=dist;
                        min_subscript=i;
                    }
                }
            }
        }
    }
    else if(state == 4){
        for(i=0;i<studios.size();i++){
            if(studios[i].type ==7 && studios[i].r_id==-1 && studios[i].pStatus == 1){  //7 and no robot choose ,get
                if(material[studios[i].type].size()>0){
                    dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                    if(dist < min){
                        min=dist;
                        min_subscript=i;
                    }
                }
            }
        }
    }
    else if(state == 5){               //send
        if(item_type == 1){
            for(i=0;i<studios.size();i++){
                if(((studios[i].type == 4 || studios[i].type == 5 ) && (studios_rid[i][item_type] == -1) && (studios[i].bitSatus & 2)==0 )|| studios[i].type == 9){  //1 and no robot choose ,send
                    dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                    if(dist<min){
                        min=dist;
                        min_subscript=i;
                    }
                }
            }
        }
        else if(item_type == 2){
            for(i=0;i<studios.size();i++){
                if(((studios[i].type == 4 || studios[i].type == 6 ) && (studios_rid[i][item_type] == -1) && (studios[i].bitSatus & 4)==0) || studios[i].type == 9){  //2 and no robot choose ,send
                    dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                    if(dist<min){
                        min=dist;
                        min_subscript=i;
                    }
                }
            }
        }
        else if(item_type == 3){
            for(i=0;i<studios.size();i++){
                if(((studios[i].type == 5 || studios[i].type == 6) && (studios_rid[i][item_type] == -1) && (studios[i].bitSatus & 8)==0 )|| studios[i].type == 9){  //3 and no robot choose ,send
                    dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                    if(dist<min){
                        min=dist;
                        min_subscript=i;
                    }
                }
            }
        }
        else if(item_type == 4){
            for(i=0;i<studios.size();i++){
                if((studios[i].type == 7 && (studios_rid[i][item_type] == -1) && (studios[i].bitSatus & 16)==0 )|| studios[i].type == 9){  //4 and no robot choose ,send
                    dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                    if(dist<min){
                        min=dist;
                        min_subscript=i;
                    }
                }
            }
        }
        else if(item_type == 5){
            for(i=0;i<studios.size();i++){
                if((studios[i].type == 7 && (studios_rid[i][item_type] == -1) && (studios[i].bitSatus & 32)==0 ) || studios[i].type == 9){  //5 and no robot choose ,send
                    dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                    if(dist<min){
                        min=dist;
                        min_subscript=i;
                    }
                }
            }
        }
        else if(item_type == 6){
            for(i=0;i<studios.size();i++){
                if(((studios[i].type == 7) && (studios_rid[i][item_type] == -1) && (studios[i].bitSatus & 64)==0)|| studios[i].type == 9) {  //6 and no robot choose ,send
                    dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                    if(dist<min){
                        min=dist;
                        min_subscript=i;
                    }
                }
            }
        }
        else if(item_type == 7){
            for(i=0;i<studios.size();i++){
                if((studios[i].type == 8 ||studios[i].type == 9)){  //7 and no robot choose ,send
                    dist=calcuDis(robots[robot_id].pos,studios[i].pos);
                    if(dist<min){
                        min=dist;
                        min_subscript=i;
                    }
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
        robots[i].target_id = pick_point(i, 1).first;
    }
    for(i = 0; i < robots.size(); i++)
    {
        for(j = i + 1; j < robots.size(); j++){
            if(robots[i].target_id == robots[j].target_id){
                p = pick_point(i,1);
                f = pick_point(j,1);
                if(gt(p.second,f.second)){
                    studios[robots[i].target_id].r_id = i;
                    robots[j].target_id = f.first;
                    studios[robots[j].target_id].r_id = j;
                }else {
                    studios[robots[j].target_id].r_id = j;
                    robots[i].target_id = p.first;
                    studios[robots[i].target_id].r_id = i;
                }
            }
        }
        //studios[robots[i].target_id].r_id = i;
        //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
    }
}

bool judge_full(int level, double threshold){
    int i;
    int count = 0;
    int full_count = 0;
    double v;
    if(level == 2){
        for(i = 0;i < studios.size();i++){
            if(studios[i].type >= 4 && studios[i].type <= 6){
                count++;
                if(studios[i].pStatus == 1 && studios[i].r_id == -1){
                    full_count++;
                }
            }
        }
        v = (double)full_count/(double)count;
        //cerr<<" full_count = "<<full_count<<" count = "<<count<<" bilu "<<v<<endl;
        if(v >= threshold){
            return true;
        }
    }
    if(level == 3){
        for(i = 0;i < studios.size();i++){
            if(studios[i].type == 7){
                count++;
                if(studios[i].pStatus == 1 && studios[i].r_id == -1){
                    full_count++;
                }
            }
        }
        v = (double)full_count/(double)count;
        //cerr<<" full_count = "<<full_count<<" count = "<<count<<" bilu "<<v<<endl;
        if(v >= threshold){
            return true;
        }
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
                    if(!checkeTimeEnough(i,robots[i].target_id,9000-state.FrameID))ins[i].buy = -1;
                }
            }
            else{
                //dosomething sell
                ins[i].sell = 1;
                ins[i].buy = -1;
                robots[i].lastSign=0;
                robots[i].isTurn=0;
                // cerr<<"robots "<< i<<" sell "<<robots[i].get_type<<endl;
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

void robot_action(){
    //cerr<<"start"<<endl;
    //print_matr();
    for(int i =0;i<=7;i++)robot_get_type[i]=0;
    for(int i = 0;i<4;i++){
        if(robots[i].get_type != 0)robot_get_type[robots[i].get_type]++;
        else if(robots[i].target_id != -1)robot_get_type[studios[robots[i].target_id].type]++;
    }
    //for(int i =0;i<=7;i++)cerr<<"type "<<i<<" has "<<robot_get_type[i];
    // cerr <<endl;
    int full = 0;
    if(judge_full(2,0.1))full = 1;   //4,5,6 full threshold
    if(judge_full(3,0.2))full = 2;   //7 full threshold Higher priority
    //cerr<<" full = "<<full<<endl;
    robot_judge(full,1.5,5);
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
    
        return lt(dis1,dis);
}
bool who_isFirst(int i1,int i2){
    pair<double, double> p1 = subVector(robots[i1].pos, robots[i2].pos);
    double s1=robots[i1].direction;
    pair<double, double> p2=make_pair<double ,double>(cos(s1),sin(s1));
    double tmpCos=get_angle(p1,p2);
    return gt(tmpCos,0.0);
}