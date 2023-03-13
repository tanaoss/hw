#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include<algorithm>
#include "class.h"
using namespace std;
vector<vector<double>> dis(50, vector<double>(50, 0));
vector<Studio> studios;
vector<Robot> robots;
State state;//当前帧数，全局可见
vector<Ins> ins(4);
vector<int> material[8];
vector<vector<int>> product(8);
double EPS=1e-7;
double acceleration_no;
double acceleration_has;
double angular_acceleration_no;
double angular_acceleration_has;
vector<bool> need_stop(4,false);
int robot_get_type[8];
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
    for(int i=0;i<7;++i) material[i].clear();
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
            if(studios[studio_id].type == 4){
                //cerr <<"aaaa"<<studios[studio_id].bitSatus<<endl;
                if((studios[studio_id].bitSatus & 2) == 0 && (studios[studio_id].r_id == -1 ||(studios[studio_id].r_id != -1 && robots[studios[studio_id].r_id].get_type != 1))) material[1].push_back(studio_id);
                if((studios[studio_id].bitSatus & 4) == 0 && (studios[studio_id].r_id == -1 ||(studios[studio_id].r_id != -1 && robots[studios[studio_id].r_id].get_type != 2))) material[2].push_back(studio_id);
            }
            if(studios[studio_id].type == 5){
                if((studios[studio_id].bitSatus & 2) == 0 && (studios[studio_id].r_id == -1 ||(studios[studio_id].r_id != -1 && robots[studios[studio_id].r_id].get_type != 1))) material[1].push_back(studio_id);
                if((studios[studio_id].bitSatus & 8) == 0 && (studios[studio_id].r_id == -1 ||(studios[studio_id].r_id != -1 && robots[studios[studio_id].r_id].get_type != 3))) material[3].push_back(studio_id);
            }
            if(studios[studio_id].type == 6){
                if((studios[studio_id].bitSatus & 4) == 0 && (studios[studio_id].r_id == -1 ||(studios[studio_id].r_id != -1 && robots[studios[studio_id].r_id].get_type != 2))) material[2].push_back(studio_id);
                if((studios[studio_id].bitSatus & 8) == 0 && (studios[studio_id].r_id == -1 ||(studios[studio_id].r_id != -1 && robots[studios[studio_id].r_id].get_type != 3))) material[3].push_back(studio_id);
            }
            if(studios[studio_id].type == 7){
                if((studios[studio_id].bitSatus & 16) == 0 && (studios[studio_id].r_id == -1 ||(studios[studio_id].r_id != -1 && robots[studios[studio_id].r_id].get_type != 4))) material[4].push_back(studio_id);
                if((studios[studio_id].bitSatus & 32) == 0 && (studios[studio_id].r_id == -1 ||(studios[studio_id].r_id != -1 && robots[studios[studio_id].r_id].get_type != 5))) material[5].push_back(studio_id);
                if((studios[studio_id].bitSatus & 64) == 0 && (studios[studio_id].r_id == -1 ||(studios[studio_id].r_id != -1 && robots[studios[studio_id].r_id].get_type != 6))) material[6].push_back(studio_id);
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
    for(i = 1 ; i < 7; i++){
        cerr << "kkkkkkk"<<material[i].size()<<endl;
        for(j=0;j<material[i].size();j++) 
            cerr<<"mater "<<i<<"studio "<<material[i][j]<<endl;
    }
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
    double angle1 = acos(robortToStudio.first / calVectorSize(robortToStudio));
    angle1 = lt(robortToStudio.second, 0.0) ? 2 * Pi- angle1: angle1;

    double angle2 = ge(robort.direction, 0.0) ? robort.direction: 2 * Pi + robort.direction;
    double angle = angle2 - angle1;

    int sign;

    if(ge(angle, 0) && lt(angle, Pi) || lt(angle, -Pi))
        sign = -1;
    else
        sign = 1;
    angle = fabs(angle);
    angle  = gt(angle, Pi)? angle - Pi: angle;


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
    return lt(getRobotRadius(robotA_id) + getRobotRadius(robotB_id), calcuDis(robotA.pos, robortB.pos));
}

bool checkRobortsCollison(int robotA_id, pair<double, double> next_pos, int robotB_id) {
    Robot robotA = robots[robotA_id];
    Robot robortB = robots[robotB_id];
    return lt(getRobotRadius(robotA_id) + getRobotRadius(robotB_id), calcuDis(next_pos, robortB.pos));
}

bool checkeTimeEnough(int robot_id, int target_id, int frame) {
    double dis = calcuDis(robots[robot_id].pos, studios[target_id].pos);
    double time = (9000.0 - frame) * 0.02;//剩余秒数
    return true;
}

pair<double, double> getNextPos(int robot_id) {
    return addVector(robots[robot_id].pos, calVectorProduct(robots[robot_id].pos, 0.02));
}


void solveRobortsCollison() {
    int stopID, goID;
    pair<double, double> next_pos;
    for(int i = 0; i < 4; i++) {
        for(int j = i + 1; j < 4; j++) {
            if(checkRobortsCollison(i, j)) {
                //优先级小的先停下来
                stopID = robots[i] < robots[j] ? i: j;
                goID = robots[i] < robots[j] ? j: i;
                ins[i].forward = 0;
                //判断停下来的球是否会阻挡路线
                next_pos = getNextPos(goID);
                if(checkRobortsCollison(goID, next_pos, stopID)) {
                    if(eq(ins[goID].rotate, 0)) {
                        ins[goID].rotate = Pi;
                    }
                }
            }
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
    auto check=[&](int rid)->bool{
        double radius=robots[rid].get_type==0? 0.45:0.53;
        double n_x=robots[rid].pos.first+robots[rid].xy_pos.first*time,n_y=robots[rid].pos.second+robots[rid].xy_pos.second*time;
        if(lt(n_x-radius,0)||lt(n_y-radius,0)||gt(n_x+radius,50)||gt(n_y+radius,50))
        return true;
        return false;
    };//判断是否有可能撞墙
    auto cmp=[&](int id1,int id2)->bool{
        return robots[id1].get_type>robots[id2].get_type;
    };
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
        bool can_st=can_stop(robots[i].pos,studios[robots[i].target_id].pos,fabs(payLoad[i].angle));
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
        // int can_st_flag=1;
        if(can_st&&ins[i].rotate==0){
            if(can_speed_z(robots[i].target_id,robots[i].xy_pos,robots[i].pos,payLoad[i].acceleration)){
            cerr<<"~"<<payLoad[i].angle<<" "<<robots[i].direction<<" "<<i
            <<"~"<<robots[i].target_id<<" "<<robots[i].xy_pos.first<<" "<<robots[i].xy_pos.second<<endl;
                ins[i].forward=0;
                // can_st_flag=0;
            }else{
                ins[i].forward=6;
            }
        }else if(check(robID)){
            ins[i].forward=0.5;
        }else{
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
                if((studios[i].type == 4 || studios[i].type == 5 || studios[i].type == 9) && (studios[i].r_id==-1 || ((studios[i].r_id !=-1) && (robots[studios[i].r_id].get_type != item_type))) && (studios[i].bitSatus & 2)==0 ){  //1 and no robot choose ,send
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
                if((studios[i].type == 4 || studios[i].type == 6 || studios[i].type == 9) && (studios[i].r_id==-1 || ((studios[i].r_id !=-1) && (robots[studios[i].r_id].get_type != item_type))) && (studios[i].bitSatus & 4)==0 ){  //2 and no robot choose ,send
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
                if((studios[i].type == 5 || studios[i].type == 6 || studios[i].type == 9) && (studios[i].r_id==-1 || ((studios[i].r_id !=-1) && (robots[studios[i].r_id].get_type != item_type))) && (studios[i].bitSatus & 8)==0 ){  //3 and no robot choose ,send
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
                if((studios[i].type == 7 || studios[i].type == 9) && (studios[i].r_id==-1 || ((studios[i].r_id !=-1) && (robots[studios[i].r_id].get_type != item_type))) && (studios[i].bitSatus & 16)==0 ){  //4 and no robot choose ,send
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
                if((studios[i].type == 7 ||studios[i].type == 9) && (studios[i].r_id==-1 || ((studios[i].r_id !=-1) && (robots[studios[i].r_id].get_type != item_type))) && (studios[i].bitSatus & 32)==0 ){  //5 and no robot choose ,send
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
                if((studios[i].type == 7 || studios[i].type == 9) && (studios[i].r_id==-1 || ((studios[i].r_id !=-1) && (robots[studios[i].r_id].get_type != item_type))) && (studios[i].bitSatus & 64)==0) {  //6 and no robot choose ,send
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
                if((studios[i].type == 8 ||studios[i].type == 9)&&(studios[i].r_id==-1 || ((studios[i].r_id !=-1) && (robots[studios[i].r_id].get_type != item_type))) && (studios[i].bitSatus & 128)==0 ){  //7 and no robot choose ,send
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
                if(studios[i].pStatus == 1 && studios[i].r_id != -1){
                    full_count++;
                }
            }
        }
        v = (double)full_count/(double)count;
        if(v >= threshold){
            return true;
        }
    }
    if(level == 3){
        for(i = 0;i < studios.size();i++){
            if(studios[i].type == 7){
                count++;
                if(studios[i].pStatus == 1 && studios[i].r_id != -1){
                    full_count++;
                }
            }
        }
        v = (double)full_count/(double)count;
        if(v >= threshold){
            return true;
        }
    }
    return false;
}

void robot_judge(int full){
    int i;
    //cerr<<robots.size()<<endl;
    for(i = 0; i < robots.size(); i++){
        //cerr<<i<<robots[i].target_id<<endl;
        if(robots[i].loc_id == robots[i].target_id && robots[i].target_id != -1){
            if(robots[i].get_type == 0){
                //dosomething buy ,next send
                ins[i].buy = 1;
                robots[i].lastSign=0;
                robots[i].isTurn=0;
                //cerr<<"robots "<< i<<" buy "<<endl;
                robots[i].get_type = studios[robots[i].loc_id].type;
                studios[robots[i].loc_id].r_id = -1;
                robots[i].target_id = pick_point(i,5).first;
                if(robots[i].target_id!= -1){
                    studios[robots[i].target_id].r_id = i;
                    //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
                }
            }
            else{
                //dosomething sell
                ins[i].sell = 1;
                robots[i].lastSign=0;
                robots[i].isTurn=0;
                
                //cerr<<"robots "<< i<<" sell "<<endl;
                studios[robots[i].loc_id].r_id = -1;
                robots[i].get_type = 0;
                if(full == 1){
                    robots[i].target_id = pick_point(i,3).first; //find near 456
                    if(robots[i].target_id!= -1){
                        //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
                        studios[robots[i].target_id].r_id = i;
                    }
                }
                else if(full == 2){
                    robots[i].target_id = pick_point(i,4).first; //find near 7
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
        else{
            ins[i].buy = -1;
            ins[i].sell = -1;
            ins[i].destroy = -1;
        }
        if(robots[i].target_id == -1){
            if(robots[i].get_type ==0){
                robots[i].target_id = pick_point(i,1).first; //no target
                if(robots[i].target_id!= -1){
                        //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
                    studios[robots[i].target_id].r_id = i;
                    //cerr<< "kkkkk"<<endl;
                }
            }
            else{
                robots[i].target_id = pick_point(i,5).first;
                if(robots[i].target_id!= -1){
                    studios[robots[i].target_id].r_id = i;
                        //cerr<< "robots "<< i<<" target_id = "<<robots[i].target_id <<" get_type = "<<robots[i].get_type<<" target_type= "<<studios[robots[i].target_id].type<<endl;
                }
            }
        }
        
        // if(i==2 && robots[i].target_id != -1){
        //     cerr<<" robot 2 dis "<<calcuDis(robots[i].pos,studios[robots[i].target_id].pos)<<endl;
        // }
    }
}

void robot_action(){
    //cerr<<"start"<<endl;
    for(int i =0;i<=7;i++)robot_get_type[i]=0;
    for(int i = 0;i<4;i++){
        if(robots[i].get_type != 0)robot_get_type[robots[i].get_type]++;
        else if(robots[i].target_id != -1)robot_get_type[studios[robots[i].target_id].type]++;
    }
    int full = 0;
    if(judge_full(2,0.1))full = 1;   //4,5,6 full threshold
    if(judge_full(3,0.5))full = 2;   //7 full threshold Higher priority
    robot_judge(full);
}

vector<double>  get_T_limits(pair<double,double>pos,int id){
    double radius=robots[id].get_type==0? 0.45:0.53;
   
    vector<double> tmp{-7,-7};
    double redundancy=0.1+radius;//冗余，避免频繁转向
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
    double dis=calcuDis(p1,p2);
    if(lt(sin(angle)*dis,0.4)){
        return true;
    }
    return false;

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
    cerr<<stuID<<" "<<dis3<<" "<<dis4<<" "<<dis5<<" "<<dis1<<endl;
    if(ge(dis3,dis5-dis4))return true;
    return false;
}
