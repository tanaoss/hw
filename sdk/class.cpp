#include <iostream>
#include <vector>
#include<string>
#include <cmath>
#include<algorithm>
#include "class.h"
using namespace std;
vector<vector<double>> dis(50, vector<double>(50, 0));
vector<Studio> studios;
vector<Robot> robots;
State state;//当前帧数，全局可见
vector<Ins> ins(4);
double EPS=1e-7;
bool readMapUntilOK() {
    char line[1024];
    int count = 0;
    int count_robot = 0, count_studio = 0;
    double x,y;
    int i;
    while (cin.getline(line,sizeof(line))) {
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
                // cout<<x<<" "<<y<<endl;
                Robot  robot(count_robot,0,0,0,1,1,xy_pos_robot,0,pos_robot,0);
                robots.push_back(robot);
                count_robot++;
            }
            else if(line[i] >= '1' && line[i] <= '9'){
                x = i*0.5+0.25;
                y = (100-count)*0.5-0.25;
                pair<double,double>pos_studio(x,y);
                // cout<<x<<" "<<y<<endl;
                Studio studio(count_studio,0,0,pos_studio,0,0,0);
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
    while (K--)
    {
        vector<double> tmp(6,0);
        for(int i=0;i<tmp.size();i++){
            cin>>tmp[i];
        }
        studios[studio_id].set(studio_id,tmp[0],pair<double,double>(tmp[1],tmp[2]),tmp[3],tmp[4],tmp[5]);
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

bool eq(double a, double b) { return abs(a - b) < EPS; } // ==
bool gt(double a, double b) { return a - b > EPS; }      // >
bool lt(double a, double b) { return a - b < -EPS; }     // <
bool ge(double a, double b) { return a - b > -EPS; }     // >=
bool le(double a, double b) { return a - b < EPS; }      // <=

void control(vector<PayLoad> payLoad){
    const double time=0.04;//预测的时间。
    const double rateLim=0.24434609528;//14度
    const double pie=3.141592654;
    const double Dec_val=0.4;//减速系数
    const double Dec_val_ra=0.5;//角速度减速系数
    vector<int> arr{0,1,2,3};
    auto check=[&](int rid)->bool{
        double radius=robots[rid].get_type==0? 0.45:0.53;
        double n_x=robots[rid].xy_pos.first*time,n_y=robots[rid].xy_pos.second*time;
        if(lt(n_x-radius,0)||lt(n_y-radius,0)||gt(n_x+radius,50)||gt(n_y+radius,50))
        return false;
        return true;
    };//判断是否有可能撞墙
    auto cmp=[&](int id1,int id2)->bool{
        return robots[id1].get_type>robots[id2].get_type;
    };
    for(int i=0;i<4;i++){
        int robID=robots[i].id;
        double Dev_val=robots[i].angular_velocity*robots[i].angular_velocity/2*payLoad[i].angular_acceleration;
        if(check(robID)){
            ins[i].forward*=Dec_val;
        }else{
            ins[i].forward=6;
        }
        if(gt(Dev_val,payLoad[i].angle)){
            ins[i].rotate=0;
        }else{
            if(gt(payLoad[i].angle,rateLim)){
                ins[i].rotate=pie*payLoad[i].sign;
            }else{
                ins[i].rotate*=Dec_val_ra;
            }
        }
    }
    sort(arr.begin(),arr.end());
    for(int i=0;i<4;i++){
        double dis_stop=ins[arr[i]].forward*ins[arr[i]].forward/2*payLoad[arr[i]].acceleration;
        for(int j=i+1;j<4;j++){
            if(lt(calcuDis(robots[arr[i]].pos,robots[arr[j]].pos),dis_stop)){
                ins[arr[j]].forward*=Dec_val;
            }
        }
    }
    out_put();
}
