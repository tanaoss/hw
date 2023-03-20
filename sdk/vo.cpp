#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <string.h>
#include <math.h>
#include"vec.h"
#include"class.h"
using namespace std;
const double eps = 1e-8;
double Dis_o(Circle  circle,pair<double,double>pos){
    return sqrt(pow(circle.pos.first-pos.first,2)+pow(circle.pos.second-pos.second,2));
}
vector<Vec> get_rAndf(Circle  circle,pair<double,double>pos){
    double redundancy=0.0;
    double angle_add= asin(circle.r/Dis_o(circle,pos))+redundancy;
    double angle_base=acos(cos_t(Vec{circle.pos},Vec{1,0}));
    return vector<Vec>{{cos(angle_base-angle_add),sin(angle_base-angle_add)},
    {cos(angle_base+angle_add),sin(angle_base-angle_add)}};

}
Vec transmit_v_v1(Vec v,Vec v1){//绝对速度转相对速度
    return v-v1;
}
Vec transmit_v1_v(Vec v1,Vec v){
    return v+v1;
}
// 计算在给定加速度和角加速度约束下，机器人在指定时间内可达的最大速度
Vec achievable_speed(Vec v, double angular_acceleration, double acceleration, double delta_t) {
    double v_max = v.x + acceleration * delta_t;
    double w_max = v.theta + angular_acceleration * delta_t;

    double v_min = max(0.0, v.x - acceleration * delta_t);
    double w_min = max(0.0, v.theta - angular_acceleration * delta_t);

    Vec result;
    result.x = (v_max + v_min) / 2.0;
    result.y = 0.0;
    result.theta = (w_max + w_min) / 2.0;

    return result;
}
vector<Vec> achievable_speed(Vec v,double angular_acceleration,double acceleration){//给定加速度角加速度的情况下，可达速度
    vector<Vec> speeds;

    for (double delta_t = 0.1; delta_t <= 10.0; delta_t += 0.1) {
        Vec achievable = achievable_speed(v, angular_acceleration, acceleration, delta_t);
        speeds.push_back(achievable);
    }

    return speeds;
}
vector<vector<Vec>> speed_set_difference(vector<Vec> v1,vector<Vec> v2){//求两个速度集合的差

}
Vec suitable_speeds(Vec v,vector<vector<Vec>>v_set){//从速度集合中选取最接近原速度的v
    Vec sub;
    Vec close_v;     
    double min=100;
    double sub_v;

    for(int i=0;i<v_set.size();i++){
        for(int j=0;j<v_set[i].size();j++){
            sub.x=v.x-v_set[i][j].x;               //集合中的速度与原速度向量相减
            sub.y=v.y-v_set[i][j].y;
            sub_v=sqrt(sub.x*sub.x+sub.y*sub.y);      //sub向量的绝对大小
            if(min>sub_v){
                min=sub_v;
                close_v.x=v_set[i][j].x;
                close_v.y=v_set[i][j].y;
            }
        }
    }
    return close_v
}
pair<double,double>  Calculation_results(Vec v,Vec v1){//计算从速度1变换到速度2需要下达的forwad指令和rotate指令

}
double Dis_o(Circle  circle,pair<double,double>pos){//计算圆心到原点的距离
    return calcuDis(circle.pos,pos);
}