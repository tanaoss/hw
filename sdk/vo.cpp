#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <string.h>
#include <math.h>
#include"vec.h"
using namespace std;
const double eps = 1e-8;
double Dis_o(Circle  circle){
    return sqrt(pow(circle.pos.first,2)+pow(circle.pos.second,2));
}
vector<Vec> get_rAndf(Circle  circle){
    double redundancy=0.0;
    double angle_add= asin(circle.r/Dis_o(circle))+redundancy;
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
vector<Vec> achievable_speed(Vec v,double angular_acceleration,double acceleration){//给定加速度角加速度的情况下，可达速度

}

vector<vector<Vec>> speed_set_difference(vector<Vec> v1,vector<Vec> v2);//求两个速度集合的差
Vec suitable_speeds(Vec v,vector<vector<Vec>>v_set);//从速度集合中选取最接近原速度的v
pair<double,double>  Calculation_results(Vec v,Vec v1);//计算从速度1变换到速度2需要下达的forwad指令和rotate指令
