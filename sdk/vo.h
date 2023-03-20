#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <string.h>
#include <math.h>
#include"vec.h"
using namespace std;
const double eps = 1e-8;
vector<Vec> get_rAndf(Circle  circle,pair<double,double>pos);//返回相对坐标下的碰撞速度
Vec transmit_v_v1(Vec v,Vec v1);//绝对速度转相对速度
Vec transmit_v1_v(Vec v1,Vec v);//相对速度转绝对速度
Vec achievable_speed(Vec v, double angular_acceleration, double acceleration, double delta_t);
vector<Vec> achievable_speed(Vec v,double angular_acceleration,double acceleration);
//给定加速度角加速度的情况下，可达速度

vector<vector<Vec>> speed_set_difference(vector<Vec> v1,vector<Vec> v2);//求两个速度集合的差
Vec suitable_speeds(Vec v,vector<vector<Vec>>v_set);//从速度集合中选取最接近原速度的v
pair<double,double>  Calculation_results(Vec v,Vec v1);//计算从速度1变换到速度2需要下达的forwad指令和rotate指令
double Dis_o(Circle  circle,pair<double,double>pos);//计算圆心到原点的距离
