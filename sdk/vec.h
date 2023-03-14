#pragma once
#include <iostream>
#include <vector>
#include<string>
#include<cmath>
using namespace std;
struct Vec
{
    double x;
    double y;
    Vec(double _x,double _y){
        x=_x;
        y=_y;
    }
    Vec(pair<double,double>p1){
        x=p1.first;
        y=p1.second;
    }
};
Vec r90a(Vec v) { return {-v.y, v.x}; }                          // 逆时针旋转90度的向量
Vec r90c(Vec v) { return {v.y, -v.x}; }                          // 顺时针旋转90度的向量
Vec operator+(Vec u, Vec v) { return {u.x + v.x, u.y + v.y}; }   // 向量加向量
Vec operator-(Vec u, Vec v) { return {u.x - v.x, u.y - v.y}; }   // 向量减向量
Vec operator*(double k, Vec v) { return {k * v.x, k * v.y}; }    // 数乘
double operator*(Vec u, Vec v) { return u.x * v.x + u.y * v.y; } // 点乘
double operator^(Vec u, Vec v) { return u.x * v.y - u.y * v.x; } // 叉乘
double len(Vec v) { return sqrt(v.x * v.x + v.y * v.y); }        // 向量长度
double slope(Vec v) { return v.y / v.x; }                        // 斜率 // NOTE 不要用isinf判断斜率不存在，用后面的paral_y
double cos_t(Vec u, Vec v) { return u * v / len(u) / len(v); }
double get_angle(double s1,double s2);
double get_angle(pair<double,double> p1,pair<double,double> p2);
bool is_less(int i1,int i2);
bool who_isFirst(int i1,int i2);