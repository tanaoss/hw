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
    Vec(){}
    Vec(double _x,double _y){
        x=_x;
        y=_y;
    }
    Vec(pair<double,double>p1){
        x=p1.first;
        y=p1.second;
    }
};
using Point_Ta = Vec;                     
struct Line_Ta { Point_Ta P; Vec v;
    Line_Ta(Point_Ta _p,Vec _v){
        P=_p;
        v=_v;
    }
 };    
struct Circle{
    double r;
    pair<double,double>pos;
    Circle(pair<double,double>_pos,double _r){
        pos=_pos;
        r=_r;
    }
};
Vec r90a(Vec v) { return {-v.y, v.x}; }                         
Vec r90c(Vec v) { return {v.y, -v.x}; }                         
Vec operator+(Vec u, Vec v) { return {u.x + v.x, u.y + v.y}; }   // 向量加向量
Vec operator-(Vec u, Vec v) { return {u.x - v.x, u.y - v.y}; }   // 向量减向量
Vec operator*(double k, Vec v) { return {k * v.x, k * v.y}; }    // 数乘
double operator*(Vec u, Vec v) { return u.x * v.x + u.y * v.y; } // 点乘
double operator^(Vec u, Vec v) { return u.x * v.y - u.y * v.x; } // 叉乘
double len(Vec v) { return sqrt(v.x * v.x + v.y * v.y); }        // 向量长度
double slope(Vec v) { return v.y / v.x; }                        // 斜率 
double cos_t(Vec u, Vec v) { return u * v / len(u) / len(v); }
double dis_lp(Point_Ta P, Line_Ta l) { return abs((P ^ l.v) - (l.P ^ l.v)) / len(l.v); }
Vec norm(Vec v) { return {v.x / len(v), v.y / len(v)}; }
double get_angle(double s1,double s2);
double get_angle(pair<double,double> p1,pair<double,double> p2);
bool is_less(int i1,int i2);
bool who_isFirst(int i1,int i2);
