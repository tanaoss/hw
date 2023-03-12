#pragma once
#include <iostream>
#include <vector>
#include<string>
using namespace std;

struct payLoad
{
    double angle;
    double angular_acceleration;
    double acceleration;
    double distance;
    int sign;//当前角速度是在贴合目标点的夹角还是远离目标点的夹角
};


struct  State
{
    int FrameID; 
    int money;
};

struct Ins
{
    int robID;
    double forward;
    double rotate;
    int buy;
    int sell;
    int destroy;
    Ins(int _robID, double _forward, double _rotate, int _buy = -1, int _sell = -1, int _destroy = -1) : robID(_robID), forward(_forward), rotate(_rotate), buy(_buy), sell(_sell), destroy(_destroy)
    {
    }
    friend std::ostream &operator<<(std::ostream &os, Ins ins)
    {
        std::cout << "forward " << ins.robID << " " << ins.forward << "\n"
             << "rotate " << ins.robID << " " << ins.rotate << std::endl;
        if (ins.buy != -1)
        {
            std::cout << "buy " << ins.robID << std::endl;
        }
        if (ins.sell != -1)
        {
            std::cout << "sell " << ins.robID << std::endl;
        }
        if (ins.destroy != -1)
        {
            std::cout << "destroy " << ins.robID << std::endl;
        }
        return os;
    }
};

struct Robot
{
    int id;
    int loc_id;//所处工作台id
    int get_type; // 携带物品类型
    double time_val;
    double collision_val;
    double angular_velocity;
    pair<double, double> xy_pos;
    double direction;
    pair<double, double> pos;
    int target_id; // 正在赶往的工作台；
    Robot(int _id, int _loc_id,int _get_type, double _time_val, double _collision_val, double _angular_velocity, pair<double, double> &_xy_pos,
          double _direction, pair<double, double> &_pos, int _target_id = 0) : xy_pos(_xy_pos), pos(_pos)
    {
        id = _id;
        loc_id=_loc_id;
        get_type = _get_type;
        time_val = _time_val;
        collision_val = _collision_val;
        angular_velocity = _angular_velocity;
        direction = _direction;
        target_id = _target_id;
    }
    void set(int _id,  int _loc_id,int _get_type, double _time_val, double _collision_val, double _angular_velocity, pair<double, double> &&_xy_pos,
             double _direction, pair<double, double> &&_pos)
    {
        id = _id;
        loc_id=_loc_id;
        get_type = _get_type;
        time_val = _time_val;
        collision_val = _collision_val;
        angular_velocity = _angular_velocity;
        direction = _direction;
        xy_pos = _xy_pos;
        pos = _pos;
    }
};

struct Studio
{
    int id;
    int type;
    int r_id; // 正在赶来的机器人id
    pair<double, double> pos;
    int r_time;   // 剩余生产时间（帧数）
    int bitSatus; // 原材料格状态
    int pStatus;  // 产品格状态
    Studio(int _id, int _type,int _r_id, pair<double, double> &_pos, int _r_time, int _bitSatus, int _pStatus) : 
    id(_id),type(_type) ,r_id(_r_id), pos(_pos), r_time(_r_time), bitSatus(_bitSatus), pStatus(_pStatus)
    {
    }
    void set(int _id, int _type, pair<double, double> &&_pos, int _r_time, int _bitSatus, int _pStatus)
    {
        id = _id;
        type=_type;
        pos = _pos;
        r_time = _r_time;
        bitSatus = _bitSatus;
        pStatus = _pStatus;
    }
};
bool readMapUntilOK();
bool readStatusUntilOK();
void out_put(vector<Ins>&out);
double calcuDis(pair<double, double> a, pair<double, double> b);
void calcuStudioDis();