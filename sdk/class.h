#pragma once
#include <iostream>
#include <vector>
#include<string>
using namespace std;

#define Pi 3.141592653589793

struct PayLoad
{
    double angle;//角度
    double angular_acceleration;//角加速度
    double acceleration;//加速度
    double distance;
    int sign;//当前角速度是在贴合目标点的夹角还是远离目标点的夹角
    PayLoad(double _angle,double _angular_acceleration,double _acceleration,double _distance,int _sign){
        angle=_angle;
        angular_acceleration=_angular_acceleration;
        acceleration=_acceleration;
        distance=_distance;
        sign=_sign;
    }
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
    Ins(int _robID=-1, double _forward=-1, double _rotate=-1, int _buy = -1, int _sell = -1, int _destroy = -1) : robID(_robID), forward(_forward), rotate(_rotate), buy(_buy), sell(_sell), destroy(_destroy)
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
    int lastSign;//
    double lastRate;
    double isTurn;
    Robot(int _id, int _loc_id,int _get_type, double _time_val, double _collision_val, double _angular_velocity, pair<double, double> &_xy_pos,
          double _direction, pair<double, double> &_pos, int _target_id = -1,int _lastSign=0,int _isTurn=0) : xy_pos(_xy_pos), pos(_pos)
    {
        id = _id;
        loc_id=_loc_id;
        get_type = _get_type;
        time_val = _time_val;
        collision_val = _collision_val;
        angular_velocity = _angular_velocity;
        direction = _direction;
        target_id = _target_id;
        lastSign=_lastSign;
        isTurn=_isTurn;
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

    bool operator< (const Robot& a) const {
        if (get_type == 0 && a.get_type != 0)
            return false;
        if (a.get_type == 0 && get_type != 0)
            return true;
        double speed2 = xy_pos.first * xy_pos.first;
        double speed2_a = a.xy_pos.first * a.xy_pos.first;
        if(speed2 - speed2_a > 1e-5)
            return false;
        if(speed2_a - speed2 > 1e-5)
            return true;
        if (get_type == get_type) 
            return time_val * collision_val - a.time_val * a.collision_val < -1e-7;
        return get_type < a.get_type;
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
struct Line { pair<double, double>  P; pair<double, double> v; };      // 直线（点向式）

bool eq(double a, double b);// ==
bool gt(double a, double b);// >
bool lt(double a, double b);// <
bool ge(double a, double b);// >=
bool le(double a, double b);// <=
void initRobortInfo();
bool readMapUntilOK();//读地图
bool readStatusUntilOK();//读判题器输出
void out_put();//输出指令
void calcuStudioDis();//计算工作台之间的距离
void control(vector<PayLoad> payLoad);//控制球体运行
void first_pick_point();
void robot_action();
void process();
PayLoad calPayload(int robortID);//计算机器人与目标之间的夹角、距离等信息
vector<double> get_T_limits(pair<double,double>pos,int id,int ctr=-1,double dis=0.0);//靠近墙体时，需要把方向转到那个范围才能加速
pair<double, double> subVector(pair<double, double> a, pair<double, double> b);//向量减（a-b）
double calVectorProduct(pair<double, double> a, pair<double, double> b);//向量乘
double calVectorSize(pair<double, double> a);//计算向量大小
double calAngle(pair<double, double> a, pair<double, double> b);//calcu vectors' angle
double calcuDis(pair<double, double> a, pair<double, double> b);//计算点之间的距离
double getRobotRadius(int robort_id);//获取机器人当前的半径
double calNextTimeDistance(double speed, double time, double  acceleration);//计算time时间后运动的距离
pair<double, double> getNextPos(int robot_id);


bool checkIsTrySeparate(int robotA_id, int robotB_id);//
bool checkRobortsCollison(int robotA_id, int robotB_id);//判断机器人a，b是否相撞
void solveRobortsCollison();//解决机器人相撞

void first_action();                                                 //The robot selects the point for the first time
void robot_action();                                                 //The robot selects the point for the second time and afterward
pair<int,double> pick_point(int robot_id, int state);                //Robot selection point
bool judge_full(int level, double threshold);                         //Set the load factor to determine whether the 4567 product is full
void robot_judge(int full);                                          //The robot makes buy and sell judgments based on the current state
bool can_stop(pair<double,double>p1,pair<double,double>p2,double angle);//能够停止转动
bool is_range(double dire,vector<double>&tmp);//判断角度是否在范围内123
pair<double,double> set_af(int robID);//给出机器人的速度和角度
bool can_speed_z(int stuID,pair<double,double>xy_pos,pair<double,double>pos,double acceleration);
double get_dis(pair<double, double> P, Line l) ;
bool isWall(int stuID);
bool will_impact(int robID,double dis=0.0);
double return_cal(pair<double,double>p1,pair<double,double>p2,double angle);
void init_studio_parameter();
bool isWall_r(int robID,double angle);