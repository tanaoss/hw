#pragma once
#include <iostream>
#include <vector>
#include <string>
using namespace std;

#define Pi 3.141592653589793

struct PayLoad
{
    double radius;
    double angle;                // 角度
    double angular_acceleration; // 角加速度
    double acceleration;         // 加速度
    double distance;
    double speed;
    int sign; // 当前角速度是在贴合目标点的夹角还是远离目标点的夹角
    PayLoad(double _radius, double _angle, double _angular_acceleration, double _acceleration, double _distance, double _speed, int _sign)
    {
        radius = _radius;
        angle = _angle;
        angular_acceleration = _angular_acceleration;
        acceleration = _acceleration;
        distance = _distance;
        speed = _speed;
        sign = _sign;
    }
    PayLoad(double _angle, double _angular_acceleration, double _acceleration, double _distance, int _sign)
    {
        angle = _angle;
        angular_acceleration = _angular_acceleration;
        acceleration = _acceleration;
        distance = _distance;
        sign = _sign;
    }
};

struct State
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
    Ins(int _robID = -1, double _forward = -1, double _rotate = -1, int _buy = -1, int _sell = -1, int _destroy = -1) : robID(_robID), forward(_forward), rotate(_rotate), buy(_buy), sell(_sell), destroy(_destroy)
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
    int loc_id;   // 所处工作台id
    int get_type; // 携带物品类型
    double time_val;
    double collision_val;
    double collision_val_pre;
    double angular_velocity;
    pair<double, double> xy_pos;
    double direction;
    pair<double, double> pos;
    int target_id; // 正在赶往的工作台；
    int lastSign;  //
    double lastRate;
    double isTurn;
    int pre_forWard;
    int pre_rote;
    int pre_cnt;
    int wait;
    bool operator!=(Robot s1){
    if(s1.id!=id||target_id!=s1.target_id||s1.loc_id!=loc_id||xy_pos!=s1.xy_pos||pos!=s1.pos){
            return true;
        }
        return false;
    }
    Robot(int _id, int _loc_id, int _get_type, double _time_val, double _collision_val, double _angular_velocity, pair<double, double> &_xy_pos,
          double _direction, pair<double, double> &_pos, int _target_id = -1,int _wait = -1,int _lastSign = 0, int _isTurn = 0) : xy_pos(_xy_pos), pos(_pos)
    {
        id = _id;
        loc_id = _loc_id;
        get_type = _get_type;
        time_val = _time_val;
        collision_val = _collision_val;
        angular_velocity = _angular_velocity;
        direction = _direction;
        target_id = _target_id;
        lastSign = _lastSign;
        isTurn = _isTurn;
        pre_cnt=0;
        wait=_wait;
    }
    void set(int _id, int _loc_id, int _get_type, double _time_val, double _collision_val, double _angular_velocity, pair<double, double> &&_xy_pos,
             double _direction, pair<double, double> &&_pos)
    {
        id = _id;
        loc_id = _loc_id;
        get_type = _get_type;
        time_val = _time_val;
        collision_val = _collision_val;
        angular_velocity = _angular_velocity;
        direction = _direction;
        xy_pos = _xy_pos;
        pos = _pos;
    }

    bool operator<(const Robot &a) const
    {
        // if (get_type == 0 && a.get_type != 0)
        //     return false;
        // if (a.get_type == 0 && get_type != 0)
        //     return true;
        // double speed2 = xy_pos.first * xy_pos.first;
        // double speed2_a = a.xy_pos.first * a.xy_pos.first;
        // if(speed2 - speed2_a > 1e-5)
        //     return false;
        // if(speed2_a - speed2 > 1e-5)
        //     return true;
        // if(fabs(angular_velocity) - fabs(a.angular_velocity) > 1e-10)
        //     return false;
        // if(fabs(a.angular_velocity) - fabs(angular_velocity) > 1e-10)
        //     return true;
        if (get_type == get_type){
            if(fabs(time_val * collision_val - a.time_val * a.collision_val) < 1e-10)
                return id < a.id;
            return time_val * collision_val - a.time_val * a.collision_val < 1e-10;
        }
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
    int wait_time; //等待时间
    Studio(int _id, int _type, int _r_id, pair<double, double> &_pos, int _r_time, int _bitSatus, int _pStatus) : id(_id), type(_type), r_id(_r_id), pos(_pos), r_time(_r_time), bitSatus(_bitSatus), pStatus(_pStatus)
    {
    }
    void set(int _id, int _type, pair<double, double> &&_pos, int _r_time, int _bitSatus, int _pStatus)
    {
        id = _id;
        type = _type;
        pos = _pos;
        r_time = _r_time;
        bitSatus = _bitSatus;
        pStatus = _pStatus;
    }
    bool operator!=(Studio s1){
        if(pos!=s1.pos||s1.id!=id||type!=s1.type||s1.r_id!=r_id||r_time!=s1.r_time||bitSatus!=s1.bitSatus||s1.pStatus!=pStatus){
            return true;
        }
        return false;
    }
};
struct Line { pair<double, double>  P; pair<double, double> v; };      // 直线（点向式）
bool eq(double a, double b);// ==
bool gt(double a, double b);// >
bool lt(double a, double b);// <
bool ge(double a, double b);// >=
bool le(double a, double b);// <=
void initrobotInfo();
bool readMapUntilOK();                 // 读地图
bool readStatusUntilOK();              // 读判题器输出
void out_put();                        // 输出指令
void calcuStudioDis();                 // 计算工作台之间的距离
void control(vector<PayLoad> payLoad); // 控制球体运行
void first_pick_point();
void robot_action();
void process();
PayLoad calPayload(int robotID,int studioID);                                                              // 计算机器人与目标之间的夹角、距离等信息
vector<double> get_T_limits(pair<double, double> pos, int id, int ctr = -1, double dis = 0.0); // 靠近墙体时，需要把方向转到那个范围才能加速
pair<double, double> subVector(pair<double, double> a, pair<double, double> b);                // 向量减（a-b）
double calVectorProduct(pair<double, double> a, pair<double, double> b);                       // 向量乘
double calVectorSize(pair<double, double> a);                                                  // 计算向量大小
pair<double, double> transformVector(double direction);
double calAngle(pair<double, double> a, pair<double, double> b);                               // calcu vectors' angle
double calcuDis(pair<double, double> a, pair<double, double> b);                               // 计算点之间的距离
double getRobotRadius(int robot_id);                                                          // 获取机器人当前的半径
double calNextTimeDistance(double speed, double time, double acceleration);                    // 计算time时间后运动的距离
pair<double, double> getNextPos(int robot_id);

bool isAcuteAngle(pair<double, double> a, pair<double, double> b);//判断是否为锐角
bool isAcuteAngle(pair<double, double> a, double x);

bool checkIsTrySeparate(int robotA_id, int robotB_id);   //
bool checkrobotsCollison(int robotA_id, int robotB_id, double k); // 判断机器人a，b是否相撞
void solveRobotsCollision();                             // 解决机器人相撞

void printRobotInfo(int i);

void first_action();                                                           // The robot selects the point for the first time
void robot_action();                                                           // The robot selects the point for the second time and afterward
pair<int, double> pick_point(int robot_id, int state);                         // Robot selection point
bool judge_full(int level, double threshold);                                  // Set the load factor to determine whether the 4567 product is full
void robot_judge(int full);                                                    // The robot makes buy and sell judgments based on the current state
bool can_stop(pair<double, double> p1, pair<double, double> p2, double angle); // 能够停止转动
bool is_range(double dire, vector<double> &tmp);                               // 判断角度是否在范围内123
pair<double, double> set_af(int robID);                                        // 给出机器人的速度和角度
bool can_speed_z(int stuID, pair<double, double> xy_pos, pair<double, double> pos, double acceleration);
double get_dis(pair<double, double> P, Line l);
bool isWall(int stuID);
bool will_impact(int robID, double dis = 0.0);
double return_cal(pair<double, double> p1, pair<double, double> p2, double angle);
void init_studio_parameter();
bool isWall_r(int robID,double angle);
int special_test(int robID1,int robID2);
double get_angle_1(double s1,double s2);
double get_angle_1(pair<double,double> p1,pair<double,double> p2);
double get_angle(double s1,double s2);
double get_angle(pair<double,double> p1,pair<double,double> p2);
bool is_less(int i1,int i2);
bool who_isFirst(int i1,int i2);
double return_v(int id);
int Calculate_root(int i1,int i2);
bool will_collision(int i1,int i2,int ctr=1);
bool will_collision_Careful(int i1,int i2);
bool return_collision(int i1,int i2);
pair<int,int> far_away(int i1,int i2,int base1,int base2);
double return_maxAng(int id1);
bool Check_for_balls_around(int i);
int return_line_dire(int i1,int i2,int signBase);
int return_line_dire(int i1,int i2);
pair<double,bool> return_int_dis(int base);
vector<int> return_int_pos(int base);
int return_int_neg(int base);
void Collision_detection(vector<PayLoad> payLoad);
bool check_material_full(int studio_id);
void Detect_codirection();
bool is_same_direction(int i1,int i2);
double get_rotation(int i1,int i2);
int addSign(int i1,int i2,int baseSign);
int getSign(int i1,int i2);
pair<double,double> return_seta(int i1,int i2);
double return_type(int i1);
void change_getType();
bool can_Pass(int i1,double seta,double arf, double canAngle,
double dis,double time,int ctr);
double get_at_v(double t,double a,double v,int sign_v1);
double get_at_v_canSet(double t,double a,double v,double v1,int sign_v1);
double get_at_v_z(double t,double a,double v,int sign_v1);
bool is_near_tar(int id);
double anger_to_length(int robot_id,int studio_id);
vector<pair<double,double>>Calculate_the_trajectory(Robot rob,int cnt,int tar);
PayLoad calPayload_trajectory(Robot rob,int studioID);
vector<pair<double,double>>Calculate_the_trajectory(Robot rob,Ins ins,vector<pair<double,double>> tra,int cnt,int tar);
double get_at_stop(double t,double a,double v,int sign_v1);
Ins contr_one_rob(Robot robot ,PayLoad payload);
vector<double>  get_T_limits(pair<double,double>pos,Robot robot);
double get_at_stop_a(double t,double x,double v,int sign_v1);
double return_time_root_v(double a,double b,double c,double v,double a1);
double get_at_v_limt(double t,double a,double v,double v1,int sign_v1);
double return_ac(double a,double v1,double v2);