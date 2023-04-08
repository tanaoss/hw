#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <map>
#include <set>
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
    PayLoad()
    {}
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
    int target_id_pre;//
    int last_target_id;
    int target_id_send;
    int target_id_buy;
    int virtual_id;
    int lastSign;  //
    double lastRate;
    double isTurn;
    int pre_forWard;
    int pre_rote;
    int pre_cnt;
    int wait;
    int node_id;
    int close_node;
    int real_get_type;
    int last_get_type;
    bool need_rote_wall;
    double radius;
    int pane_id;
    int robot_slow_v_cnt;
    pair<double,double> virtual_pos;
    int robot_area_type[2];
    int cnt_tar;//标定的路径tar
    bool isVir;
    bool need_adjust_statues;
    bool adjust_w;
    bool adjust_pos;
    bool need_slow;
    bool is_illegal;
    bool is_dangerous;
    bool is_new_tar_ing;
    bool need_collison;
    bool operator!=(Robot s1){
    if(s1.id!=id||target_id!=s1.target_id||s1.loc_id!=loc_id||xy_pos!=s1.xy_pos||pos!=s1.pos){
            return true;
        }
        return false;
    }
    Robot(int _id, int _loc_id, int _get_type, double _time_val, double _collision_val, double _angular_velocity, pair<double, double> &_xy_pos,
          double _direction, pair<double, double> &_pos, int _target_id = -1, int _node_id = 0,int _wait = -1,int _lastSign = 0, int _isTurn = 0, double _radius = 0.45) : xy_pos(_xy_pos), pos(_pos)
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
        last_get_type = 0;
        wait=_wait;
        need_rote_wall=false;
        radius = _radius;
        node_id = _node_id;
        isVir=false;
        need_adjust_statues=true;
        adjust_w=false;
        adjust_pos=false;
        cnt_tar=0;
        target_id_pre=-1;
        need_slow=false;
        is_illegal=false;
        is_dangerous=false;
        is_new_tar_ing=false;
        robot_slow_v_cnt=0;
        need_collison=false;
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
        if (get_type == a.get_type){
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
    int studio_area_type[2];
    int node_id;
    int pane_id;
    int has_suspicious_spots;
    vector<int> material_studios[8];
    vector<int> suspicious_spots;
    bool corner;
    Studio(int _id, int _type, int _r_id, pair<double, double> &_pos, int _r_time, int _bitSatus, int _pStatus, int _node_id) : id(_id), type(_type), r_id(_r_id), pos(_pos), r_time(_r_time), bitSatus(_bitSatus), pStatus(_pStatus), node_id(_node_id)
    {
        corner=false;
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

struct pane
{
    int id;                   // 空格id
    pair<double, double> pos; // 空格中心点坐标
    int type;                 // 空格类型
};
struct type_area
{
    int type;
    int height;
    map<int,pair<double,double>> entrance;
};
struct Graph_node{
    int id;//i*100+j
    int pre_id;//最短路中的前置pre_id;
    double dis;
    double angle_sum;
    int dangerous_sum;
    Graph_node(int _id,double _dis,int _pre_id){
        id=_id;
        dis=_dis;
        pre_id=_pre_id;
        angle_sum = 0;
    }

    Graph_node(int _id,double _dis,int _pre_id, double _angle_sum){
        id=_id;
        dis=_dis;
        pre_id=_pre_id;
        angle_sum = _angle_sum;
    }

    Graph_node(int _id,double _dis,int _pre_id, double _angle_sum, int _dangerous_sum){
        id=_id;
        dis=_dis;
        pre_id=_pre_id;
        dangerous_sum = _dangerous_sum;
        angle_sum = _angle_sum;
    }
    
};//转换图节点

struct cmp_Graph_node
{
    bool operator()(const Graph_node &a,const Graph_node &b)
    {
        // if(a.dangerous_sum != b.dangerous_sum)
        //     return a.dangerous_sum > b.dangerous_sum;
        if(fabs(a.dis - b.dis) < 1e-7) {
            return a.angle_sum > b.angle_sum;
        }
        return a.dis > b.dis;
    }
};

void mock_fram_skip();

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

void control(); // 控制球体运行
void first_pick_point();
void robot_action();
void process();
PayLoad calPayload(Robot robot, pair<double, double> virtual_pos);                               // 计算机器人与目标之间的夹角、距离等信息
PayLoad calPayload_back(Robot robot, pair<double, double> virtual_pos);                               // 计算机器人后退与目标之间的夹角、距离等信息
vector<double> get_T_limits(pair<double, double> pos,const Robot& robot, int ctr = -1, double dis = 0.0); // 靠近墙体时，需要把方向转到那个范围才能加速
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
bool can_stop(pair<double, double> p1, pair<double, double> p2, double angle,bool isWall,int ctr=1); // 能够停止转动
bool is_range(double dire, vector<double> &tmp);                               // 判断角度是否在范围内123
pair<double, double> set_af(int robID);                                        // 给出机器人的速度和角度
bool can_speed_z(int stuID, pair<double, double> xy_pos, pair<double, double> pos, double acceleration);
double get_dis(pair<double, double> P, Line l);
bool isWall(int stuID);
bool will_impact(const Robot& robot, double dis = 0.0);
double return_cal(pair<double, double> p1, pair<double, double> p2, double angle);
void init_studio_parameter();
bool isWall_r(const Robot& robot,double angle);
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
bool check_material_full(int studio_id);
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
vector<pair<double, double>> Calculate_the_trajectory(Robot rob, int cnt, int tar,  int ctrF=1);
PayLoad calPayload_trajectory(Robot rob,int studioID);
vector<pair<double,double>>Calculate_the_trajectory(Robot rob,Ins ins, int forward_change, int rotate_change,const vector<pair<double,double>>& tra,int cnt,int tar,double rob_dis,double pre_dis=100);
double get_at_stop(double t,double a,double v,int sign_v1);
Ins contr_one_rob(Robot& robot);
vector<double>  get_T_limits(Robot& rob);
double get_at_stop_a(double t,double x,double v,int sign_v1);
double return_time_root_v(double a,double b,double c,double v,double a1);
double get_at_v_limt(double t,double a,double v,double v1,int sign_v1);
double return_ac(double a,double v1,double v2);
void updateGetType();
void updateIns(int id, int i);
void collision_solve(int frame);
int checkNoCollision(const vector<pair<double,double>> &a, const vector<pair<double,double>> &b, double mindis);
void solveNoSolution(int x, int y);
int getTimeToStudio(int id, const vector<pair<double,double>> &a);
void printPair(pair<double,double> a);
void printRobotsDis(int i, int j);
void printRobotsDis(Robot ro, pair<double,double> a);
void printPredictRobotsDis(const vector<pair<double,double>> &a, const vector<pair<double,double>> &b);
pair<double,double> distance(int  robot_id,int studio_id);
double get_at_stop_test(double t,double a,double v,int sign_v1);
pair<double ,double> return_change_v(double w,double changeSeta,pair<double,double>v);
double will_Collo_new(int i1,int i2);
void adjust_collo_new(int i1,int i2,int baseSign);
bool check_wall_r(int i);
bool  isWall_r(int id);
double get_Angle_xy(Robot& rob);
double Calculate_the_projection_speed(Robot& rob);
void cal_matrix(vector<vector<double>>&c,double angle1_w,double angle2);
pair<double,bool> get_w_now(const Robot& robot, const PayLoad& payload);
double get_v_now(const Robot& robot, const PayLoad& payload);


bool checkNearBar(const pair<double,double> &a, double radius);
void floyd();
void print_queue();
void divide_space(int is_take);
void deal_graph();
void floyd_area(int is_take);
void studio_distance(int is_take);
void analyze_space(int is_take);

void print_target(int i, int j);
void init_trans();//将原来的地图中不是-2的部分全部更改为0
void Translation_graph_no();//转换机器人不带物品的原始图
void Translation_graph_has();//转换机器人带物品的原始图
double Angle_conversion(double angle);//将角度转换为距离
void Dijkstra(int s, int is_take);//对源点s做最短路，s为node_id
bool check_4(int i,int j);//检查坐标i,j是否是一个四个格子的合法点
pair<int,pair<double,double>> check_8(int i,int j);//检查坐标i,j是否是一个四个格子的合法点
void getEdgeRalative();//得到边关系
void trans_studio_rob_toID();//建立工作台和机器人id与编号的关系；
bool is_corner(int id);//判断工作台是不是在墙角


double calAngleToDis(int x, int y, int z);//nodeID转角度转距离
void init_data();
void printMap(int f);
void printEdge(int id);
bool check_slope(int id1, int id2);
void printPath(int from_id, int is_robot, int to_id, int is_take);
void print_dijkstra(int studio_id, int is_take, int is_path);
int trans_pos_to_nodeID(pair<double, double> pos);
int trans_pos_to_nodeID(int robot_id);
pair<double, double> trans_nodeID_to_pos(int nodeID);
void init_vector();
bool is_connected(int node_id_a, int node_id_b);
int get_bar_num(int node_id_a, int node_id_b);
void init_bar_sum();
PayLoad calPayload_back(Robot robot, pair<double, double> virtual_pos);
int choose_best_to(Robot &ro, pair<double, double> pos);
double get_dis(const Robot &ro1, const Robot &ro2);
int choose_close_node(int is_take, pair<double, double> pos);
bool do_back(int id, int id1, pair<double, double> pos);
bool check_speed(Robot ro_a, Robot ro_b, double mindis);
bool check_node_illegal(int x, int y);
bool check_nead_slow_down(const Robot &ro, const Robot &ro_static, double mindis, int coll_frame);
double get_rotation_stop_time(const Robot &ro, PayLoad pay);
double get_stop_time(double x, double v0, int sign, double acceleration);
int get_avoid_node(const Robot &ro_back, const Robot &ro_go, double mindis);
bool check_node_safe(int node_id, int is_take, double mindis, const Robot &ro);


bool empty_pos(const Robot& rob);


vector<int> get_future_node(int robot_id);
bool is_need_slow(Robot& robot,pair<double,double> pos,pair<double,double> pos1);
void adjust_virtual_pos(Robot& robot);
void adjust_virtual_pos_total(Robot& robot);
bool check_can_arrival(int istake,int id1,int id2,bool ctr);
set<int> getEqID(int istake,int id1);
bool checkEnough(int robot_id, int target_id, int frame);
void new_robot_action();
void new_first_action();
void setVirPos(Robot& robot);
pair<double,double>select_visPos(Robot& robot,vector<int> range,int tar3);
int ret_next(Robot& robot,int tar_cnt);
bool at_least_three(Robot& robot,int tar_cnt);
bool calMinAngle(Robot& robot,pair<double,double>pos);
double vir_v_1(Robot rob,int v_limit);
bool can_trajectory_virpos_0(Robot rob,double v,int cnt);
double vir_v_0(Robot rob);
bool can_trajectory_virpos(Robot rob,double v,int cnt);
int getPosID(pair<double,double>pos);
pair<double,bool> get_vir_w(Robot& rob,PayLoad& payload);
void init_rob_status(Robot& rob);
bool check_can_arrival_z(int id1,int id2);
bool check_corner_collosion(Robot& rob);
bool has_next(Robot& rob);
bool check_tar_line(Robot& rob,double dis);
Ins contr_one_rob_0(Robot& robot);
Ins contr_one_rob_1(Robot& robot);
Ins contr_new_tar(Robot& robot);
void get_point_type() ;
void check_robot_pos_status(Robot& robot);
void adjust_illegal_pos(Robot& robot);
bool check_will_collison_wall(Robot& robot);
void  select_the_standard_id(Robot& robot);
void check_suspicious_spots(int studio_id);
bool  need_to_step_back(const Robot& rob);
int get_best_pos(Robot& rob);
bool check_slope_studios(int id1,int id2);
bool check_barrier(pair<double,double> start,pair<double,double> end,int carry);