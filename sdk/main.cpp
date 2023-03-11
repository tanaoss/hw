#include <iostream>
#include<vector>
using namespace std;
struct  Ins
{
    int robID;
    double forward;
    double rotate;
    int  buy;
    int sell;
    int destroy;
    Ins(int _robID,double _forward, double _rotate,int  _buy=-1,int _sell=-1,int _destroy=1):
    robID(_robID),forward(_forward),rotate(_rotate),buy(_buy),sell(_sell),destroy(_destroy){

    }
    friend ostream& operator<<(ostream& os,Ins ins){
        cout<<"forward "<<ins.robID<<" "<<ins.forward<<"\n"
        <<"rotate "<<ins.robID<<" "<<ins.rotate<<endl;
        if(ins.buy!=-1){
            cout<<"buy "<<ins.robID<<endl;
        }
        if(ins.sell!=-1){
            cout<<"sell "<<ins.robID<<endl;
        }
        if(ins.destroy!=-1){
            cout<<"destroy "<<ins.robID<<endl;
        }
     return os;   
    }
};

struct Robot
{
    int id;
    int get_type;//携带物品类型
    double time_val;
    double collision_val;
    double angular_velocity;
    pair<double,double>xy_pos;
    double direction;
    pair<double,double>pos;
    int target_id;//正在赶往的工作台；
    Robot(int _id,int _get_type,double _time_val,double _collision_val,double _angular_velocity,pair<double,double>& _xy_pos,
    double _direction,pair<double,double>&_pos,int _target_id=0):
    xy_pos(_xy_pos),pos(_pos)
    {
        id=_id;
        get_type=_get_type;
        time_val=_time_val;
        collision_val=_collision_val;
        angular_velocity=_angular_velocity;
        direction=_direction;
        target_id=_target_id;
    }
    void set(int _id,int _get_type,double _time_val,double _collision_val,double _angular_velocity,pair<double,double>& _xy_pos,
    double _direction,pair<double,double>&_pos,int _target_id=0)
    {
        id=_id;
        get_type=_get_type;
        time_val=_time_val;
        collision_val=_collision_val;
        angular_velocity=_angular_velocity;
        direction=_direction;
        target_id=_target_id;
        xy_pos=_xy_pos;
        pos=_pos;
    }
};

struct  Studio
{
    int id;
    int r_id;//正在赶来的机器人id
    pair<double,double>pos;
    int r_time;//剩余生产时间（帧数）
    int bitSatus;//原材料格状态
    int pStatus;//产品格状态 
    Studio(int _id,int _r_id, pair<double,double>&_pos,int _r_time,int _bitSatus,int _pStatus):
    id(_id),r_id(_r_id),pos(_pos),r_time(_r_time),bitSatus(_bitSatus),pStatus(_pStatus)
    {

    }
    void set(int _id,int _r_id, pair<double,double>&_pos,int _r_time,int _bitSatus,int _pStatus){
        id=_id;
        r_id=_r_id;
        pos=_pos;
        r_time=_r_time;
        bitSatus=_bitSatus;
        pStatus=_pStatus;
    }
};
vector<vector<double>>dis(50,vector<double>(50,0));
vector<Studio> studios;
vector<Robot> robots;
void init();


int main() {
    Ins ins(0,0,0,0,0,0);
    cout<<ins;
    return 0;
}
