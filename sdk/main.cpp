#include <iostream>
#include <vector>
#include<string>
#include <cmath>
using namespace std;
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
    friend ostream &operator<<(ostream &os, Ins ins)
    {
        cout << "forward " << ins.robID << " " << ins.forward << "\n"
             << "rotate " << ins.robID << " " << ins.rotate << endl;
        if (ins.buy != -1)
        {
            cout << "buy " << ins.robID << endl;
        }
        if (ins.sell != -1)
        {
            cout << "sell " << ins.robID << endl;
        }
        if (ins.destroy != -1)
        {
            cout << "destroy " << ins.robID << endl;
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
vector<vector<double>> dis(50, vector<double>(50, 0));
vector<Studio> studios;
vector<Robot> robots;
State state;//当前帧数，全局可见
void init();
void getNums(string& line,vector<double>&tmp){
        int cnt=0;
        int flag=0;//标识是整数还是小数部分
        int iBase=10;//乘基
        double fBase=0.1;
        int iNum=0;
        double fNum=0.0;//小数部分
        int sign=1;//符号 
        for(auto c:line){
            if(c>='0'&&c<='9'){
                if(flag==0){
                    iNum=iNum*10+int(c-'0');
                }else{
                    fNum+=double(c-'0')*fBase;
                    fBase*=fBase;
                }
            }else if(c=='.'){
                flag=1;
            }else if(c==' '){
                tmp[cnt]=sign*(iNum+fNum);
                cnt++;
                flag=0;//标识是整数还是小数部分
                iNum=0;
                fNum=0.0;//小数部分
                fBase=0.1;
                sign=1;//符号 
            }else if(c=='-'){
                sign=-1;
            }else if(c=='+'){
                sign=1;
            }
        }
        tmp[cnt]=sign*(iNum+fNum);
}
bool readMapUntilOK() {
    char line[1024];
    int count = 0;
    int count_robot = 0, count_studio = 0;
    double x,y;
    int i;
    while (cin.getline(line,sizeof(line))) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
        for(i=0;i<100;i++){
            if(line[i] == 'A'){
                x = i*0.5+0.25;
                y = (100-count)*0.5-0.25;
                pair<double,double>pos_robot(x,y);
                pair<double,double>xy_pos_robot(0,0);
                Robot  robot(count_robot,0,0,0,1,1,xy_pos_robot,0,pos_robot,0);
                robots.push_back(robot);
                count_robot++;
            }
            else if(line[i] >= '1' && line[i] <= '9'){
                x = i*0.5+0.25;
                y = (100-count)*0.5-0.25;
                pair<double,double>pos_studio(x,y);
                Studio studio(count_studio,0,0,pos_studio,0,0,0);
                studio.type = (int)line[i]-48;
                studios.push_back(studio);
                count_studio++;
            }
        }
	    count++;
    }
    return false;
}
bool readStatusUntilOK() {
    string line;
    cin>>state.FrameID>>state.money;
    int K;
    int studio_id=0;
    int rob_id=0;
    cin>>K;
    while (K--)
    {
        cin>>line;
        vector<double> tmp(6,0);
        getNums(line,tmp);
        studios[studio_id].set(studio_id,tmp[0],pair<double,double>(tmp[1],tmp[2]),tmp[3],tmp[4],tmp[5]);
        studio_id++;
    }
    for(int i=0;i<4;i++){
        cin>>line;
        vector<double> tmp(10,0);
        getNums(line,tmp);
        robots[rob_id].set(rob_id,tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],pair<double,double>(tmp[5],tmp[6]),tmp[7],
        pair<double,double>(tmp[8],tmp[9]));
        rob_id++;
    }
    return false;
}
void out_put(vector<Ins>&out){
    for(auto ins:out){
        cout<<ins;
    }
    cout.flush();
}
double calcuDis(pair<double, double> a, pair<double, double> b)
{
    return sqrt((a.first - b.first) * (a.first - b.first) + (a.second - b.second) * (a.second - b.second));
}

void calcuStudioDis()
{
    int num = studios.size();
    int i, j;
    for (i = 0; i < num; i++)
    {
        for (j = 0; j < i; j++)
        {
            dis[j][i] = dis[i][j] = calcuDis(studios[i].pos, studios[j].pos);
        }
    }
}

int main()
{
    std::ios::sync_with_stdio(false);   
    std::cin.tie(0);    // IO
    return 0;
}
