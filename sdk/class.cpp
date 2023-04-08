#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <algorithm>
#include <cstring>
#include <queue>
#include<unordered_map>
#include<iomanip>
#include<set>
#include"vec.h"
#include "class.h"

// #include "line.h"

using namespace std;
vector<Studio> studios;
vector<Robot> robots;
State state;//当前帧数，全局可见
vector<Ins> ins(4);
vector<int> material[4][8];
vector<int> product[8];
vector<int> full_product;
vector<int> studios_type[10];
vector<PayLoad> payloads;
vector<pane> panes;
int class_map;
int price[8][2];
double EPS=1e-7;
double acceleration_no;
double acceleration_has;
double angular_acceleration_no;
double angular_acceleration_has;
double linear_velocity[2];
vector<bool> need_stop(4,false);
int robot_get_type[8];
int last_solution[4][4];
int studios_rid[50][8];
int studio_material[6][8];
int studio_level[5][2];
int material_send[8][3];
int RootFlag=-2;
int robot_last_last_state[4][2];
int robot_last_state[4][2];
int Flag_sumulate=0;
int last_count[50];
int lack_material[8];
int produce_product[8];
int priority[8]; 
int max_wait_time[4];
int robot_area[4];
int contr_print_flag=0;
int graph[100][100];
int sum_matrix[2][100][100];
int product_time[8];
// int graphs[2][100][100];
// int target_sequence[2][500][500];
int wail[101][101];
// double dis_area[2][500][500];
// double studio_dis[2][50][50];
// double init_robot_dis[4][50];
double new_cllo_time = 0;
int cerr_flag_j=0;
int start_time=-1;
int end_time =3;
pair<double ,double> Root;
pair<double ,double> Collision_point;

vector<type_area>types[2];
vector<pair<double,double>>arri_Set;
double Compute_redundancy=0;
Ins ins_set[8];
unordered_map<int,vector<Graph_node>> graph_edge[2];//点id的边集
unordered_map<int,vector<Graph_node>> studio_edge[2];//studios edge
unordered_map<int,pair<double,double>> exist_id[2];//确定存在的id，便于建立边关系
unordered_map<int,int> exist_id_type[2];//确定存在的点的关系
unordered_map<int,int> stu_transID;//建立工作台id与转换后id的关系

unordered_map<int,bool> can_arrival[2];//判断任意两个小格之间是否可以直达;
unordered_map<int,bool> illegal_point[2];//判断是否是非法点
unordered_map<int,bool> dangerous_point[2];//判断是否是危险点
unordered_map<int,int> dangerous_nums[2];//判断是否是危险点
int graph_trans[100][100];
bool print_cerr_flag_ta;
vector<int> next_node[50][2];//next_node[studio_id][2][node_id]:node_id去往studio_id工作台的下一个点
vector<double> dis_to_studios[50][2];//dis_studios[studio_id][2][node_id]:node_id去往studio_id工作台的距离
bool collision_cerr_flag = false;

int bar_sum[100][100];


void initrobotInfo() {

    
    double weightMin = 0.45 * 0.45 * Pi * 20;
    double weightMax = 0.53 * 0.53 * Pi * 20;
    double inertiaMin = weightMin * 0.45 * 0.45 *0.5;
    double inertiaMax = weightMax * 0.53 * 0.53 *0.5;

    acceleration_no = 250/ weightMin;
    acceleration_has = 250 / weightMax;

    angular_acceleration_no = 50 / inertiaMin;
    angular_acceleration_has = 50 /inertiaMax;

    linear_velocity[0] = acceleration_no * 0.02;
    linear_velocity[1] = acceleration_has * 0.02;

    memset(last_solution, -1, sizeof(last_solution));


    for(int i = 0; i < 4; ++i) {
        if(i % 3 == 0) ins_set[i].rotate = 0;
        else if(i % 3 == 1) ins_set[i].rotate = Pi;
        else ins_set[i].rotate = -Pi;
    }
    ins_set[3].forward = 0;
    ins_set[4].forward = -2;


    // int tar1 = 1;
    // int tar2 = 7;
    // int is_take1 = 0;
    // int is_take2 = 1;
    // int node1 = choose_close_node(1, make_pair(28.27 , 21.19));
    // int node2 = choose_close_node(1, make_pair(28.53, 22.73));
    // cerr << "nodeid: " << choose_close_node(1, make_pair(28.53, 22.73)) << "\n";
    // cerr << dis_to_studios[tar1][is_take1][node1] << "* x:" << dis_to_studios[tar2][is_take2][node2] << "\n";
    // if (gt(dis_to_studios[tar2][is_take2][node2] - dis_to_studios[tar1][is_take1][node1], 20))
    // {
    //     cerr << "change choose x\n";
    // }
}
void init_studio_parameter(){
    int studio_id;
    for(int i=0;i<50;i++){
        last_count[i]=0;
        for(int j=0;j<8;j++)studios_rid[i][j]=-1;
    }
    for(int i=0;i<studios.size();i++){
        studios_type[studios[i].type].push_back(i);
    }

    studio_material[0][0]=2;
    studio_material[0][1]=1;
    studio_material[0][2]=2;
    studio_material[1][0]=2;
    studio_material[1][1]=1;
    studio_material[1][2]=3;
    studio_material[2][0]=2;
    studio_material[2][1]=2;
    studio_material[2][2]=3;
    studio_material[3][0]=3;
    studio_material[3][1]=4;
    studio_material[3][2]=5;
    studio_material[3][3]=6;
    studio_material[4][0]=1;
    studio_material[4][1]=7;
    studio_material[5][0]=7;
    studio_material[5][1]=1;
    studio_material[5][2]=2;
    studio_material[5][3]=3;
    studio_material[5][4]=4;
    studio_material[5][5]=5;
    studio_material[5][6]=6;
    studio_material[5][7]=7;
    studio_level[2][0] = 1;
    studio_level[2][1] = 3;
    studio_level[3][0] = 4;
    studio_level[3][1] = 6;
    studio_level[4][0] = 7;
    studio_level[4][1] = 7;
    material_send[1][0] = 2;
    material_send[1][1] = 4;
    material_send[1][2] = 5;
    material_send[2][0] = 2;
    material_send[2][1] = 4;
    material_send[2][2] = 6;
    material_send[3][0] = 2;
    material_send[3][1] = 5;
    material_send[3][2] = 6;
    material_send[4][0] = 1;
    material_send[4][1] = 7;
    material_send[5][0] = 1;
    material_send[5][1] = 7;
    material_send[6][0] = 1;
    material_send[6][1] = 7;
    material_send[7][0] = 1;
    material_send[7][1] = 8;
    price[1][0]=3000;
    price[1][1]=6000;
    price[2][0]=4400;
    price[2][1]=7600;
    price[3][0]=5800;
    price[3][1]=9200;
    price[4][0]=15400;
    price[4][1]=22500;
    price[5][0]=17200;
    price[5][1]=25000;
    price[6][0]=19200;
    price[6][1]=27500;
    price[7][0]=76000;
    price[7][1]=105000;
    product_time[1] = 50;
    product_time[2] = 50;
    product_time[3] = 50;
    product_time[4] = 500;
    product_time[5] = 500;
    product_time[6] = 500;
    product_time[7] = 1000;
    for(int i=0;i<studios.size();i++){
        if(studios[i].type>3){
            // cerr<<"studio : "<<i<<"type = "<<studios[i].type<<"\n";
            for(int j = 1;j<=studio_material[studios[i].type-4][0];j++){
                // cerr<<" material type = "<<studio_material[studios[i].type-4][j]<<"\n";
                for(int k = 0;k<studios_type[studio_material[studios[i].type-4][j]].size();k++){
                    studio_id = studios_type[studio_material[studios[i].type-4][j]][k];
                    // cerr<<" studio_id "<<studio_id<<" dist = "<<dis_to_studios[studio_id][1][studios[i].node_id]<<"\n";
                    if(!(eq(dis_to_studios[studio_id][1][studios[i].node_id],10000))){
                        // cerr<<"j = "<<j<<"\n";
                        studios[i].material_studios[j-1].push_back(studio_id);
                        // cerr<<' '<<studio_id<<' ';
                    }
                }
                // cerr<<"\n";
            }
        }
        check_suspicious_spots(i);
        if(studios[i].has_suspicious_spots == 1){
            int node_id = studios[i].suspicious_spots[0];
            int suspicious_spots_x =  node_id/100;
            int suspicious_spots_y = node_id%100;
            int studio_x = studios[i].node_id/100;
            int studio_y = studios[i].node_id%100;
            double x,y;
            if(eq((suspicious_spots_x-studio_x),0)||eq((suspicious_spots_y-studio_y),0)){
                if(eq((suspicious_spots_x-studio_x),0)){
                    x = studios[i].pos.first;
                    if(lt(suspicious_spots_y,studio_y)){
                        y = studios[i].pos.second-0.35;
                    }
                    else{
                        y = studios[i].pos.second+0.35;
                    }
                }
                else{
                    y = studios[i].pos.second;
                    if(lt(suspicious_spots_x,studio_x)){
                        x = studios[i].pos.first-0.35;
                    }
                    else{
                        x = studios[i].pos.first+0.35;
                    }
                }
            }
            else{
                if(lt(suspicious_spots_x,studio_x)&&lt(suspicious_spots_y,studio_y)){
                    x = studios[i].pos.first-pow(0.35,0.5);
                    y = studios[i].pos.second - pow(0.35,0.5);
                }
                else if(gt(suspicious_spots_x,studio_x)&&lt(suspicious_spots_y,studio_y)){
                    x = studios[i].pos.first+pow(0.35,0.5);
                    y = studios[i].pos.second - pow(0.35,0.5);
                }
                else if(lt(suspicious_spots_x,studio_x)&&gt(suspicious_spots_y,studio_y)){
                    x = studios[i].pos.first-pow(0.35,0.5);
                    y = studios[i].pos.second + pow(0.35,0.5);
                    
                }
                else{
                    x = studios[i].pos.first+pow(0.35,0.5);
                    y = studios[i].pos.second + pow(0.35,0.5);
                }
            }
        }
    }

}




bool readMapUntilOK() {
    char line[1024];
    int count = 0;
    int count_robot = 0, count_studio = 0;
    double x,y;
    int i;
    int row = 0;
    int num = 0;
    for(int k = 0;k<101;k++){
        for(int j = 0; j < 101; j++){
            wail[k][j] = 0;
        }
    }
    while (cin.getline(line,sizeof(line))) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
        for(i=0;i<100;i++){
            pane train;
            train.id = num;
            train.pos.first = i * 0.5 + 0.25;
            train.pos.second = (100 - count) * 0.5 - 0.25;
            train.type = -1;
            if(line[i] == 'A'){
                x = i*0.5+0.25;
                y = (100-count)*0.5-0.25;
                pair<double,double>pos_robot(x,y);
                pair<double,double>xy_pos_robot(0,0);
                // cout<<x<<" "<<y<<"\n";
                Robot  robot(count_robot,0,0,0,1,1,xy_pos_robot,0,pos_robot,-1, (99-row)*100+i);
                robot.pane_id = train.id;
                robots.push_back(robot);
                count_robot++;
            }
            else if(line[i] >= '1' && line[i] <= '9'){
                x = i*0.5+0.25;
                y = (100-count)*0.5-0.25;
                pair<double,double>pos_studio(x,y);
                // cout<<x<<" "<<y<<"\n";
                Studio studio(count_studio,0,-1,pos_studio,0,0,0, (99-row)*100+i);
                stu_transID[(99-row)*100+i] = count_studio;
                studio.type = (int)line[i]-48;
                studio.pane_id = train.id;
                studios.push_back(studio);
                count_studio++;
            }
            if (line[i] == '#')
            {
                graph[99-row][i] = -2;
                train.type = -2;
                wail[99-row][i] = -2;
                wail[99-row][i+1] = -2;
                wail[99-(row+1)][i] = -2;
                wail[99-(row+1)][i+1] = -2;
            }
            else
                graph[99-row][i] = 0;
            panes.push_back(train);
            num++;
        }
        row++;
	    count++;
    }
    return false;
}

void mock_fram_skip() {
    string line;
    cin>>state.money;
    cin.ignore();
    int K;
    int studio_id=0;
    int rob_id=0;
    cin>>K;
    cin.ignore();
    for(int i=0;i<4;i++){
        for(int j=0;j<8;j++){
            material[i][j].clear();
        }
    }
    for(int i=0;i<8;++i) {
        product[i].clear();
        produce_product[i]=0;
        lack_material[i]=0;
        priority[i]=0;
    }
    for(int i=0;i<4;i++){
        max_wait_time[i]=0;
    }
    while (K--)
    {
        vector<double> tmp(6,0);
        for(int i=0;i<tmp.size();i++){
            cin>>tmp[i];
        }
        studios[studio_id].set(studio_id,tmp[0],pair<double,double>(tmp[1],tmp[2]),tmp[3],tmp[4],tmp[5]);
        if(studios[studio_id].pStatus == 1 ){
            product[studios[studio_id].type].push_back(studio_id);
            if (studios[studio_id].type >= 4 && studios[studio_id].type<=6){
                produce_product[studios[studio_id].type]++;
            }
        }
        if (studios[studio_id].type >= 4 && studios[studio_id].type<=7){
            
            if (studios[studio_id].bitSatus != 0 ){
                for(int i = 1; i <= studio_material[studios[studio_id].type-4][0]; i++){
                    if ((studios[studio_id].bitSatus & (int)pow(2, studio_material[studios[studio_id].type-4][i]))==0){
                        lack_material[studio_material[studios[studio_id].type - 4][i]]++;
                    }
                }
            }
        }
        if (studios[studio_id].type > 3)
        {
            if(studios[studio_id].type < 8){
                
                    for(int i = 0;i < 4;i++){
                        if(studios[studio_id].type == i+4){
                            for(int j = 0;j<studio_material[i][0];j++){
                                if((studios[studio_id].bitSatus & (int)pow(2,studio_material[i][j+1])) == 0){
                                    // if(studios_rid[studio_id][studio_material[i][j+1]] == -1)material[studio_material[i][j+1]].push_back(studio_id);
                                    for(int k = 0;k<4;k++){
                                        if(!eq(dis_to_studios[studio_id][0][robots[k].node_id],10000))
                                            material[k][studio_material[i][j+1]].push_back(studio_id);
                                    }
                                }
                            }
                        }
                    }
            
            }
            if(studios[studio_id].type == 8){
                for(int k = 0;k<4;k++){
                    if(!eq(dis_to_studios[studio_id][0][robots[k].node_id],10000))
                        material[k][7].push_back(studio_id);
                }
                                    
                // material[7].push_back(studio_id);
            }
            if(studios[studio_id].type == 9){
                for(int h = 1;h <=7;h++){
                    for(int k = 0;k<4;k++){
                        if(!eq(dis_to_studios[studio_id][0][robots[k].node_id],10000))
                            material[k][h].push_back(studio_id);
                    }
                    // material[h].push_back(studio_id);
                }
            }
        }
        studio_id++;
    }
    for(int i=1;i<7;i++){
        if(produce_product[i]<lack_material[i]){
            priority[i] = lack_material[i]-produce_product[i];
        }
    }
    for(int i=0;i<4;i++){
        vector<double> tmp(10,0);
        for(int i=0;i<tmp.size();i++){
            cin>>tmp[i];
        }
        robots[rob_id].collision_val_pre=robots[rob_id].collision_val;
        robots[rob_id].set(rob_id,tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],pair<double,double>(tmp[5],tmp[6]),tmp[7],
        pair<double,double>(tmp[8],tmp[9]));
        robots[rob_id].node_id = trans_pos_to_nodeID(rob_id);
        robots[rob_id].close_node = choose_close_node(robots[rob_id].get_type!=0, robots[rob_id].pos);
        robots[rob_id].radius = (robots[rob_id].get_type == 0? 0.45: 0.53);
        

        // if(gt(robots[rob_id].collision_val_pre, robots[rob_id].collision_val) && robots[rob_id].get_type != 0)
        //     cerr<<"time-collision:"<< state.FrameID <<"collision" <<rob_id<< endl<<"\n";
        rob_id++;
    }
    cin>>line;
    if (line[0] == 'O' && line[1] == 'K') {
        cout<<state.FrameID<<endl;
        out_put();
    }
}

bool readStatusUntilOK() {
    string line;
    cin>>state.money;
    cin.ignore();
    int K;
    int studio_id=0;
    int rob_id=0;
    cin>>K;
    cin.ignore();
    for(int i=0;i<4;i++){
        for(int j=0;j<8;j++){
            material[i][j].clear();
        }
    }
    for(int i=0;i<8;++i) {
        product[i].clear();
        produce_product[i]=0;
        lack_material[i]=0;
        priority[i]=0;
    }
    for(int i=0;i<4;i++){
        max_wait_time[i]=0;
    }
    while (K--)
    {
        vector<double> tmp(6,0);
        for(int i=0;i<tmp.size();i++){
            cin>>tmp[i];
        }
        studios[studio_id].set(studio_id,tmp[0],pair<double,double>(tmp[1],tmp[2]),tmp[3],tmp[4],tmp[5]);
        if(studios[studio_id].pStatus == 1 ){
            product[studios[studio_id].type].push_back(studio_id);
            if (studios[studio_id].type >= 4 && studios[studio_id].type<=6){
                produce_product[studios[studio_id].type]++;
            }
        }
        if (studios[studio_id].type >= 4 && studios[studio_id].type<=7){
            
            if (studios[studio_id].bitSatus != 0 ){
                for(int i = 1; i <= studio_material[studios[studio_id].type-4][0]; i++){
                    if ((studios[studio_id].bitSatus & (int)pow(2, studio_material[studios[studio_id].type-4][i]))==0){
                        lack_material[studio_material[studios[studio_id].type - 4][i]]++;
                    }
                }
            }
        }
        if (studios[studio_id].type > 3)
        {
            if(studios[studio_id].type < 8){
                
                    for(int i = 0;i < 4;i++){
                        if(studios[studio_id].type == i+4){
                            for(int j = 0;j<studio_material[i][0];j++){
                                if((studios[studio_id].bitSatus & (int)pow(2,studio_material[i][j+1])) == 0){
                                    // if(studios_rid[studio_id][studio_material[i][j+1]] == -1)material[studio_material[i][j+1]].push_back(studio_id);
                                    for(int k = 0;k<4;k++){
                                        if(!eq(dis_to_studios[studio_id][0][robots[k].node_id],10000))
                                            material[k][studio_material[i][j+1]].push_back(studio_id);
                                    }
                                }
                            }
                        }
                    }
            
            }
            if(studios[studio_id].type == 8){
                for(int k = 0;k<4;k++){
                    if(!eq(dis_to_studios[studio_id][0][robots[k].node_id],10000))
                        material[k][7].push_back(studio_id);
                }
                                    
                // material[7].push_back(studio_id);
            }
            if(studios[studio_id].type == 9){
                for(int h = 1;h <=7;h++){
                    for(int k = 0;k<4;k++){
                        if(!eq(dis_to_studios[studio_id][0][robots[k].node_id],10000))
                            material[k][h].push_back(studio_id);
                    }
                    // material[h].push_back(studio_id);
                }
            }
        }
        studio_id++;
    }
    for(int i=1;i<7;i++){
        if(produce_product[i]<lack_material[i]){
            priority[i] = lack_material[i]-produce_product[i];
        }
    }
    for(int i=0;i<4;i++){
        vector<double> tmp(10,0);
        for(int i=0;i<tmp.size();i++){
            cin>>tmp[i];
        }
        robots[rob_id].collision_val_pre=robots[rob_id].collision_val;
        robots[rob_id].set(rob_id,tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],pair<double,double>(tmp[5],tmp[6]),tmp[7],
        pair<double,double>(tmp[8],tmp[9]));
        robots[rob_id].node_id = trans_pos_to_nodeID(rob_id);
        robots[rob_id].close_node = choose_close_node(robots[rob_id].get_type!=0, robots[rob_id].pos);
        robots[rob_id].radius = (robots[rob_id].get_type == 0? 0.45: 0.53);
        

        // if(gt(robots[rob_id].collision_val_pre, robots[rob_id].collision_val) && robots[rob_id].get_type != 0)
        //     cerr<<"time-collision:"<< state.FrameID <<"collision" <<rob_id<< endl<<"\n";
        rob_id++;
    }
    cin>>line;
    if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
    return false;
}

void out_put(){
    for(auto tmp:ins){
        cout<<tmp;
    }
    cout<<"OK\n";
    cout.flush();
}

pair<double, double> subVector(pair<double, double> a, pair<double, double> b) {
    return make_pair(a.first - b.first, a.second - b.second);
}

pair<double, double> addVector(pair<double, double> a, pair<double, double> b) {
    return make_pair(a.first + b.first, a.second + b.second);
}

pair<double, double> calVectorProduct(pair<double, double> a, double x) {
    return make_pair(a.first * x, a.second * x);
}


double calVectorProduct(pair<double, double> a, pair<double, double> b) {
    return a.first * b.first + a.second * b.second;
}

double calVectorSize(pair<double, double> a) {
    return sqrt(a.first * a.first + a.second * a.second);
}

double calcuDis(pair<double, double> a, pair<double, double> b)
{
    return sqrt((a.first - b.first) * (a.first - b.first) + (a.second - b.second) * (a.second - b.second));
}





// void calcuStudioDis(){
//     int num = studios.size();
//     int i, j;
//     for (i = 0; i < num; i++)
//     {
//         for (j = 0; j < i; j++)
//         {
//             dis[j][i] = dis[i][j] = calcuDis(studios[i].pos, studios[j].pos);
//         }
//     }
// }

// void print_matr(){
//     int i = 0;
//     int j;
//     for(i = 1 ; i <= 7; i++){
//     }
// }

double calAngle(pair<double, double> a, pair<double, double> b) {
    return acos(calVectorProduct(a, b) / calVectorSize(a) / calVectorSize(b));
}

double calAngle(pair<double, double> a) {

    double angle = acos(a.first / calVectorSize(a));
    return lt(a.second, 0.0) ? 2 * Pi- angle: angle;
}


PayLoad calPayload(Robot robot, pair<double, double> virtual_pos) {
    
    //int target = rand() % ((int)studios.size());
    //robots[robotID].target_id = target;

    //cerr << robotID << target<<"\n";

    // Robot robot = robots[robotID];
    // pair<double, double> virtual_pos = robots[robotID].virtual_pos;


    // cerr << robotID << "--"<< robot.target_id<<"\n";
    

    double distance = calcuDis(robot.pos, virtual_pos);
    double angular_acceleration = robot.get_type == 0? angular_acceleration_no :angular_acceleration_has;
    double acceleration = robot.get_type == 0? acceleration_no: acceleration_has;
    double speed = calVectorSize(robot.xy_pos) * (ge(calVectorProduct(robot.xy_pos, transformVector(robot.direction)), 0.0)? 1: -1);

    if(robot.target_id == -1) {
        return PayLoad((robot.get_type == 0? 0.45: 0.53), 0, 0, 0, 0, speed, 0);
    }

    // 计算机器人与目标点构成的向量与x轴正方向夹角
    pair<double, double> robotToStudio = subVector(virtual_pos, robot.pos);
    double angle1 = calAngle(robotToStudio);

    double angle2 = ge(robot.direction, 0.0) ? robot.direction: 2 * Pi + robot.direction;
    // double angle2 = calAngle(robot.xy_pos);

    double angle = angle2 - angle1;

    
    // if(state.FrameID==7010&& robotID==2) {
    //     printPair(robot.xy_pos);
    //     cerr<<"payload-speed:"<<speed<<"\n";
    // }
    int sign;

    if(ge(angle, 0) && lt(angle, Pi) || lt(angle, -Pi))
        sign = -1;
    else
        sign = 1;
    angle = fabs(angle);
    angle  = gt(angle, Pi)? 2 * Pi - angle: angle;


    // cerr<<"**"<< angle1<<"**dir:"<<robot.direction<<"**"<<angle2<<"\n";
    // cerr<<"**"<< angle << "**"<<distance<<"**"<<sign<<"\n";

    return PayLoad((robot.get_type == 0? 0.45: 0.53), angle, angular_acceleration, acceleration, distance, speed, sign);
}

bool eq(double a, double b) { return abs(a - b) < EPS; } // ==
bool gt(double a, double b) { return a - b > EPS; }      // >
bool lt(double a, double b) { return a - b < -EPS; }     // <
bool ge(double a, double b) { return a - b > -EPS; }     // >=
bool le(double a, double b) { return a - b < EPS; }      // <=

double getRobotRadius(int robot_id) {
    return robots[robot_id].get_type == 0? 0.45: 0.53;
}




// double calNextTimeDistance(double speed, double time, double  acceleration) {
//     double speed_max = min(speed + time * acceleration, 6.0);
//     double time_rest = max(time - (speed_max - speed) / acceleration, 0.0);
//     return (speed_max * speed_max - speed * speed) / 2 / acceleration + speed_max * time_rest;
// }

// bool checkTimeEnough(int robot_id, int target_id, int frame) {
//     double dis = distance(robot_id, target_id).second;
//     double time = frame * 0.02;//剩余秒数
//     double speed = calVectorSize(robots[robot_id].xy_pos);
//     double acceleration = robots[robot_id].get_type == 0? acceleration_no: acceleration_has;
//     // if((state.FrameID > 8500 )){
//     //     cerr<<"FrameID "<<state.FrameID<<"\n";
//     //     cerr<<robot_id<<"-"<<target_id<<"\n";
//     //     cerr<<"dis:"<<dis<<" speed:"<<speed<<"\n";
//     //     cerr<<calNextTimeDistance(speed, time, acceleration)<<"\n";
//     // }
//     if(lt(calNextTimeDistance(speed, time, acceleration), dis+3))
//         return false;
        
//     return true;
// }

bool checkEnough(int robot_id, int target_id, int frame)
{
    double dis = dis_to_studios[target_id][1][robots[robot_id].node_id]+0.4;
    if (dis > 0)
    {
        double time = dis/6/0.02; // 剩余秒数
        // cerr<<"time = "<<time<<" least time = "<<frame<<"\n";
        if (time > ((frame)+2))
            return true;
        else
            return false;
    }
    return false;
}

pair<double, double> transformVector(double direction) {
    direction = gt(direction, 0)? direction: Pi * 2 + direction;
    return make_pair(cos(direction), sin(direction));
}

// pair<double, double> getNextSpeed(double direction, pair<double, double> speed)
// {
//     direction = gt(direction, 0) ? direction : 2 * Pi + direction;
//     double val = calVectorSize(speed);
//     return make_pair(cos(direction) * val, sin(direction) * val);
// }

// pair<double, double> getNextPos(int robot_id)
// {
//     return addVector(robots[robot_id].pos, calVectorProduct(robots[robot_id].xy_pos, 0.02));
// }

// pair<double, double> getNextPos(pair<double, double> pos, pair<double, double> speed)
// {
//     return addVector(pos, calVectorProduct(speed, 0.02));
// }

// pair<double, double> getNextTimePos(int time, pair<double, double> pos, pair<double, double> speed, double direction, double angular_velocity)
// {
//     while (time--)
//     {
//         pos = getNextPos(pos, speed);
//         speed = getNextSpeed(direction, speed);
//         direction = direction + angular_velocity * 0.02;
//     }
//     return pos;
// }


// int getAvoidDirection(int goID, int stopID)
// {
//     double angle1 = calAngle(subVector(robots[stopID].pos, robots[goID].pos));
//     double angle2 = ge(robots[goID].direction, 0.0) ? robots[goID].direction : 2 * Pi + robots[goID].direction;
//     double angle3 = angle2 - angle1;

//     double included_angle = fabs(angle3);
//     included_angle = gt(included_angle, Pi) ? 2 * Pi - included_angle : included_angle;
//     int sign;
        
//     // 如果stopID-goID方向与goTD前进方向是锐角，go旋转
//     if (gt(fabs(included_angle), Pi / 2))
//         sign = 0;
//     else if (eq(angle3, 0) || eq(angle3, Pi) || eq(angle3, -Pi))
//         sign = gt(robots[goID].angular_velocity, 0)? 1: -1;
//     else if (gt(angle3, 0) && lt(angle3, Pi) || lt(angle3, -Pi))
//         sign = 1;
//     else
//         sign = -1;
        
//         // if(lt(included_angle, robots[goID].angular_velocity * 0.05)) {
//         //     ins[goID].rotate = gt(robots[goID].angular_velocity, 0)? Pi: -Pi;
//         // }
//         // if (lt(robots[goID].angular_velocity * sign, 0) && ge(fabs(robots[goID].angular_velocity) * 0.01, included_angle))
//         //     sign = -sign;

//         // if (state.FrameID == 4093){
//         //     cerr << "included_angle" << included_angle << "sign:" << sign << endl;
//         //     cerr<<sign<<"*"<<robots[goID].angular_velocity<<"\n";
//         // }
        
//     return sign;
// }

// bool isAcuteAngle(pair<double, double> a, pair<double, double> b)
// {
//     return gt(calVectorProduct(a, b), 0);
// }

// bool isAcuteAngle(pair<double, double> a, double x)
// {
//     x = gt(x, 0) ? x: 2 * Pi + x;
//     return gt(calVectorProduct(a, make_pair(cos(x), sin(x))), 0);
// }

void printPair(pair<double,double> a) {
    cerr<<"pos:("<<a.first<<", "<<a.second<<")\n";
}
void printPair(Vec a) {
    cerr<<"pos:("<<a.x<<", "<<a.y<<")\n";
}

// bool isNearWall(int id) {
//     int i=robots[id].pos.first;
//     int j=robots[id].pos.second;
//     if(i-1<=0||j-2<=0||i+2>=50||j+2>=50)
//         return true;
//     return false;
// }




void updateLastRate()
{
    for (int i = 0; i < 4; ++i)
    {
        robots[i].lastRate = ins[i].rotate;
    }
}

// double calc_priority(int studio_id){
//     double priority_value = 1;
//     // if(class_map == 3) {
//     //     if(studios[studio_id].type == 5 || studios[studio_id].type == 6) {
//     //         priority_value = 0.1;
//     //     }
//     // }
//     if (class_map == 4)
//     {

//         if (studios[studio_id].type == 1 )
//         {
//             priority_value = 0.8;
//         }
//         else if (studios[studio_id].type == 2){
//             priority_value = 0.9;
//         }
//         else
//             {
//                 if (studios[studio_id].type > 3)
//                 {
//                     if (priority[studios[studio_id].type] > 0)
//                     {

//                         priority_value -= (priority[studios[studio_id].type]) * 0.2;
//                         if (priority_value < 0.5)
//                             priority_value = 0.5;
//                     }
//                 }
//                 // else
//                 // {
//                 //     priority_value -= (priority[studios[studio_id].type]) * 0.1;
//                 //     if (priority_value < 0.5)
//                 //         priority_value = 0.5;
//                 // }
//             }
        
//     }
//     else if(class_map != 3){
//         if(priority[studios[studio_id].type]>0){
//             if(studios[studio_id].type>3){
//                 if(class_map == 2){
//                     priority_value -= (priority[studios[studio_id].type]) * 0.2;
//                 }
//                 else priority_value -= (priority[studios[studio_id].type]) * 0.3;
//                 if (priority_value < 0.5)
//                     priority_value=0.5;
//             }
//             else{
//                 priority_value -= (priority[studios[studio_id].type]) * 0.1;
//                 if (priority_value < 0.5)
//                     priority_value = 0.5;
//             }
//         }
//     }
//     // if (class_map == 3)
//     //     priority_value = 1;
//      return priority_value;
// }

void control(){
    contr_print_flag=1;
    payloads.clear();
    // if(state.FrameID>=800&&state.FrameID<=1900&&contr_print_flag){
    //      cerr<<"+++++"<<endl;
    // cerr<<" FrameID "<< state.FrameID<<" "<<robots[3].virtual_pos.first<<"-"<<robots[3].virtual_pos.second<<endl;
    // cerr<<"rob node_id"<<robots[3].close_node<<endl;
    // cerr<<" robot.cnt_tar "<<robots[3].cnt_tar<<endl;
    // cerr<<" rob.is_new_tar_ing=false;  "<<robots[3].is_new_tar_ing<<endl;
    // }
    for(int i=0;i<4;i++){
        // select_the_standard_id(robots[i]);
        check_robot_pos_status(robots[i]);
        auto tins=contr_one_rob(robots[i]);
        ins[i].forward=tins.forward;
        ins[i].rotate=tins.rotate;
        ins[i].robID=i;
        payloads.emplace_back(calPayload(robots[i], robots[i].virtual_pos));
    }
    // if(state.FrameID>=13000&&state.FrameID<=13700&&contr_print_flag){
    //     cerr<<"初始控制速度"<<ins[1].forward<<endl;
    //     cerr<<"机器人类型"<<robots[1].get_type<<endl;
    // }
//     if(state.FrameID>=800&&state.FrameID<=1900&&contr_print_flag){
//         cerr<<"-----------------"<<endl;
//     cerr<<" FrameID "<< state.FrameID<<" "<<robots[3].virtual_pos.first<<"-"<<robots[3].virtual_pos.second<<endl;
//     cerr<<"rob node_id"<<robots[3].close_node<<endl;
//     cerr<<" robot.cnt_tar "<<robots[3].cnt_tar<<endl;
// cerr<<" rob.is_new_tar_ing=false;  "<<robots[3].is_new_tar_ing<<endl;
//     }
    // if(state.FrameID>=1003&&state.FrameID<=1023){
    //     cerr<<state.FrameID<<" "<< ins[2].forward<<" "<<payLoad[2].distance<<" "<<payLoad[2].distance*sin(payLoad[2].angle) << endl;
    //     // cerr<<state.FrameID<<" "<<stop_dis<<" "<<payload.speed<<" "<<can_st<<" "<<(sin(cmpAngle)*payload.distance)<<" "<<robot.need_rote_wall<<"\n";
    //     // cerr<<robot.pos.first<<"-"<<robot.pos.second<<"\n";
    // }
    // cerr<<"2 rob forward "<<ins[2].forward<<"\n";
    // printPair(robots[2].xy_pos);
     contr_print_flag=0;
    // for(int i=0;i<4;i++){
    //     if(ins[i].forward==-1||ins[i].rotate==-1){
    //         cerr<<"io err\n";
    //     }
    // }
    //control
    // if(state.FrameID==1){
    //     cerr<<"------------------------------------\n";
    //     auto tmp=Calculate_the_trajectory(robots[0],0,100);
    //     auto iter=tmp.rbegin();
    //     int pos=0;
        
    //     cerr<<tmp.size()<<"\n";
    //     for(iter;iter!=tmp.rend();iter++){
    //         cerr<<state.FrameID+pos<<": "<<iter->first<<"-"<<iter->second<<" ";pos++;
    //     }
        
    //     cerr<<"\n";
    //     cerr<<"------------------------------------\n";
    // }

    // Collision_detection(payLoad);

    // if(state.FrameID >= 5600 && state.FrameID < 5610) {
    //     cerr<<state.FrameID<<"\n";
    //     // cerr<<payloads[2].angle<<payloads[2].sign<<"\n";
    //     cerr<<"ins:"<<ins[2].forward<<"  "<<ins[2].rotate<<"\n";
    // }
    // if(state.FrameID>=854&&state.FrameID<=858){
    //     cerr<<state.FrameID<<" ins befoer "<<ins[0].forward<<"\n";
    //     cerr<<check_will_colloWithWall(robots[0])<<"\n";
    // }

    // if(state.FrameID > 13000) cerr << "aa\n";
    collision_solve(25);
    // if(state.FrameID > 13000) cerr << "ss\n" ;

    // if(state.FrameID >= 5600 && state.FrameID < 5610) {
    //     cerr<<"~ins:"<<ins[2].forward<<"  "<<ins[2].rotate<<"\n";
    // }

    //  if(state.FrameID >= 212 && state.FrameID < 267) {
    //     cerr<<"~ins:"<<ins[3].forward<<"  "<<ins[3].rotate<<"\n";
    // }
  
    // if(state.FrameID >= 720 && state.FrameID <= 730)
    //     cerr<<"hello"<< robots[2].target_id<<"\n";

    // if(state.FrameID >= 2760 && state.FrameID < 2780) {
    //     cerr<<state.FrameID<<"\n";
    //     cerr<<"ins:"<<ins[0].forward<<"  "<<ins[0].rotate<<"\n";
    //     cerr<<"tar_dis: "<<payLoad[2].distance<<"\n";
    // }
    // for(int i=3;i<=3;i++){
    //     if(state.FrameID==1800){
    //         cerr<<"------------------"<<i<<"------------------\n";
    //         Calculate_the_trajectory(robots[i],0,25);
    //         cerr<<"------------------"<<i<<"------------------\n";
    //     }        
    // }

    // if(state.FrameID==149){
    //     cerr<<"------------------------------------\n";
    //     Calculate_the_trajectory(robots[0],0,20,0);
    //     cerr<<"------------------------------------\n";
    // }
    // if(state.FrameID==5490){
    //     cerr<<"------------------------------------\n";
    //     Calculate_the_trajectory(robots[0],0,20,0);
    //     cerr<<"------------------------------------\n";
    // }

    // if(state.FrameID==2962){
    //     cerr<<ins[2].
    // }
    
    // if(state.FrameID>=1800&&state.FrameID<=1825)
    // {
    //     for(int i=3;i<=3;i++){
    //     cerr<<" && "<<state.FrameID<<":real_wv  "
    //     <<robots[i].angular_velocity<<" real_dire: "<<robots[i].direction<<" real_pos "<<robots[i].pos.first<<"-"<<robots[i].pos.second
    //     <<" real_v_xy "<<robots[i].xy_pos.first<<"-"<<robots[i].xy_pos.second<<"\n" ;
    //     cerr<< payloads[i].speed<<"\n";       
    //     }

    // }

    // if(state.FrameID == 2940) {
    //     for(int i = 0;i<4;++i)
    //         trajectory[i]=Calculate_the_trajectory(robots[i], 0, 25);
    // }

    // if(state.FrameID >= 2940 && state.FrameID < 2940+25) {
    //     cerr<<"\n"<<state.FrameID<<"\n";
    //     for(int j=0;j<4;++j){
    //         cerr<<robots[j].id<<":\n"<<robots[j].pos.first<<","<<robots[j].pos.second<<"\n";
    //         cerr<<"predict\n";
    //         cerr<<trajectory[j][state.FrameID -2940].first<<","<<trajectory[j][state.FrameID -2940].second<<"\n";
    //     }
    // }
    // if(state.FrameID>=4855&&state.FrameID<=4900){
    //     cerr<<state.FrameID<<" ins after "<<ins[1].forward<<"\n";
    // }

    // for(int i =0;i<4;++i){
    //     if(lt(payloads[i].speed, 3)) {
    //         cerr<<state.FrameID<<":"<<i<<"speed:"<<payloads[i].speed<<"\n";
    //     }
    // }

    updateLastRate();
    // if(state.FrameID>=13000&&state.FrameID<=15700){
    //     cerr<<"最终控制速度"<<ins[3].forward<<endl;
    // }
    out_put();
}
// double get_at_stop_test(double t,double a,double v,int sign_v1){
//     double lef_time=0;
//     double s=0;
//     a*=sign_v1;
//     if(gt(a*sign_v1,0))a*=sign_v1;
//     if(lt(fabs(v),Pi)){
//         double tmpTime=(0-v)/(a);
//         double realTime=min(tmpTime,t);
//         s=v*realTime+0.5*a*realTime*realTime;
//         double res=(s);
//         if(le(t,tmpTime)){
//             if(gt(res*sign_v1,0)){
//                 return fabs(res);
//             }
//             else{
//                 return -1*fabs(res);
//             }
//             return (s);
//         }
//         t=t-realTime;
//     }
//     double res=(s+sign_v1*Pi*t);
//     if(gt(res*sign_v1,0)){
//         return fabs(res);
//     }else{
//         return -1*fabs(res);
//     }
//     return (s+sign_v1*Pi*t);
// }

bool check_no_send(int studio_id){
    int i;
    if(studios[studio_id].type>3){
        for(i = 1;i<= studio_material[studios[studio_id].type-4][0];i++){
            // if(state.FrameID>=2110&&state.FrameID<=2130&&studio_id==0){
            //     cerr<< "studio 10 rid = "<<studios_rid[studio_id][i]<<"type = "<<i<<"\n";
            // }
            if(studios_rid[studio_id][i]!=-1)break;
        }
        if(i!=studio_material[studios[studio_id].type-4][0]+1)return false;
    }
    return true;
}

/*
  control target_id
*/
// double check_root(){
//     if(Root.first<0 &&Root.second<0 )return -1;
//     if(Root.first>0 && Root.second>0){
//         if(Root.first>Root.second)return Root.second;
//         else return Root.first;
//     }
//     if(Root.first>Root.second)return Root.first;
//     return Root.second;
// }
// double Calc_collisions_dis(int robot_id,int studio_id){
//     int i,count = 0,target;
//     double time;
//     double dis = 0;
//     pair<double,double> line_speed; 
//     double dist = calcuDis(robots[robot_id].pos,studios[studio_id].pos);
//     line_speed.first = robots[robot_id].xy_pos.first;
//     line_speed.second = robots[robot_id].xy_pos.second;
//     //cerr<<"ddd"<<robots[robot_id].xy_pos.first<<' '<<robots[robot_id].xy_pos.second<<' '<<robots[robot_id].target_id<<' '<<robots[robot_id].pos.first<<' '<<robots[robot_id].pos.second;
//     robots[robot_id].xy_pos.first = (6/dist)*(studios[studio_id].pos.first-robots[robot_id].pos.first);
//     robots[robot_id].xy_pos.second = (6/dist)*(studios[studio_id].pos.second-robots[robot_id].pos.second);
//     target = robots[robot_id].target_id;
//     robots[robot_id].target_id = studio_id;
//     for(i = 0;i<4;i++){
//         if(i!=robot_id&&(robots[i].target_id != -1)){
//             vector<Robot>tmpS=robots;
//             will_collision(robot_id,i);
//             for(int i=0;i<tmpS.size();i++){
//                 if(tmpS[i]!=robots[i]){
//                     // cerr<<"--+--"<<" "<<i<<"\n";
//                 }
//             }
//             // time = check_root();
//             // cerr<<" time = "<<time;
//             // time = 1;
//             if(time>0){
//                 if(studios[studio_id].pos.first>(robots[robot_id].pos.first+robots[robot_id].xy_pos.first*time)){
//                     if(studios[studio_id].pos.second>(robots[robot_id].pos.second+robots[robot_id].xy_pos.second*time))count++;
//                 }
//             } 
//         }
//     }
//     // cerr<<" count = "<<count;
//     if(count>=2){
//         dis = 0.53*2*Pi*(1.3+(count-2)*0.3);
//     }
//     robots[robot_id].xy_pos.first = line_speed.first;
//     robots[robot_id].xy_pos.second = line_speed.second;
//     robots[robot_id].target_id = target;

//     return dis;
// }
bool check_material_full(int studio_id)
{
    int j;
    if(studios[studio_id].type>=3 &&studios[studio_id].type<=7){
        for (j = 0; j < studio_material[studios[studio_id].type - 4][0]; j++)
        {
            if ((studios[studio_id].bitSatus & (int)pow(2, studio_material[studios[studio_id].type - 4][j + 1])) == 0)
                break;
        }
        if (j == studio_material[studios[studio_id].type - 4][0])
        {
            return true;
        }
    }
    return false;
}

int check_lack(int studio_id){
    int i;
    int count = 0;
    if(studios[studio_id].type>=3 &&studios[studio_id].type<=7){
        for (i = 1; i <= studio_material[studios[studio_id].type - 4][0]; i++)
        {
            if ((studios[studio_id].bitSatus & (int)pow(2, studio_material[studios[studio_id].type - 4][i])) != 0) 
                count ++;
                
        }
        if(count != studio_material[studios[studio_id].type - 4][0]){
            return count+1;
        }
        else{
            return 1;
        }
        return 0;
    }
    return 0;
}
int check_lack_to_studio(int studio_id){
    int count_lack=0,count_have=0;
    if(studios[studio_id].type>3&&studios[studio_id].type<7){
        for(int i=0;i< studios_type[7].size();i++){
            if(studios[studios_type[7][i]].bitSatus!=0){
                if((studios[studios_type[7][i]].bitSatus & (int)pow(2 , studios[studio_id].type))==0)
                {
                    count_lack++;
                }
            }
        }
        for(int i=0;i<studios_type[studios[studio_id].type].size();i++){
            if(studios[studios_type[studios[studio_id].type][i]].pStatus==1||studios[studios_type[studios[studio_id].type][i]].r_time>0){
                count_have++;
            }
        }
    }
    // if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j)
    //     cerr<<"count_lack-count_have "<<(count_lack-count_have)<<"studio_type :"<<studios[studio_id].type<<endl;
    if(count_lack > count_have){
        return (count_lack-count_have);
    }
    return 0;
}
// bool check_other_robot_close(int robot_id,double threshold){
//     int studio_id = robots[robot_id].target_id;
//     int robot;
//     if(studios[studio_id].type>=3 &&studios[studio_id].type<=7){
//         for (int i = 1; i <= studio_material[studios[studio_id].type - 4][0]; i++)
//         {
//             robot = studios_rid[studio_id][studio_material[studios[studio_id].type - 4][i]];
//             if(robot!=-1 &&robots[robot].get_type!=0){
//                 // cerr<<"robot "<<robot_id<<"wait studio :"<<studio_id<<"robot :"<<robot<<"close close dis = "<<dis_to_studios[studio_id][1][robots[robot].node_id]<<"\n";
//                 if(lt(dis_to_studios[studio_id][1][robots[robot].node_id],threshold)){
//                     // cerr<<"true\n";
//                     return true;
//                 }
//             }         
//         }
//     }
//     return false;
// }
bool check_material_consumption(int studio_id,double dis){
    int robot;
    double dist;
    double max_dist=0;
    if(studios[studio_id].type>=3 &&studios[studio_id].type<=7){
        if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                            cerr<<"bbb"<<endl;
        }
        if(studios[studio_id].r_time != 0 ){
            if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                            cerr<<"ddd"<<endl;
            }
            for (int i = 1; i <= studio_material[studios[studio_id].type - 4][0]; i++)
            {
                robot = studios_rid[studio_id][studio_material[studios[studio_id].type - 4][i]];
                if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                            cerr<<"robot : "<<robot<<endl;
                }
                if(robot == -1 && ((studios[studio_id].bitSatus & (int)pow(2, studio_material[studios[studio_id].type - 4][i]))==0)){
                    if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                            cerr<<"false"<<endl;
                    }
                    return false;
                }
                if(robot != -1){
                    if(robots[robot].get_type!=0){
                        dist = dis_to_studios[studio_id][1][robots[robot].node_id];
                    }
                    else{
                         if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                            cerr<<"target_id"<<robots[robot].target_id<<endl;
                        }
                        if(robots[robot].target_id != -1)    
                            dist = dis_to_studios[robots[robot].target_id][0][robots[robot].node_id]+dis_to_studios[studio_id][1][studios[robots[robot].target_id].node_id];
                        else{
                            cerr<<"robot : "<<robot<<"studio : "<<studio_id<<" type : "<< studio_material[studios[studio_id].type - 4][i]<<endl;
                            dist = 0;
                        }
                    }
                    if(gt(dist,max_dist)){
                        max_dist = dist;
                    }
                }

            }
            if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                            cerr<<"fff"<<endl;
            }
            if(studios[studio_id].r_time==-1){
                if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                            cerr<<"eee"<<endl;
                }
                if(gt(dis,max_dist)){
                    // cerr<<"true1\n";
                    if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                            cerr<<"true"<<endl;
                        }
                    return true;
                }
            }
            else if(studios[studio_id].r_time>0&&studios[studio_id].pStatus!=1){
                if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                            cerr<<"kkk"<<endl;
                }
                if(gt((dist/6/0.02),studios[studio_id].r_time)){
                    if(gt(dis,max_dist)){
                        // cerr<<"true2\n";
                        if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                            cerr<<"true"<<endl;
                        }
                        return true;
                    }
                }
            }
        }
    }
    if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
        cerr<<"false "<<endl;
    }
    // cerr<<"false\n";
    return false;
}
bool check_double_choose(int robot_id,int studio_id,int material_id){
    int count = 0;
    for(int i = 0;i<robots.size();i++){
        if(i!=robot_id){
            if(robots[i].get_type == 0){
                if(studios[robots[i].target_id_buy].type == studios[material_id].type && robots[i].target_id_send == studio_id){
                    count++;
                }
            }
            else{
                if(robots[i].get_type== studios[material_id].type && robots[i].target_id_send == studio_id){
                    count++;
                }
            }
        }
    }
    if(count>=1){
        int j;
        // cerr<<"hh"<<endl;
        if(studios[studio_id].type>=3 &&studios[studio_id].type<=7){
            // cerr<<"kk"<<endl;
            for (j = 0; j < studio_material[studios[studio_id].type - 4][0]; j++)
            {
                if ((studios[studio_id].bitSatus & (int)pow(2, studio_material[studios[studio_id].type - 4][j + 1])) == 0){
                    // cerr<<"aa"<<endl;
                    if(studios_rid[studio_id][studio_material[studios[studio_id].type - 4][j + 1]]==-1){
                        break;
                    }
                    // cerr<<"bb"<<endl;
                }
            }
            // cerr<<"cc"<<endl;
            if (j == studio_material[studios[studio_id].type - 4][0]){
                if(count>1)return true;
            }
            else{
                if(count>=1) return true;
            }
        }
    }
    
    return false;
}
bool check_robots_wait_closest(int robot_id, double dist_robot,int studio_id){
    int i,count=0;
    double income;
    double dist;
    double income_ratio;
    for(int i=0;i<robots.size();i++){
        if(i!=robot_id&&robots[i].target_id != -1){
            if(robots[i].get_type == 0){
                dist = dis_to_studios[studio_id][0][robots[i].node_id]; 
            }
            else{
                dist = dis_to_studios[robots[i].target_id][1][robots[i].node_id];
                dist += dis_to_studios[studio_id][0][studios[robots[i].target_id].node_id];
            }
            if(lt(dist,dist_robot))count++;
        }
    }
    if(count>0){
        return false;
    }
    return true;
}
bool check_robot_close(int robot_id,int studio_id){
    int count = 0;
    double dist;
    // cerr<<robot_id<<' '<<studio_id<<' '<<endl;
    double dist_robot = dis_to_studios[studio_id][0][robots[robot_id].node_id];
    // cerr<<"dist_robot : "<<dist_robot<<endl;
    for(int i = 0;i<robots.size();i++){
        if(i!=robot_id &&robots[i].target_id != -1){
            if(robots[i].get_type != 0){
                dist = dis_to_studios[robots[i].target_id][1][robots[i].node_id];
                dist += dis_to_studios[studio_id][0][studios[robots[i].target_id].node_id];
            }
            else{
                dist = dis_to_studios[studio_id][1][robots[i].node_id];
            }
            if(lt(dist,dist_robot))count++;
        }
        // cerr<<"robot : "<<i<<" count : "<<count<<endl;

    }
    if(count>0){
        return true;
    }
    return false;
}
void check_suspicious_spots(int studio_id){
    int i,j;
    double dist1,dist2;
    i = (int)(studios[studio_id].node_id/100);
    j = studios[studio_id].node_id%100;
    studios[studio_id].has_suspicious_spots = 0;
    for(int x =(i-1);x<(i+2);x++){
        for(int y = (j-1);y<(j+2);y++){
            if(x==i&&y==j)continue;
            if(x<0||x>99)continue;
            if(y<0||y>99)continue;
            dist1 = dis_to_studios[studio_id][0][x*100+y];
            if(eq(dist1,10000))continue;
            dist2 = dis_to_studios[studio_id][1][x*100+y];
            if(eq(dist2,10000))continue;
            if(lt(dist1,1.5) &&gt(dist2,8)){
                // cerr<<"studio : "<<studio_id<<"dist1 = "<<dist1<<"dist2 = "<<dist2<<" x offset : "<<(x-i)<<" y offset : "<<(y-j)<<endl;
                studios[studio_id].suspicious_spots.push_back(x*100+y);
                studios[studio_id].has_suspicious_spots = 1;
            }
        }
    }
    // if(studios[studio_id].has_suspicious_spots != 1)studios[studio_id].has_suspicious_spots = 0;
}
double check_suspicious_dis(int robot_id,int studio_id,int send_id,double dis){
    double min_dist = dis;
    double dist;
    int min_subscript = -1;
    if(lt(dis_to_studios[studio_id][1][robots[robot_id].node_id],dis_to_studios[send_id][1][robots[robot_id].node_id])){
        return dis;
    }
    for(int i = 0;i<studios[studio_id].suspicious_spots.size();i++){
        dist = dis_to_studios[send_id][1][studios[studio_id].suspicious_spots[i]];
        if(lt(dist,(min_dist-3))){
            min_dist = dist;
            min_subscript = studios[studio_id].suspicious_spots[i];
        }
    }
    // cerr<<"studio from "<<studio_id<<" - "<<send_id<<"offset : "<<min_subscript<<" change dist "<<dis<<" to "<<min_dist<<endl;
    return min_dist;
}
double calc_time_factor(int studio_material,int studio_send){
    double dist = dis_to_studios[studio_send][1][studios[studio_material].node_id];
    double time = dist/4/0.02;
    double time_factor = (1-pow((1-(1-pow((time/15000),2))),0.5)*0.2+0.8);
    return time_factor;
}
pair<pair<int,int>,double> new_pick_point(int robot_id,int state_type,int change_target_flag){
    double max = 0;
    int studio_buy = -1,studio_send = -1;
    double dist;
    double dist2;
    double income;
    double income_ratio;      //收益比
    int material_studio_id;
    int flag =0;
    int time = 0;
    // if(state.FrameID>14000){
    //     flag=1;
    // }
    // if(state_type == 1){
    //     // for(int i =0;i<studios.size();i++){
    //     //     if(studios[i].type<=3){
    //     //     }
    //     // }
    // }
    // else 
    if(state_type ==2){
        for(int i =0;i<studios.size();i++){
            if(studios[i].type>3){
                for(int j = 1;j<=studio_material[studios[i].type-4][0];j++){
                    if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                            cerr<<"studio : "<<i<<"material_type : "<<studio_material[studios[i].type-4][j]<<" material rid : "<<studios_rid[i][studio_material[studios[i].type-4][j]]<<endl;
                        }
                    if(((studios[i].bitSatus & (int)pow(2,studio_material[studios[i].type-4][j])) == 0||(check_material_full(i)&&studios[i].r_time == -1)) && (studios[i].type>7||studios_rid[i][studio_material[studios[i].type-4][j]]==-1)){
                        
                        for(int k=0;k<studios[i].material_studios[j-1].size();k++){
                            material_studio_id = studios[i].material_studios[j-1][k];
                            if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                cerr<<"studio : "<<i<<" material : "<< material_studio_id <<" r_id = "<<studios[material_studio_id].r_id<<"\n";
                            }
                            if((studios[material_studio_id].pStatus == 1||(studios[material_studio_id].r_time>0)) && studios[material_studio_id].r_id < 50 ){         
                                income = (price[studios[material_studio_id].type][1]-price[studios[material_studio_id].type][0])* calc_time_factor(material_studio_id,i);
                                if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                    cerr<<"studio : "<<i<<" check_lack_material : "<<check_lack(i)<<endl;
                                }
                                income = income + (price[studios[i].type][1]-price[studios[i].type][0])/(studio_material[studios[i].type-4][0]*2)*(check_lack(i));
                                income += check_lack_to_studio(i)*((price[7][1]-price[7][0])/3);
                                dist = dis_to_studios[material_studio_id][0][robots[robot_id].node_id];   //
                                time = dist/6/0.02;
                                if(eq(dist,10000))continue;
                                if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                    cerr<<"bbb "<<endl;
                                }
                                if(studios[material_studio_id].r_id != -1){
                                    if(studios[material_studio_id].type>3){
                                        if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                            cerr<<studios[material_studio_id].r_id<<endl;
                                        }
                                        dist2 = dist-dis_to_studios[material_studio_id][0][robots[studios[material_studio_id].r_id ].node_id];
                                        if(lt(dist2/6/0.02,product_time[studios[material_studio_id].type])){
                                            if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                                // cerr<<"ccc "<<endl;
                                            }
                                            continue;
                                        }
                                    }
                                    else{
                                        if(lt(dist/6/0.02,product_time[studios[material_studio_id].type]))continue;
                                    }
                                }
                                if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                    // cerr<<"ccc "<<endl;
                                }
                                if(!check_no_send(material_studio_id))continue;
                                if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                    cerr<<"ddd "<<endl;
                                }
                                if(studios[material_studio_id].pStatus != 1 ){
                                    // if(!check_no_send(material_studio_id)){
                                    //     // if(state.FrameID>2110 &&state.FrameID<2130)
                                    //     //     cerr<<material_studio_id<<" have robot to send\n";
                                    //     continue;
                                    // }
                                    if(lt(dist/6/0.02,studios[material_studio_id].r_time)){
                                        if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                            cerr<<"123 "<<endl;
                                        }
                                        dist2 = (studios[material_studio_id].r_time-(dist/6/0.02))*0.02*6;
                                        if(!check_robots_wait_closest(robot_id,dist2+5,material_studio_id))continue;
                                        dist += dist2;
                                    }
                                }
                                if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                    cerr<<"eee "<<endl;
                                }
                                if(studios[material_studio_id].has_suspicious_spots){
                                    dist += check_suspicious_dis(robot_id,material_studio_id,i,dis_to_studios[i][1][studios[material_studio_id].node_id]);
                                    time += check_suspicious_dis(robot_id,material_studio_id,i,dis_to_studios[i][1][studios[material_studio_id].node_id])/4/0.02;
                                }
                                else{
                                     dist += dis_to_studios[i][1][studios[material_studio_id].node_id];
                                     time += dis_to_studios[i][1][studios[material_studio_id].node_id]/4/0.02;
                                }
                                // cerr<<"kkk"<<endl;
                                if(flag == 1){
                                    if(time>(15000-state.FrameID))continue;
                                }
                                if(check_robot_close(robot_id,material_studio_id)){
                                    // cerr<<"into"<<endl;
                                    continue;
                                }
                                // cerr<<"out"<<endl;
                                if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                    cerr<<"ggg "<<endl;
                                }
                                income_ratio = (income/dist);
                                if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                    cerr<<"to buy dist = "<< dis_to_studios[material_studio_id][0][robots[robot_id].node_id]<<" to send dist = "<<dis_to_studios[i][1][studios[material_studio_id].node_id]<<"\n";
                                    cerr<< "robot : "<<robot_id<<" buy : "<<material_studio_id<<" type : "<<studios[material_studio_id].type<<" send : "<<i<<" type : "<< studios[i].type<<" income_ratio : "<<income_ratio<<" income = "<<income<<" dist = "<<dist<<"\n";
                                }
                                if(gt(income_ratio,max)){
                                    max = income_ratio;
                                    studio_buy = material_studio_id;
                                    studio_send = i;
                                }
                            }
                        }
                    }
                }
            }
        }
        if(studio_buy == -1 && change_target_flag==0){
            if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                // cerr<<"eee "<<endl;
            }
            for(int i =0;i<studios.size();i++){
                if(studios[i].type>3){
                    for(int j = 1;j<=studio_material[studios[i].type-4][0];j++){
                        // if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                        //     cerr<<"studio : "<<i<<"material_type : "<<studio_material[studios[i].type-4][j]<<" material rid : "<<studios_rid[i][studio_material[studios[i].type-4][j]]<<endl;
                        // }
                        if(((studios[i].bitSatus & (int)pow(2,studio_material[studios[i].type-4][j])) == 0||(check_material_full(i)&&studios[i].r_time == -1))){
                            for(int k=0;k<studios[i].material_studios[j-1].size();k++){
                                material_studio_id = studios[i].material_studios[j-1][k];
                                if((studios[material_studio_id].pStatus == 1||(studios[material_studio_id].r_time>0)) && studios[material_studio_id].r_id < 50 ){         //
                                    income =(price[studios[material_studio_id].type][1]-price[studios[material_studio_id].type][0])* calc_time_factor(material_studio_id,i);
                                    // cerr<<"studio : "<<i<<" check_lack_material : "<<check_lack(i)<<"\n";
                                    income = income + (price[studios[i].type][1]-price[studios[i].type][0])/(studio_material[studios[i].type-4][0]*2)*(check_lack(i));
                                    income += check_lack_to_studio(i)*((price[7][1]-price[7][0])/3);
                                    dist = dis_to_studios[material_studio_id][0][robots[robot_id].node_id];   //
                                    time = dist/6/0.02;
                                    if(eq(dist,10000))continue;
                                    if(studios[material_studio_id].r_id != -1){
                                        if(studios[material_studio_id].type>3){
                                            dist2 = dist-dis_to_studios[material_studio_id][0][robots[studios[material_studio_id].r_id ].node_id];
                                            if(lt(dist2/6/0.02,product_time[studios[material_studio_id].type]))continue;
                                        }
                                        else{
                                            if(lt(dist/6/0.02,product_time[studios[material_studio_id].type]))continue;
                                        }
                                    }
                                    if(!check_no_send(material_studio_id))continue;
                                    if(studios[material_studio_id].pStatus != 1 ){
                                        // if(!check_no_send(material_studio_id)){
                                        //     // if(state.FrameID>2110 &&state.FrameID<2130)
                                        //     //     cerr<<material_studio_id<<" have robot to send\n";
                                        //     continue;
                                        // }
                                        if(lt(dist/6/0.02,studios[material_studio_id].r_time)){
                                            dist += (studios[material_studio_id].r_time-(dist/6/0.02))*0.02*6;
                                            time += studios[material_studio_id].r_time-(dist/6/0.02);
                                        }
                                    }
                                    dist += dis_to_studios[i][1][studios[material_studio_id].node_id];
                                    time += dis_to_studios[i][1][studios[material_studio_id].node_id]/4/0.02;
                                    if(flag == 1){
                                        if(time>(15000-state.FrameID))continue;
                                    }
                                    if((studios_rid[i][studio_material[studios[i].type-4][j]]!=-1)){
                                        // if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                        //     cerr<<"aaa "<<endl;
                                        // }
                                        if(!check_material_consumption(i,dist))continue;
                                        // if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                        //     cerr<<"bbb "<<endl;
                                        // }
                                        if(check_double_choose(robot_id,i,material_studio_id))continue;
                                        // if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                        //     cerr<<"ccc "<<endl;
                                        // }
                                    }
                                    income_ratio = (income/dist);
                                    // if(state.FrameID>=start_time &&state.FrameID< end_time &&cerr_flag_j){
                                    //     cerr<<"走的目标-1\n";
                                    //     cerr<<"to buy dist = "<< dis_to_studios[material_studio_id][0][robots[robot_id].node_id]<<" to send dist = "<<dis_to_studios[i][1][studios[material_studio_id].node_id]<<"\n";
                                    //     cerr<< "robot : "<<robot_id<<" buy : "<<material_studio_id<<" type : "<<studios[material_studio_id].type<<" send : "<<i<<" type : "<< studios[i].type<<" income_ratio : "<<income_ratio<<" income = "<<income<<" dist = "<<dist<<"\n";
                                    // }
                                    if(gt(income_ratio,max)){
                                        max = income_ratio;
                                        studio_buy = material_studio_id;
                                        studio_send = i;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else if(state_type == 3){
        int item_type =robots[robot_id].get_type;
        // 只有送
        for(int i=0;i<studios.size();i++){
            for(int j=1;j<=material_send[item_type][0];j++){
                if(item_type != 7){
                    if(studios[i].type == material_send[item_type][j] && (studios_rid[i][item_type] == -1) ){
                        if(((studios[i].bitSatus & ((int)pow(2,item_type)))==0)||((check_material_full(i)&&(studios[i].pStatus != 1)&&(studios[i].r_time>0&&(checkEnough(robot_id,i,studios[i].r_time)))))){
                            dist = dis_to_studios[i][1][robots[robot_id].node_id];   //修改
                            time = dist/4/0.02;
                            if(eq(dist,10000))continue;
                            if(flag == 1){
                                if(time>(15000-state.FrameID))continue;
                            }
                            income = price[item_type][1]-price[item_type][0];
                            // cerr<<"studio : "<<i<<" check_lack_material : "<<check_lack(i)<<"\n";
                            income = income + (price[studios[i].type][1]-price[studios[i].type][0])/(studio_material[studios[i].type-4][0]*2)*(check_lack(i));
                            income += check_lack_to_studio(i)*((price[7][1]-price[7][0])/3);
                            income_ratio = (income/dist);
                            if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                                    cerr<<" to send dist = "<<dis_to_studios[i][1][robots[robot_id].node_id]<<"\n";
                                    cerr<< "robot : "<<robot_id<<" send : "<<i<<" type : "<< studios[i].type<<" income_ratio : "<<income_ratio<<" income = "<<income<<" dist = "<<dist<<"\n";
                            }
                            if(gt(income_ratio,max)){
                                max = income_ratio;
                                studio_send = i;
                            }
                        }
                    }
                }
                else{
                    if(studios[i].type == material_send[item_type][j]){
                        dist = dis_to_studios[i][1][robots[robot_id].node_id];
                        if(eq(dist,10000))continue;
                        income = price[item_type][1]-price[item_type][0];
                        income = income + (price[studios[i].type][1]-price[studios[i].type][0])/(studio_material[studios[i].type-4][0]*2)*(check_lack(i));
                        // income += check_lack_to_studio(i)*((price[7][1]-price[7][0])/6);
                        income_ratio = (income/dist);
                        if(gt(income_ratio,max)){
                            max = income_ratio;
                            studio_send = i;
                        }
                    }
                }
            }
            if(studios[i].type == 9){
                dist = dis_to_studios[i][1][robots[robot_id].node_id];
                if(eq(dist,10000))continue;
                income = price[item_type][1]-price[item_type][0];
                income_ratio = (income/dist);
                if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                    cerr<<" to send dist = "<<dis_to_studios[i][1][robots[robot_id].node_id]<<"\n";
                    cerr<< "robot : "<<robot_id<<" send : "<<i<<" type : "<< studios[i].type<<" income_ratio : "<<income_ratio<<" income = "<<income<<" dist = "<<dist<<"\n";
                }
                if(gt(income_ratio,max)){
                    max = income_ratio;
                    studio_send = i;
                }
            }
        }
    }
    if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
        cerr<<" buy = "<<studio_buy<<" send = "<<studio_send<<"\n";
        cerr<<"income_ratio "<<max<<"\n";
    }
    pair<int,int> road (studio_buy,studio_send);
    return pair<pair<int,int>,double>(road,max);
}
void change_status(int robot_id,pair<pair<int,int>,double>temp){
    robots[robot_id].target_id_buy = temp.first.first;
    robots[robot_id].target_id_send = temp.first.second;
    // cerr<<" buy = "<<robots[robot_id].target_id_buy<<" send = "<<robots[robot_id].target_id_send<<"\n";
    robots[robot_id].target_id = robots[robot_id].target_id_buy;
    if(robots[robot_id].target_id != -1){
        if (studios[robots[robot_id].target_id].r_id != -1 && studios[robots[robot_id].target_id].r_id != robot_id)
            studios[robots[robot_id].target_id].r_id += 50;
        else
            studios[robots[robot_id].target_id].r_id = robot_id;
        robot_get_type[studios[robots[robot_id].target_id].type]++;
        // cerr<<" studio_rid : "<<robots[robot_id].target_id_send<<" - "<<studios[robots[robot_id].target_id_buy].type<<"\n";
        if(studios[robots[robot_id].target_id_send].type!=8&&studios[robots[robot_id].target_id_send].type!=9 &&studios_rid[robots[robot_id].target_id_send][studios[robots[robot_id].target_id_buy].type]==-1){
            studios_rid[robots[robot_id].target_id_send][studios[robots[robot_id].target_id_buy].type] = robot_id;
        }
    }
}
void complete_trans(int robot_id,int change_target_flag){
    pair<pair<int,int>,double>temp;
    // cerr<<"bbb\n";
    temp=new_pick_point(robot_id,2,change_target_flag);
    // if(state.FrameID>=1550 && state.FrameID<=1600 &&robot_id == 0)
    // {
    //     cerr <<"temp buy :"<<temp.first.first<<" temp send :"<<temp.first.second;
    // }
    // cerr<<"ccc\n";
    // cerr<<" buy = "<<temp.first.first<<" send = "<<temp.first.second<<"\n";
    change_status(robot_id,temp);
    // cerr<<"robot_id : "<<robot_id<<"\n";
}
bool check_robots_change_closest(int robot_id, pair<pair<int,int>,double>temp){
    int i,count=0;
    double income;
    double dist;
    double income_ratio;
    income = price[studios[temp.first.first].type][1]-price[studios[temp.first.first].type][0];
    income = income + (price[studios[temp.first.second].type][1]-price[studios[temp.first.second].type][0])/(studio_material[studios[temp.first.second].type-4][0]*2)*check_lack(temp.first.second);
    income += check_lack_to_studio(temp.first.second)*((price[7][1]-price[7][0])/3);
    for(int i=0;i<robots.size();i++){
        if(i!=robot_id){
            if(robots[i].get_type == 0){
                dist = dis_to_studios[temp.first.first][0][robots[i].node_id]; 
            }
            else{
                dist = dis_to_studios[robots[i].target_id][1][robots[i].node_id];
                dist += dis_to_studios[temp.first.first][0][studios[robots[i].target_id].node_id];
            }
            dist += dis_to_studios[temp.first.second][1][studios[temp.first.first].node_id];
            income_ratio = (income/dist);
            if(state.FrameID>start_time &&state.FrameID<end_time &&cerr_flag_j){
                cerr<<"robot :"<< i<<"studio :"<<temp.first.first<<" - "<<temp.first.second<<"dist = "<<dist<<" income = "<<income<<" ratio = "<<income_ratio<<"\n"; 
            }
            if(lt(income_ratio,temp.second))count++;
        }
    }
    if(count>0){
        return false;
    }
    return true;
}

void charge_target(int robot_id){
    int i =robot_id;
    pair<pair<int,int>,double>temp;
    if(robots[i].real_get_type==0){
        temp=new_pick_point(i,2,1);
        if(temp.first.first != -1){
            double income = (price[studios[robots[i].target_id_buy].type][1]-price[studios[robots[i].target_id_buy].type][0])* calc_time_factor(temp.first.first,temp.first.second);
            income += (price[studios[robots[i].target_id_send].type][1]-price[studios[robots[i].target_id_send].type][0])/(studio_material[studios[robots[i].target_id_send].type-4][0]*2)*(check_lack(robots[i].target_id_send));
            income += check_lack_to_studio(robots[i].target_id_send)*((price[7][1]-price[7][0])/3);
            double income_ratio =  income/(dis_to_studios[robots[i].target_id_buy][0][robots[i].node_id]+dis_to_studios[robots[i].target_id_send][1][studios[robots[i].target_id_buy].node_id]);
            if(lt(income_ratio+15,temp.second)&&temp.first.second != robots[i].target_id_buy){
                // cerr<<" robot : "<<i<<" change_target_id "<<"from "<<robots[i].target_id_buy<<" - "<<robots[i].target_id_send<<" to "<<temp.first.first<<" - "<<temp.first.second<<" income = "<<income_ratio<<" after change income = "<<temp.second<<"\n";
                if(check_robots_change_closest(robot_id,temp)){
                    if (studios[robots[i].target_id].r_id >= 50)
                        studios[robots[i].target_id].r_id -= 50;
                    else
                        studios[robots[i].target_id].r_id = -1;
                    if(studios[robots[i].target_id_send].type!=8&&studios[robots[i].target_id_send].type!=9)studios_rid[robots[i].target_id_send][studios[robots[i].target_id_buy].type] = -1;
                    robot_get_type[studios[robots[i].target_id].type]--;
                    change_status(i,temp);
                    robots[i].cnt_tar=robots[i].node_id;
                }
            }
        }
    }
    // else{
        // temp=new_pick_point(i,3,1);
        // if(studios[temp.first.second].type!=9){
        //     if(temp.first.second != -1){
        //         double income = (price[robots[i].get_type][1]-price[robots[i].get_type][0]);
        //         income += (price[studios[robots[i].target_id_send].type][1]-price[studios[robots[i].target_id_send].type][0])/(studio_material[studios[robots[i].target_id_send].type-4][0]*2)*(check_lack(robots[i].target_id_send));
        //         income += check_lack_to_studio(robots[i].target_id_send)*((price[7][1]-price[7][0])/3);
        //         double income_ratio =  income/(dis_to_studios[robots[i].target_id][1][robots[robot_id].node_id]);
        //         if(lt(income_ratio,temp.second)&& temp.first.second != robots[i].target_id_send){
        //             if(state.FrameID>start_time&&state.FrameID<end_time){
        //                 cerr<<income<<"dist = "<<(dis_to_studios[robots[i].target_id][1][robots[robot_id].node_id])<<"\n";
        //                 cerr<<income_ratio<<" change to "<<temp.second<<"\n";
        //             }
        //             // cerr<<" robot : "<<i<<" change_target_id "<<"from "<<robots[i].target_id_buy<<" - "<<robots[i].target_id_send<<" to "<<temp.first.first<<" - "<<temp.first.second<<" income = "<<income_ratio<<" after change income = "<<temp.second<<"\n";
        //             if(studios[robots[i].target_id_send].type!=8&&studios[robots[i].target_id_send].type!=9)studios_rid[robots[i].target_id_send][robots[i].get_type] = -1;
        //             robots[i].target_id_send = temp.first.second;
        //             robots[i].target_id = robots[i].target_id_send;
        //             robots[i].cnt_tar=robots[i].node_id;
        //             if(studios[robots[i].target_id_send].type!=8&&studios[robots[i].target_id_send].type!=9 &&studios_rid[robots[i].target_id_send][robots[i].get_type] == -1)studios_rid[robots[i].target_id_send][robots[i].get_type] = i;
        //         }
        //     }
        // }
    // }
}
void new_robot_judge(){
    double dist;
    pair<pair<int,int>,double>temp;
    for(int i=0;i<4;i++){
        if(robots[i].loc_id == robots[i].target_id && robots[i].target_id != -1){
            if(robots[i].get_type != 0){
                //sell;
                // cerr<<"aaa\n";
                ins[i].sell = 1;
                ins[i].buy = -1;
                // if(i == 1)cerr<<" ins[i].buy = "<<ins[i].buy<<" ins[i].sell = "<<ins[i].sell<<"\n";
                robots[i].lastSign=0;
                robots[i].isTurn=0;
                if(studios[robots[i].loc_id].type>3&&studios[robots[i].loc_id].type<=7){
                    studios[robots[i].loc_id].bitSatus += (int)pow(2,robots[i].get_type);
                }
                studios_rid[robots[i].loc_id][robots[i].get_type] = -1;
                robots[i].get_type = 0;
                complete_trans(i,0);
                robots[i].cnt_tar=robots[i].node_id;
            }
            else{
                //do something buy;
                // cerr<<"bbb\n";
                if(studios[robots[i].target_id].pStatus==1){
                    
                    ins[i].buy = 1;
                    ins[i].sell = -1;
                    robots[i].lastSign=0;
                    robots[i].isTurn=0;
                    if (studios[robots[i].loc_id].r_id >= 50)
                        studios[robots[i].loc_id].r_id -= 50;
                    else
                        studios[robots[i].loc_id].r_id = -1;
                    studios[robots[i].loc_id].pStatus = 0;
                    robots[i].get_type = studios[robots[i].loc_id].type;
                    robots[i].target_id = robots[i].target_id_send;
                    if(state.FrameID>14000){
                        if((dis_to_studios[robots[i].target_id][1][robots[i].node_id]/4/0.02)>(15000-state.FrameID)){
                            ins[i].buy = -1;
                            robots[i].target_id = -1;
                            studios_rid[robots[i].target_id_send][robots[i].get_type]=-1;
                            robots[i].target_id_send = -1;
                            robots[i].get_type = 0;
                        }
                    }
                    robots[i].cnt_tar=robots[i].node_id;
                }
                   
                else{
                    // cerr<<"robot: "<<i<<" wait \n";
                    ins[i].buy = -1;   //wait;
                    ins[i].sell = -1;
                    dist = studios[robots[i].target_id].r_time*6*0.02;
                    if(!check_robots_wait_closest(i,dist,robots[i].target_id)){
                        // cerr<<"change\n";
                        // complete_trans(i,0);
                        temp=new_pick_point(i,2,0);
                        if(temp.first.first != -1){
                            if (studios[robots[i].target_id].r_id >= 50)
                                studios[robots[i].target_id].r_id -= 50;
                            else
                                studios[robots[i].target_id].r_id = -1;
                            if(studios[robots[i].target_id_send].type!=8&&studios[robots[i].target_id_send].type!=9)studios_rid[robots[i].target_id_send][studios[robots[i].target_id_buy].type] = -1;
                            robot_get_type[studios[robots[i].target_id].type]--;
                            change_status(i,temp);
                            robots[i].cnt_tar=robots[i].node_id;
                        }
                    }
                }
            }
        }
        else{
            // cerr<<"ccc\n";
            ins[i].buy = -1;
            ins[i].sell = -1;
            
            //change_targte
            if(robots[i].target_id != -1)
                charge_target(i);
            if(robots[i].target_id != -1&&robots[i].get_type == 0){
                if(check_double_choose(i,robots[i].target_id_send,robots[i].target_id_buy)){
                    temp=new_pick_point(i,2,0);
                    if(temp.first.first!= -1){
                        if (studios[robots[i].target_id].r_id >= 50)
                            studios[robots[i].target_id].r_id -= 50;
                        else
                            studios[robots[i].target_id].r_id = -1;
                        if(studios[robots[i].target_id_send].type!=8&&studios[robots[i].target_id_send].type!=9)studios_rid[robots[i].target_id_send][studios[robots[i].target_id_buy].type] = -1;
                        robot_get_type[studios[robots[i].target_id].type]--;
                        change_status(i,temp);
                        robots[i].cnt_tar=robots[i].node_id;
                    }
                }

            //     }
            }
        }
        if(robots[i].target_id == -1){
            // cerr<<"ddd\n";
            if(robots[i].get_type == 0){
                complete_trans(i,0);
                robots[i].cnt_tar=robots[i].node_id;
            }
            else{
                robots[i].target_id_send = new_pick_point(i,3,0).first.second;
                if(robots[i].target_id_send != -1){
                    robots[i].target_id = robots[i].target_id_send;
                    robots[i].cnt_tar=robots[i].node_id;
                    if(studios[robots[i].target_id_send].type!=8&&studios[robots[i].target_id_send].type!=9)studios_rid[robots[i].target_id_send][studios[robots[i].get_type].type] = i;
                }
            }
        }
        if(studios[robots[i].target_id_send].type>3&&studios[robots[i].target_id_send].type<=7){
            if(robots[i].get_type == 0){
                if(studios_rid[robots[i].target_id_send][studios[robots[i].target_id].type]==-1){
                    if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                        cerr<<" robot : "<<i<<" send target : "<<robots[i].target_id_send<<" type: "<<studios[robots[i].target_id_send].type<<" r_id == -1"<<" buy type: "<<studios[robots[i].target_id].type<<"\n";
                    }
                    studios_rid[robots[i].target_id_send][studios[robots[i].target_id].type] = i;
                }
            }
            else if(robots[i].get_type != 0){
                if(studios_rid[robots[i].target_id_send][robots[i].get_type]==-1){
                    if(state.FrameID>start_time&&state.FrameID<end_time&&cerr_flag_j){
                        cerr<<" robot : "<<i<<" send target : "<<robots[i].target_id_send<<" type: "<<studios[robots[i].target_id_send].type<<" r_id == -1"<<" buy type: "<<robots[i].get_type<<"\n";
                    }
                    studios_rid[robots[i].target_id_send][robots[i].get_type] = i;
                }
            }
        }
        // if(state.FrameID>4400&&state.FrameID>4450){
        //     cerr<<"robot : "<<i<<" target id = "<<robots[i].target_id<<"from "<<robots[i].target_id_buy<<" - "<<robots[i].target_id_send<<" ins[i].buy = "<<ins[i].buy<<" ins[i].sell = "<<ins[i].sell<<"loc_id :"<<robots[i].loc_id<<"\n";
        // }    
    }
}
void new_first_action(){
    for(int i=0;i<robots.size();i++){
        // cerr<<"aaa\n";
        robots[i].real_get_type = robots[i].get_type;
        complete_trans(i,0);
        robots[i].cnt_tar=robots[i].node_id;
        // cerr<<"robot : "<<i<<" target id = "<<robots[i].target_id<<"\n";
    }
    // cerr<<" open "<<"type : "<<studios[12].type<<' '<<studios[12].r_id<<' '<<dis_to_studios[12][0][robots[2].node_id]<<"\n";
}
void new_robot_action(){
    int j = 0;
    for(int i =0;i<=7;i++)robot_get_type[i]=0;
    for(int i = 0;i<4;i++){
        robots[i].real_get_type = robots[i].get_type;
        robots[i].last_target_id = robots[i].target_id;
        if(robots[i].get_type != 0)robot_get_type[robots[i].get_type]++;
        else if(robots[i].target_id != -1)robot_get_type[studios[robots[i].target_id].type]++;
    }
    if(state.FrameID >=start_time &&state.FrameID <= end_time &&cerr_flag_j){
        cerr<<" r_id "<<studios_rid[2][1]<<"\n";
    }
    new_robot_judge();
    
    // for(int i = 0;i<4;i++){
    //     if(robots[i].last_target_id != robots[i].target_id){
    //         cerr<<"time = "<<state.FrameID<<endl;
    //         cerr<<"robot : "<<i<<" change target id = "<<robots[i].target_id<<"from "<<robots[i].target_id_buy<<" - "<<robots[i].target_id_send<<endl;
    //     }
    // }
}

vector<double>  get_T_limits(Robot& robot){
    double radius=0.53;
    double tmpA=robot.direction;
    auto pos=robot.pos;
    pos.first+=3*cos(tmpA);
    pos.second+=3*sin(tmpA);
    vector<double> tmp{-7,-7};
    double redundancy= 0;//冗余，避免频繁转向
    if(gt(pos.first-redundancy,0)&&lt(pos.second-redundancy,0)){//只靠近下方x轴
        tmp[0]=0;
        tmp[1]=Pi;
    }else if(lt(pos.first-redundancy,0)&&lt(pos.second-redundancy,0)){//靠近原点
        tmp[0]=0; 
        tmp[1]=Pi/2;
    }else if(lt(pos.first-redundancy,0)&&gt(pos.second-redundancy,0)){//只靠近左方的y轴
        tmp[0]=-Pi/2;
        tmp[1]=Pi/2;
    }else if(lt(pos.first-redundancy,0)&&gt(pos.second+redundancy,50)){//靠近左上角
        tmp[0]=-Pi/2;
        tmp[1]=0;
    }else if(gt(pos.first-redundancy,0)&&gt(pos.second+redundancy,50)){////靠近上方的x轴
    
        tmp[0]=-Pi;
        tmp[1]=0;
    }else if(gt(pos.first+redundancy,50)&&gt(pos.second+redundancy,50)){//靠近右上角
        tmp[0]=-Pi;
        tmp[1]=-Pi/2;
    }else if(gt(pos.first+redundancy,50)&&gt(pos.second-redundancy,0)){//靠近右边的y轴
        tmp[0]=Pi/2;
        tmp[1]=Pi;
        tmp.push_back(-Pi);
        tmp.push_back(-Pi/2);
    }else if(gt(pos.first+redundancy,50)&&lt(pos.second-redundancy,0)){//靠近右下角
        tmp[0]=Pi/2;
        tmp[1]=Pi;
    }
    return tmp;    
}
vector<double>  get_T_limits(pair<double,double>pos,const Robot& robot,int ctr,double dis){
    double radius=robot.get_type==0? 0.45:0.53;
    double tmpA=robot.direction;
    int sign1=ge(robot.xy_pos.first,0)?1:-1;
    int sign2=ge(robot.xy_pos.second,0)?1:-1;
    dis=fabs(dis);
    pos.first+=dis*cos(tmpA);
    pos.second+=dis*sin(tmpA);
    // if(state.FrameID>=591&&state.FrameID<=603&&robot.id==2){
    //     cerr<<"\n"<<robot.xy_pos.first<<" +  "<<robot.xy_pos.second<<"\n";
    //     cerr<<"\n"<<pos.first<<" "<<pos.second<<"\n";
    // }
    vector<double> tmp{-7,-7};
    double redundancy= (ctr==-1?0.2:dis)+radius;//冗余，避免频繁转向
    if(gt(pos.first-redundancy,0)&&lt(pos.second-redundancy,0)){//只靠近下方x轴
        tmp[0]=0;
        tmp[1]=Pi;
    }else if(lt(pos.first-redundancy,0)&&lt(pos.second-redundancy,0)){//靠近原点
        tmp[0]=0; 
        tmp[1]=Pi/2;
    }else if(lt(pos.first-redundancy,0)&&gt(pos.second-redundancy,0)){//只靠近左方的y轴
        tmp[0]=-Pi/2;
        tmp[1]=Pi/2;
    }else if(lt(pos.first-redundancy,0)&&gt(pos.second+redundancy,50)){//靠近左上角
        tmp[0]=-Pi/2;
        tmp[1]=0;
    }else if(gt(pos.first-redundancy,0)&&gt(pos.second+redundancy,50)){////靠近上方的x轴
    
        tmp[0]=-Pi;
        tmp[1]=0;
    }else if(gt(pos.first+redundancy,50)&&gt(pos.second+redundancy,50)){//靠近右上角
        tmp[0]=-Pi;
        tmp[1]=-Pi/2;
    }else if(gt(pos.first+redundancy,50)&&gt(pos.second-redundancy,0)){//靠近右边的y轴
        tmp[0]=Pi/2;
        tmp[1]=Pi;
        tmp.push_back(-Pi);
        tmp.push_back(-Pi/2);
    }else if(gt(pos.first+redundancy,50)&&lt(pos.second-redundancy,0)){//靠近右下角
        tmp[0]=Pi/2;
        tmp[1]=Pi;
    }
    return tmp;
}
bool can_stop(pair<double,double>p1,pair<double,double>p2,double angle,bool isWall,int ctr){
    if(gt(angle,Pi/2))return false;
    if(lt(angle,0.08))return true;
    double dis=calcuDis(p1,p2);
    if(lt(sin(angle)*dis,0.1)&&ctr==1){
        return true;
    }else if(lt(sin(angle)*dis,0.05)&&ctr==0){
        return true;
    }
    return false;

}
// double return_cal(pair<double,double>p1,pair<double,double>p2,double angle){
//     double dis=calcuDis(p1,p2);
//     // cerr<<dis<<"--"<<sin(angle)*dis<<"\n";
//     return sin(angle)*dis-0.4;
// }
bool is_range(double dire,vector<double>&tmp){
    if(tmp.size()==2){
        if(ge(dire,tmp[0])&&le(dire,tmp[1])){
            return true;
        }else{
            return false;
        }
    }else{
        if((ge(dire,tmp[0])&&le(dire,tmp[1]))||
        (ge(dire,tmp[2])&&le(dire,tmp[3]))){
            return true;
        }else{
            return false;
        } 
    }
    
}
double get_dis(pair<double, double> P, Line l) { 
    auto get_v=[&](pair<double, double> P,pair<double, double> v)->double{
        return P.first*v.second-v.first*P.second;
    };
    double distance=abs(get_v(P,l.v)-get_v(l.P,l.v))/(sqrt(l.v.first * l.v.first  + 
    l.v.second* l.v.second));
    return distance; 
}
// bool can_speed_z(int stuID,pair<double,double>xy_pos,pair<double,double>pos,double acceleration ){
//     Line line;
//     line.v=xy_pos;
//     line.P=pos;
//     double totalV=sqrt(xy_pos.first*xy_pos.first+xy_pos.second*xy_pos.second);//合速度
//     double dis1=get_dis(studios[stuID].pos,line);//点到直线的距离
//     double dis2=calcuDis(studios[stuID].pos,pos);//点之间的距离
//     double dis3=totalV*totalV/(2*acceleration);//速度减为0的滑行距离
//     double dis4=sqrt(0.4*0.4-dis1*dis1);//圆截线的长度
//     double dis5=sqrt(dis2*dis2-dis1*dis1);//射线的长度
//     //cerr<<stuID<<" "<<dis3<<" "<<dis4<<" "<<dis5<<" "<<dis1<<"\n";
//     double rudi=0.08;
//     if(class_map==2){
//         rudi=0.1;
//     }
//     // if(contr_print_flag&&state.FrameID>=1840&&state.FrameID<=1870&&stuID==17&&pos.second<4){
//     //             cerr<<"dis: \n";
//     //             cerr<<dis3<<" "<<dis5-dis4<<"\n";
//     // }
//     if(gt(dis3,(dis5-dis4)+rudi))return true;
//     return false;
// }
bool isWall(int stuID){
    double i=studios[stuID].pos.first;
    double j=studios[stuID].pos.second;
    if(i-1<=0||j-2<=0||i+2>=50||j+2>=50)return true;
    return false;
}
// bool isWall_r(const Robot& robot){
//     double i=robot.pos.first;
//     double j=robot.pos.second;
//     double ridr=0.8+(robot.get_type==0?0.6:0.63);
//     // if(state.FrameID>=1005&&state.FrameID<=1010&&robot.id==2){
//     //     cerr<<(j)<<"\n";
//     // }
//     if(le(i-ridr,0)||le(j-ridr,0)||ge(i+ridr,50)||ge(j+ridr,50))return true;
//     return false;   
// }
// bool will_impact(const Robot& robot,double dis){
//     vector<double> tmp=get_T_limits(robot.pos,robot,1,dis);
//     if(!eq(tmp[0],-7)&&(!is_range(robot.direction,tmp)))
//     {//在墙附件，并且会撞上
//         return true;
//     }
//     return false;
// }
// int special_test(int i1,int i2){
//     int cnt=5;
//     double base=0.02;
//     double radius=robots[i1].get_type==0? 0.45:0.53;
//     for(int i=1;i<=cnt;i++){
//         double time=i*base;
//         auto p1=make_pair<double,double>(robots[i1].pos.first+robots[i1].xy_pos.first*time,
//         robots[i1].pos.second+robots[i1].xy_pos.second*time
//         );
//         auto p2=make_pair<double,double>(robots[i2].pos.first+robots[i2].xy_pos.first*time,
//         robots[i2].pos.second+robots[i2].xy_pos.second*time
//         );
//         double dis=calcuDis(p1,p2);   
//         if(lt(dis,radius*2))return i;     
//     }
//     return 0;
// }
double get_angle(double s1,double s2){
    Vec v1(make_pair<double ,double>(cos(s1),sin(s1)));
    Vec v2(make_pair<double ,double>(cos(s2),sin(s2)));
    return (cos_t(v1,v2));
}
double get_angle(pair<double,double> p1,pair<double,double> p2){
    Vec v1(p1);
    Vec v2(p2);
    return cos_t(v1,v2);
}
// double get_angle_1(double s1,double s2){
//     Vec v1(make_pair<double ,double>(cos(s1),sin(s1)));
//     Vec v2(make_pair<double ,double>(cos(s2),sin(s2)));
//     return acos(cos_t(v1,v2));
// }
// double get_angle_1(pair<double,double> p1,pair<double,double> p2){
//     Vec v1(p1);
//     Vec v2(p2);
//     return acos(cos_t(v1,v2)) ;
// }
// bool is_less(int i1,int i2){
//         double base=0.02;
//         double time=2*base;
//         auto p1=make_pair<double,double>(robots[i1].pos.first+robots[i1].xy_pos.first*time,
//         robots[i1].pos.second+robots[i1].xy_pos.second*time
//         );
//         auto p2=make_pair<double,double>(robots[i2].pos.first+robots[i2].xy_pos.first*time,
//         robots[i2].pos.second+robots[i2].xy_pos.second*time
//         );
//         double dis=calcuDis(p1,p2);   

//         auto p3=make_pair<double,double>(robots[i1].pos.first+robots[i1].xy_pos.first,
//         robots[i1].pos.second+robots[i1].xy_pos.second
//         );
//         auto p4=make_pair<double,double>(robots[i2].pos.first+robots[i2].xy_pos.first,
//         robots[i2].pos.second+robots[i2].xy_pos.second
//         );
//         double dis1=calcuDis(p3,p4);   
    
//         return lt(dis,dis1);
// }
// bool who_isFirst(int i1,int i2){
//     pair<double, double> p1 = subVector(robots[i1].pos, robots[i2].pos);
//     double s1=robots[i1].direction;
//     pair<double, double> p2=make_pair<double ,double>(cos(s1),sin(s1));
//     double tmpCos=get_angle(p1,p2);
//     return gt(tmpCos,0.0);
// }
double return_v(int id){
    auto xy_pos=robots[id].xy_pos;
    return sqrt(xy_pos.first*xy_pos.first+xy_pos.second*xy_pos.second);//合速度
}
double return_v(Robot rob){
    auto robot=rob;
    return calVectorSize(robot.xy_pos) * (ge(calVectorProduct(robot.xy_pos, transformVector(robot.direction)), 0.0)? 1: -1);
}
// int Calculate_root(int i1,int i2){
//     double tmp= get_angle(robots[i1].xy_pos,robots[i2].xy_pos);
//     if(gt(tmp,0.9)&&!will_collision(i1,i2))return -1;
//     else return 0;
// }
// bool will_collision(int i1,int i2,int ctr){
//     Vec v1(robots[i1].xy_pos);
//     Vec v2(robots[i2].xy_pos);
//     Vec x1(robots[i1].pos);
//     Vec x2(robots[i2].pos);
//     Vec c_t=x1-x2;
//     Vec v_t=v1-v2;
//     bool f1=isWall(robots[i1].target_id);
//     bool f2=isWall(robots[i2].target_id);
//     double tmpDis=calcuDis(robots[i1].pos,robots[i2].pos);
//     if(gt(tmpDis,2)){
//             Compute_redundancy=1.0;
//     }else{
//             Compute_redundancy=0.0;
//         }
//     double r=getRobotRadius(i1)+getRobotRadius(i2)+Compute_redundancy;
//     double a=v_t*v_t;
//     double b=2*(v_t*c_t);
//     double c=c_t*c_t-r*r;
//     double cla=b*b-4*a*c;   
//     if(state.FrameID>=6830&&state.FrameID<=6840&&i1==1&&i2==2){

//         // cerr<<"will_c: "<<(-1*b+sqrt(cla))/(2*a)<<" "<<b<<" "<<(-1*b-sqrt(cla))/(2*a)<<" "<<cla<<"\n";
//         // cerr<<"dis: "<<tmpDis<<"  v "<<return_v(i1)<<" "<<return_v(i2)<<"\n";
//     }
//     pair<double ,double>tmp(-7,-7);
//     if(cla<0){
//         RootFlag=-1;
//         return false;
//     }
//     if(eq(cla,0)){
//         RootFlag=0;
//         tmp.first=-1*b/(2*a);
//         pair<double,double>pos_tmp_r1=robots[i1].pos;
//         pair<double,double>pos_tmp_r_c(pos_tmp_r1.first+robots[i1].xy_pos.first*tmp.first,
//         pos_tmp_r1.second+robots[i1].xy_pos.second*tmp.first
//         );
//         pair<double,double>pos_tmp_s1=studios[robots[i1].target_id].pos;
//         double dis1=calcuDis(pos_tmp_r1,pos_tmp_s1);
//         double dis3=calcuDis(pos_tmp_r1,pos_tmp_r_c);
//         Collision_point=pos_tmp_r_c;
//         pair<double,double>pos_tmp_r2=robots[i2].pos;
//         pair<double,double>pos_tmp_s2=studios[robots[i2].target_id].pos;
//         double dis2=calcuDis(pos_tmp_r2,pos_tmp_s2);
//         double dis4=calcuDis(pos_tmp_r2,pos_tmp_r_c);
//         Root=tmp;
//         if(!f1&&!f2){
//             if(gt(tmp.first,0.0)){
//                 if(lt(tmp.first,0.3))
//                 return true;
//             }else{
//             auto t1=Root;
//             auto t2=RootFlag;
//             if(will_collision_Careful(i1,i2)){
                
//                 return true;
//             }
//             Root=t1;
//             RootFlag=t2;
//             }
//         }
//         if(gt(tmp.first,0.0)){
//             if(le(dis3,dis1)&&le(dis4,dis2))return true;
//         }else{
//             auto t1=Root;
//             auto t2=RootFlag;
//             if(will_collision_Careful(i1,i2)){
                
//                 return true;
//             }
//             Root=t1;
//             RootFlag=t2;
//         }
        
//         return false;

//     }else if(gt(cla,0)){
//         RootFlag=1;
//         tmp.first=(-1*b+sqrt(cla))/(2*a);
//         tmp.second=(-1*b-sqrt(cla))/(2*a);
//         Root=tmp;
//         if(!f1&&!f2){
//             if(gt(tmp.first,0.0)){
//                 if(lt(tmp.first,0.3))
//                 return true;
//             }else{
//                 auto t1=Root;
//                 auto t2=RootFlag;
//                 if(will_collision_Careful(i1,i2)){
                    
//                     return true;
//                 }
//                 Root=t1;
//                 RootFlag=t2;
//             }
//             if(gt(tmp.second,0.0)){
//                 if(lt(tmp.second,0.3))
//                 return true;
//             }else{
//                 auto t1=Root;
//                 auto t2=RootFlag;
//                 if(will_collision_Careful(i1,i2)){
                    
//                     return true;
//                 }
//                 Root=t1;
//                 RootFlag=t2;
//             }
//         }
 
//         pair<double,double>pos_tmp_r1=robots[i1].pos;
//         pair<double,double>pos_tmp_r_c(pos_tmp_r1.first+robots[i1].xy_pos.first*tmp.first,
//         pos_tmp_r1.second+robots[i1].xy_pos.second*tmp.first
//         );
//         pair<double,double>pos_tmp_s1=studios[robots[i1].target_id].pos;
//         double dis1=calcuDis(pos_tmp_r1,pos_tmp_s1);
//         double dis3=calcuDis(pos_tmp_r1,pos_tmp_r_c);

//         pair<double,double>pos_tmp_r2=robots[i2].pos;
//         pair<double,double>pos_tmp_s2=studios[robots[i2].target_id].pos;
//         double dis2=calcuDis(pos_tmp_r2,pos_tmp_s2);
//         double dis4=calcuDis(pos_tmp_r2,pos_tmp_r_c);
//         // if(state.FrameID==5909&&i1==3&&i2==1){
//         //     cerr<<"---\n\n";
//         //     cerr<<robots[i1].pos.first<<"-"<<robots[i1].pos.second<<" "
//         //     <<robots[i2].pos.first<<"-"<<robots[i2].pos.second<<"\n";
//         //     cerr<<robots[i1].xy_pos.first<<"-"<<robots[i1].xy_pos.second<<" "
//         //     <<robots[i2].xy_pos.first<<"-"<<robots[i2].xy_pos.second<<"\n";
//         //     cerr<<v_t.x<<"-"<<v_t.y<<"\n";
//         //     cerr<<c_t.x<<"-"<<c_t.y<<"\n";
//         //     cerr<<a<<"-"<<b<<"-"<<c<<" "<<sqrt(cla)<<" "<<(-1*b+sqrt(cla))<<" "<<
//         //     (-1*b+sqrt(cla))/(2*a)<<"\n";
//         //     cerr<<tmp.first<<"-"<<tmp.second<<"\n";
//         //     cerr<<dis1<<" "<<dis2<<" "<<dis3<<" "<<dis4<<" "<<pos_tmp_r_c.first<<"-"<<pos_tmp_r_c.second<<"\n";
//         //     cerr<<"---\n\n";
//         // }
//         Collision_point=pos_tmp_r_c;

//         if(gt(tmp.first,0.0)){
//             if(le(dis3,dis1)&&le(dis4,dis2))return true;
//         }else{
//             auto t1=Root;
//             auto t2=RootFlag;
//             if(will_collision_Careful(i1,i2)){
                
//                 return true;
//             }
//             Root=t1;
//             RootFlag=t2;
//         }
        

//         tmp.second=(-1*b-sqrt(cla))/(2*a);
        
//         pair<double,double>pos_tmp_r_c1(pos_tmp_r1.first+robots[i1].xy_pos.first*tmp.second,
//         pos_tmp_r1.second+robots[i1].xy_pos.second*tmp.second
//         );
//         double dis11=calcuDis(pos_tmp_r1,pos_tmp_s1);
//         double dis31=calcuDis(pos_tmp_r1,pos_tmp_r_c1);


//         double dis21=calcuDis(pos_tmp_r2,pos_tmp_s2);
//         double dis41=calcuDis(pos_tmp_r2,pos_tmp_r_c1);
//         Collision_point=pos_tmp_r_c1;
//         if(gt(tmp.second,0.0)){
//             if(le(dis31,dis11)&&le(dis41,dis21))return true;
//         }else{
//             auto t1=Root;
//             auto t2=RootFlag;
//             if(will_collision_Careful(i1,i2)){
                
//                 return true;
//             }
//             Root=t1;
//             RootFlag=t2;
//         }
        
//         return false;       
//     }
//     if(eq(a,0))return true;
//     return true;
// }
// bool return_collision(int i1,int i2){
//     return lt(robots[i1].collision_val_pre,robots[i1].collision_val)&&
//     lt(robots[i2].collision_val_pre,robots[i2].collision_val);
// }
// pair<int,int> far_away(int i1,int i2,int base1,int base2){
//     int sign1=payloads[i1].sign,sign2=payloads[i2].sign;
//     if(sign1*sign2<0) return pair<int,int> (sign1,sign2);
//     else{
//         if(gt(fabs(payloads[i1].angle)-fabs(payloads[i2].angle) ,2)||robots[i1].get_type>(robots[i2].get_type)){
//             return pair<int,int> (sign1,-1*sign2);
//         }
//         return pair<int,int> (-1*sign1,sign2);
//     }
//     double dis = calcuDis(robots[i1].pos, robots[i2].pos);
//     int arr[][2]{{-1*base1,1*base2},{1*base1,1*base2},{-1*base1,-1*base2},{1*base1,-1*base2}};
//     double time=0.02;
//     pair<int,int>tmp(0,0);
//     pair<double,double>pre_xy1=robots[i1].xy_pos;
//     pair<double,double>pre_xy2=robots[i2].xy_pos;
//     double mmax=0.0;
//     int pos=0;
//     for(int i=0;i<4;i++){
//         Vec v1(make_pair<double ,double>(cos(Pi/2),sin(Pi/2)));
//         Vec v2(make_pair<double ,double>(cos(Pi/2),sin(Pi/2)));
//         auto p1=make_pair<double,double>(robots[i1].pos.first+robots[i1].xy_pos.first+(time*v1.x*arr[i][0]),
//         robots[i1].pos.second+robots[i1].xy_pos.second+(time*v1.x*arr[i][0])
//         );
//         auto p2=make_pair<double,double>(robots[i2].pos.first+robots[i2].xy_pos.first+(time*v2.x*arr[i][1]),
//         robots[i2].pos.second+robots[i2].xy_pos.second+(time*v2.x*arr[i][1])
//         );
//         double dis=calcuDis(p1,p2);    
//         if(gt(dis,mmax)){
//             mmax=dis;
//             pos=i;
//         }

//     }
//     tmp.first=arr[pos][0];
//     tmp.second=arr[pos][1];
//     return tmp;
// }
// double return_maxAng(int id1){
//     double dis=calcuDis(robots[id1].pos,studios[robots[id1].target_id].pos);
//     if(lt(dis,0.4))return Pi;
//     return asin(0.4/dis);
// }
// bool Check_for_balls_around(int pos){
//     for(int i=0;i<4;i++){
//         double dis = calcuDis(robots[i].pos, robots[pos].pos);
//         if(gt(dis,3))return false;
       
//     }
//     return true;
// }
// int return_line_dire(int i1,int i2,int signBase){
    
//     int try_aginF=0;
//     int tarId1=robots[i1].target_id==-1?0:robots[i1].target_id;
//     int tarId2=robots[i2].target_id==-1?0:robots[i2].target_id;
//     bool l1=can_stop(robots[i1].pos,studios[robots[i1].target_id].pos,payloads [i1].angle,isWall(tarId1));
//     bool l2=can_stop(robots[i2].pos,studios[robots[i2].target_id].pos,payloads [i2].angle,isWall(tarId2));
//     double tmpDis=calcuDis(robots[i1].pos,robots[i2].pos);
//     // if(l1&&l2)
//     // will_collision(i1,i2,0);
//     // else
//     will_collision(i1,i2);
//     try_agin:
//     int flagSign=getSign(i1,i2);
//     // double canAngle=min(fabs(Root.first),fabs(Root.second))*40*0.3;
//     // double stop_time= (fabs(robots[i2].angular_velocity))/(payloads[i2].angular_acceleration);
//     // double subVal=stop_time*40*0.36;
//     double real_time=-8;
//     // if(gt(tmpDis,2))
//     real_time=will_Collo_new(i1,i2);
//     // else
//     // real_time=min(fabs(gt(Root.first,0)?Root.first:7),fabs(gt(Root.second,0)?Root.second:7));
//     // if(lt(real_time,0)){
//     //     return 0;
//     // }
//     double canAngle_neg=get_at_v(real_time,payloads[i2].angular_acceleration
//     ,robots[i2].angular_velocity,-1);
//     double canAngle_pos=get_at_v(real_time,payloads[i2].angular_acceleration
//     ,robots[i2].angular_velocity,1);
//     auto tmp= subVector(robots[i1].pos, robots[i2].pos);
//     Vec v2(robots[i2].xy_pos); 
//     Vec v1(tmp);
//     int sign= (lt(v1^v2,0))?-1:1;
//     auto angle= return_seta(i1,i2); 
//     double seta=angle.first;
//     double arf=angle.second;
//     double canAngle_pos_z=get_at_v_z(real_time,payloads[i2].angular_acceleration
//     ,robots[i2].angular_velocity,sign)*-1;
//     double canAngle_neg_z=get_at_v_z(real_time,payloads[i2].angular_acceleration
//     ,robots[i2].angular_velocity,sign*-1)*-1;
//     // if(lt(canAngle_neg,0.0)){
//     //     cerr<<"----------+ "<<canAngle_neg<<" "<<canAngle_pos<<" "<<
//     //      payloads[i2].angular_acceleration<<" "<<sign<<"\n"; 
//     //      cerr<<robots[i2].angular_velocity<<" "<<real_time<< endl;
//     // }
//     // if(state.FrameID>=2048&&state.FrameID<=2500&&i1==2&&i2==1){
//     //     cerr<<"Frame: "<<state.FrameID<<" "<<canAngle_neg_z<<" "<<canAngle_pos_z<<" "<<arf<<
//     //     " "<<robots[i2].angular_velocity<< " "<<sign<<"\n";
       
//     // }
//     double v_1=min(return_v(i1),ins[i1].forward);
//     double v_2=min(return_v(i2),ins[i2].forward);
//     int total=robots[i1].get_type+robots[i2].get_type;
//     if(flagSign==1){
//         bool f1=false,f2=false;
//         if(gt(sign*-1==-1?canAngle_neg:canAngle_pos,seta+arf)){
//             f2=true;
//             // return sign*-1;
//         }else if(lt(Pi-seta-arf,sign==-1?canAngle_neg:canAngle_pos)){
//             f1=true;
//             // return sign;
//         }else if(lt(fabs(Pi-seta-arf)+canAngle_pos_z,fabs(seta+arf)+canAngle_neg_z)){
//             // cerr<<"can't raote -"<<state.FrameID<<" "<<i1<<" "<<i2<<"\n";
//             // cerr<<"can't raote  angle -"<<canAngle_pos_z<<" "<<canAngle_neg_z<<" "<<sign<<"\n";
//             //ins[i1].rotate=Pi*-1*sign;
//             // cerr<<"0\n";
//             return sign;
//         }else{
//             // cerr<<"can't raote -"<<state.FrameID<<" "<<i1<<" "<<i2<<"\n";
//             // cerr<<seta<<" "<<arf<<"\n";
//             // cerr<<"can't raote  angle -"<<canAngle_pos_z<<" "<<canAngle_neg_z<<" "<<sign*-1<<"\n";
//             // cerr<<"1\n";
//             return sign*-1;
//         }
//         if(f1&&f2){
//             // return signBase;
//             if(lt(fabs(Pi-seta-arf)+canAngle_pos_z,fabs(seta+arf)+canAngle_neg_z)){
//                 return sign;
//             }else{
//                 return sign*-1;
//             }
//         }else if(f1){
//             return sign;
//         }else{
//             return sign*-1;
//         }
//     }else{
//         bool f1=false,f2=false;
//          if(gt(sign==-1?canAngle_neg:canAngle_pos,seta-arf)){
//             f1=true;
//             // return sign;
//         }else if(gt(sign*-1==-1?canAngle_neg:canAngle_pos,Pi-seta+arf)){
//             f2=true;
//             // return sign*-1;
//         }else if(lt(fabs(seta-arf)+canAngle_pos_z,fabs(Pi-seta+arf)+canAngle_neg_z)){
//             // cerr<<"can't raote "<<state.FrameID<<" "<<i1<<" "<<i2<<"\n";
//             // cerr<<"can't raote  angle "<<canAngle_pos_z<<" "<<canAngle_neg_z<<" "<<sign<<"\n";
//             return sign;
//         }else{
//             // cerr<<"can't raote "<<state.FrameID<<" "<<i1<<" "<<i2<<"\n";
//             // cerr<<"can't raote  angle "<<canAngle_pos_z<<" "<<canAngle_neg_z<<" "<<sign*-1<<"\n";
//             return sign*-1;
//         }  
//         if(f1&&f2){
//             // return signBase;
//             if(lt(fabs(seta-arf)+canAngle_pos_z,fabs(Pi-seta+arf)+canAngle_neg_z)){
//                 return sign;
//             }else{
//                 return sign*-1;
//             }
//         }else if(f1){
//             return sign;
//         }else{
//             return sign*-1;
//         }
//     } 
// }
// int return_line_dire(int i1,int i2){
//     will_collision(i1,i2);
//     auto tmp= subVector(robots[i1].pos, robots[i2].pos);
//     Vec v1(tmp);
//     Vec v2(robots[i2].xy_pos);
//     int sign= (lt(v1^v2,0))?-1:1;
    
//     return sign;
// }
// pair<double,bool>  return_int_dis(int base){
//     vector<int>arr(2);
//     int pos=0;
//     for(int i=0;i<4;i++){
//         if((base>>i)&1){
//             arr[pos++]=i;
//         }
//     }
//     if(base==12&&calcuDis(robots[arr[0]].pos,robots[arr[1]].pos)<2&&!will_collision(arr[0],arr[1])){
//                 // cerr<<"% "<<RootFlag<<" "<< Root.first<<"-"<<Root.second<<
//                 // " "<<Collision_point.first<<"-"<<Collision_point.second<< endl;
//     }
//     return pair<double,int>(calcuDis(robots[arr[0]].pos,robots[arr[1]].pos),
//     will_collision(arr[0],arr[1])
//     );
// }




// double get_rotation(int i1,int i2){
//     Vec v1(pair<double,double>(cos(robots[i1].direction),sin(robots[i1].direction)));
//     Vec v2(subVector(robots[i1].pos, robots[i2].pos));
//     return acos(cos_t(v1,v2));
// }
// int addSign(int i1,int i2,int baseSign){
//     Vec v1(pair<double,double>(cos(robots[i1].direction),sin(robots[i1].direction)));
//     Vec v2(subVector(robots[i1].pos, robots[i2].pos)); 
//     int sign= (lt(v1^v2,0))?-1:1;
//     return gt(sign*baseSign,0.0)?-1:1;  
// }
// int getSign(int i1,int i2){
//     auto tmp= subVector(robots[i1].pos, robots[i2].pos);
//     Vec v1(tmp);
//     Vec v2(robots[i2].xy_pos);
//     int sign1=(lt(v1^v2,0))?-1:1;
//     auto tmp1= subVector(robots[i2].pos, robots[i1].pos);
//     Vec v3(tmp1);
//     Vec v4(robots[i1].xy_pos);

//     int sign2=(lt(v3^v4,0))?-1:1;
//     return gt(sign1*sign2,0.0)?-1:1;
// }
// pair<double,double> return_seta(int i1,int i2){
//     auto tmp= subVector(robots[i1].pos, robots[i2].pos);
//     Vec v1(tmp);
//     Vec v2(robots[i2].xy_pos);
//     int sign1=(lt(v1^v2,0))?-1:1;
//     auto tmp1= subVector(robots[i2].pos, robots[i1].pos);
//     Vec v3(tmp1);
//     Vec v4(robots[i1].xy_pos);
//     double tmpAngle=acos(cos_t(v1,v2));
//     double tmpAngle1=acos(cos_t(v3,v4));
//     return pair<double,double>(tmpAngle,tmpAngle1);
// }
// bool will_collision_Careful(int i1,int i2){
//     Vec v1(robots[i1].xy_pos);
//     Vec v2(robots[i2].xy_pos);
//     Vec x1(robots[i1].pos);
//     Vec x2(robots[i2].pos);
//     Vec c_t=x1-x2;
//     Vec v_t=v1-v2;
//     bool f1=isWall(robots[i1].target_id);
//     bool f2=isWall(robots[i2].target_id);
//     double tmpDis=calcuDis(robots[i1].pos,robots[i2].pos);

//     double r=getRobotRadius(i1)+getRobotRadius(i2);
//     double a=v_t*v_t;
//     double b=2*(v_t*c_t);
//     double c=c_t*c_t-r*r;
//     double cla=b*b-4*a*c;   
//     pair<double ,double>tmp(-7,-7);
//     if(cla<0){
//         RootFlag=-1;
//         return false;
//     }
//         if(state.FrameID>=6830&&state.FrameID<=6840&&i1==1&&i2==2){

//         // cerr<<"will_c_c: "<<(-1*b+sqrt(cla))/(2*a)<<" "<<b<<" "<<(-1*b-sqrt(cla))/(2*a)<<" "<<cla<<"\n";
//         // cerr<<"dis: "<<tmpDis<<"  v "<<return_v(i1)<<" "<<return_v(i2)<<"\n";
//     }
//     if(eq(cla,0)){
//         RootFlag=0;
//         tmp.first=-1*b/(2*a);
//         pair<double,double>pos_tmp_r1=robots[i1].pos;
//         pair<double,double>pos_tmp_r_c(pos_tmp_r1.first+robots[i1].xy_pos.first*tmp.first,
//         pos_tmp_r1.second+robots[i1].xy_pos.second*tmp.first
//         );
//         pair<double,double>pos_tmp_s1=studios[robots[i1].target_id].pos;
//         double dis1=calcuDis(pos_tmp_r1,pos_tmp_s1);
//         double dis3=calcuDis(pos_tmp_r1,pos_tmp_r_c);
//         Collision_point=pos_tmp_r_c;
//         pair<double,double>pos_tmp_r2=robots[i2].pos;
//         pair<double,double>pos_tmp_s2=studios[robots[i2].target_id].pos;
//         double dis2=calcuDis(pos_tmp_r2,pos_tmp_s2);
//         double dis4=calcuDis(pos_tmp_r2,pos_tmp_r_c);
//         Root=tmp;
//         if(!f1&&!f2){
//             if(gt(tmp.first,0.0)&&lt(tmp.first,0.3))return true;
//         }
//         if(gt(tmp.first,0.0))
//         if(le(dis3,dis1)&&le(dis4,dis2))return true;
//         return false;

//     }else if(gt(cla,0)){
//         RootFlag=1;
//         tmp.first=(-1*b+sqrt(cla))/(2*a);
//         tmp.second=(-1*b-sqrt(cla))/(2*a);
//         Root=tmp;
//         if(!f1&&!f2){
//             if(gt(tmp.first,0.0)&&lt(tmp.first,0.3))return true;
//             if(gt(tmp.second,0.0)&&lt(tmp.second,0.3))return true;
//         }
//         pair<double,double>pos_tmp_r1=robots[i1].pos;
//         pair<double,double>pos_tmp_r_c(pos_tmp_r1.first+robots[i1].xy_pos.first*tmp.first,
//         pos_tmp_r1.second+robots[i1].xy_pos.second*tmp.first
//         );
//         pair<double,double>pos_tmp_s1=studios[robots[i1].target_id].pos;
//         double dis1=calcuDis(pos_tmp_r1,pos_tmp_s1);
//         double dis3=calcuDis(pos_tmp_r1,pos_tmp_r_c);

//         pair<double,double>pos_tmp_r2=robots[i2].pos;
//         pair<double,double>pos_tmp_s2=studios[robots[i2].target_id].pos;
//         double dis2=calcuDis(pos_tmp_r2,pos_tmp_s2);
//         double dis4=calcuDis(pos_tmp_r2,pos_tmp_r_c);
//         // if(state.FrameID==1354&&i1==0&&i2==3){
//         //     cerr<<"---\n\n";
//         //     cerr<<robots[i1].pos.first<<"-"<<robots[i1].pos.second<<" "
//         //     <<robots[i2].pos.first<<"-"<<robots[i2].pos.second<<"\n";
//         //     cerr<<robots[i1].xy_pos.first<<"-"<<robots[i1].xy_pos.second<<" "
//         //     <<robots[i2].xy_pos.first<<"-"<<robots[i2].xy_pos.second<<"\n";
//         //     cerr<<v_t.x<<"-"<<v_t.y<<"\n";
//         //     cerr<<c_t.x<<"-"<<c_t.y<<"\n";
//         //     cerr<<a<<"-"<<b<<"-"<<c<<" "<<sqrt(cla)<<" "<<(-1*b+sqrt(cla))<<" "<<
//         //     (-1*b+sqrt(cla))/(2*a)<<"\n";
//         //     cerr<<tmp.first<<"-"<<tmp.second<<"\n";
//         //     cerr<<dis1<<" "<<dis2<<" "<<dis3<<" "<<dis4<<" "<<pos_tmp_r_c.first<<"-"<<pos_tmp_r_c.second<<"\n";
//         //     cerr<<"---\n\n";
//         // }
//         Collision_point=pos_tmp_r_c;

//         if(gt(tmp.first,0.0))
//         if(le(dis3,dis1)&&le(dis4,dis2))return true;

//         tmp.second=(-1*b-sqrt(cla))/(2*a);
        
//         pair<double,double>pos_tmp_r_c1(pos_tmp_r1.first+robots[i1].xy_pos.first*tmp.second,
//         pos_tmp_r1.second+robots[i1].xy_pos.second*tmp.second
//         );
//         double dis11=calcuDis(pos_tmp_r1,pos_tmp_s1);
//         double dis31=calcuDis(pos_tmp_r1,pos_tmp_r_c1);


//         double dis21=calcuDis(pos_tmp_r2,pos_tmp_s2);
//         double dis41=calcuDis(pos_tmp_r2,pos_tmp_r_c1);
//         Collision_point=pos_tmp_r_c1;
//         if(gt(tmp.second,0.0))
//         if(le(dis31,dis11)&&le(dis41,dis21))return true;
//         return false;       
//     }
//     if(eq(a,0))return true;
//     return true;    
// }
// double return_type(int i1){
//     return robots[i1].collision_val*robots[i1].time_val;
// }
// void change_getType(){
//     // for(int i=0;i<4;i++){
//     //     double val=(eq(robots[i].collision_val,0)?1:robots[i].collision_val)*(eq(robots[i].time_val,0)?1:robots[i].time_val);
//     //     if(lt(val,0.8))
//     //     robots[i].get_type=0;
//     // }
// }
double return_ac(double a,double v,double v1){
    int si1=ge(v,0)?1:-1;
    int si2=ge(v1,0)?1:-1;
    if(eq(v,0)){
        return si2*a;
    }
  
    //a*=lt(si1*si2,0)||gt(si1*si2,0)&&gt(fabs(v),fabs(v1));
    // if(state.FrameID==2962){
    //     cerr<<v<<":"<<si1<<"\n";
    //     cerr<<v1<<":"<<si2<<"\n";
    //     cerr<<(eq(v,v1))<<"\n";
    // }
    if(eq(v,v1)){
        return 0;
    }if(lt(si1*si2,0)){
        a*=si2;
    }else if(gt(si1*si2,0)&&gt(fabs(v),fabs(v1))){
        a*=si2*-1;
    }else{
        a*=si2;
    }
    return a;    
}
double get_at_v_limt(double t,double a,double v,double v1,int sign_v1){
    double lef_time=0;
    double s=0;
    int si1=ge(v,0)?1:-1;
    int si2=ge(v1,0)?1:-1;
    // a*=lt(si1*si2,0)||gt(si1*si2,0)&&gt(fabs(v),fabs(v1));
    if(lt(si1*si2,0)){
        a*=si2;
    }else if(gt(si1*si2,0)&&gt(fabs(v),fabs(v1))){
        a*=si2*-1;
    }else{
        a*=si2;
    }
   
    double tmpTime=(v1-v)/(a);
    double realTime=min(tmpTime,t);
    s=v*realTime+a*realTime*realTime;
    // if(state.FrameID==1){
    //     cerr<<a<<" ^ "<<" "<<v<<" "<<tmpTime<<" "<<realTime<<" "<<s<<"\n";
    // }
    double res=(s);
    if(le(t,tmpTime)){
        if(gt(res*sign_v1,0)){
            return fabs(res);
        }
        else{
            return -1*fabs(res);
        }
            return (s);
    }
    t=t-realTime;
    res=(s+v1*t);
    if(gt(res*sign_v1,0)){
        return fabs(res);
    }else{
        return -1*fabs(res);
    }
    return (s+v1*t);    
}
// double get_at_v(double t,double a,double v,int sign_v1){
//     double lef_time=0;
//     double s=0;
//     a*=sign_v1;
//     if(lt(fabs(v),Pi)){
//         double tmpTime=(sign_v1*Pi-v)/(a);
//         double realTime=min(tmpTime,t);
//         s=v*realTime+0.5*a*realTime*realTime;
//         double res=(s);
//         if(le(t,tmpTime)){
//             if(gt(res*sign_v1,0)){
//                 return fabs(res);
//             }
//             else{
//                 return -1*fabs(res);
//             }
//             return (s);
//         }
//         t=t-realTime;
//     }
//     double res=(s+sign_v1*Pi*t);
//     if(gt(res*sign_v1,0)){
//         return fabs(res);
//     }else{
//         return -1*fabs(res);
//     }
//     return (s+sign_v1*Pi*t);
// }
double get_at_stop(double t,double a,double v,int sign_v1){
    double lef_time=0;
    double s=0;
    int si1=ge(v,0)?1:-1;
    int si2=ge(0,0)?1:-1;
    // a*=lt(si1*si2,0)||gt(si1*si2,0)&&gt(fabs(v),fabs(v1));
    if(lt(si1*si2,0)){
        a*=si2;
    }else if(gt(si1*si2,0)&&gt(fabs(v),fabs(0))){
        a*=si2*-1;
    }else{
        a*=si2;
    }
    // if(gt(a*sign_v1,0))a*=sign_v1;
    double tmpTime=(0-v)/(a);
    double realTime=tmpTime;
    s=v*realTime+0.5*a*realTime*realTime;
    double res=(s);
    if(gt(res*sign_v1,0)){
        return fabs(res);
    }
    else{
        return -1*fabs(res);
    }
    return (s);
}
// double get_at_v_z(double t,double a,double v,int sign_v1){
//     double lef_time=0;
//     double s=0;
//     a*=sign_v1;
//     if(lt(fabs(v),Pi)){
//         double tmpTime=(sign_v1*Pi-v)/(a);
//         double realTime=min(tmpTime,t);
//         s=v*realTime+0.5*a*realTime*realTime;
//         double res=(s);
//         if(le(t,tmpTime)){
//             if(gt(res*sign_v1,0)){
//                 return fabs(res);
//             }
//             else{
//                 return -1*fabs(res);
//             }
//             return (s);
//         }
//         t=t-realTime;
//     }
//     double res=(s+sign_v1*Pi*t);
//     if(gt(res*sign_v1,0)){
//         return fabs(res);
//     }else{
//         return -1*fabs(res);
//     }
//     return (s+sign_v1*Pi*t);
// }
// bool is_near_tar(int id){
//     double tmpDis=calcuDis(robots[id].pos,robots[robots[id].target_id].pos);
//     if(lt(tmpDis,2))return true;
//     return false;
// }

bool checkNearBar(const pair<double,double> &a, double radius){
    int i, j;
    double x1, x2, y1, y2;
    double cross_y1, cross_y2, dis, radius2, dis1;
    int x_min, x_max, y_min, y_max;
    radius2 = radius * radius;
    x1 = a.first - radius;
    x2 = a.first + radius;

    x_min = (int)(x1 / 0.5);
    x_max = (int)(x2 / 0.5);

    for(i = x_min; i <= x_max + 1; ++i) {
        if(i == (int)(a.first / 0.5))
            dis1 = radius;
        else dis1 = min(fabs(a.first - (i + 1) * 0.5), fabs(a.first - i * 0.5));
        // 没有交点
        if(gt(dis1, radius)) continue;
        dis = sqrt(radius2 - dis1 * dis1);
        y1 = a.second - dis;
        y2 = a.second + dis;
        y_min = max(0, (int)(y1 / 0.5));
        y_max = min(100, (int)(y2 / 0.5));

        for(j = y_min; j <= y_max; ++j) {
            if(graph[j][i] == -2)
                return true;
        }
    }

    return false;
}

// vector<pair<double,double>>Calculate_the_trajectory(Robot rob,Ins ins_in, int forward_change, int rotate_change,const vector<pair<double,double>>&  tra,int cnt,int tar,double rob_dis,double pre_dis){
//     //Calculate_the_trajectory_2
//     // if(state.FrameID==4330&&state.FrameID==4330&&rob.id==0){
//     //         cerr<<"ins ^ : "<<forward_change<<"-"<<rotate_change<<" "<<tra.size()<<"\n";
//     //         cerr<<state.FrameID+cnt<<"\n";
//     // }
//     double t=0.02;
//     Robot tmp=rob;
//     rob.node_id = trans_pos_to_nodeID(rob.pos);
//     rob.cnt_tar = rob.node_id;
//     Ins ins=contr_one_rob(rob);
//     // cerr<<"kkk";
//     PayLoad pay=calPayload(rob,rob.virtual_pos);
//     //倒退中
//     if(forward_change && ins_in.forward == -2 && le(pay.speed, 0)) {
//         //更新virtual_pos
//         pay=calPayload(rob, trans_nodeID_to_pos(next_node[rob.target_id][(rob.get_type != 0)][rob.node_id]));
//         ins.rotate = pay.sign * Pi;
//     }

//     Flag_sumulate=0;
//     double w_next=ins.rotate;
//     double v_next=ins.forward;
//     if(forward_change==1){
//         v_next=ins_in.forward;
//     }
//     if(rotate_change==1){
//         w_next=ins_in.rotate;
//     }

    

//     if(cnt>tar||cnt>=tra.size()){
//         // if(forward_change == 0) {
//         //     return {};
//         // }
//         return {rob.pos};
//         // return {};
//     }
//     double tmpDis=calcuDis(rob.pos,tra[cnt]);        
//     // if(state.FrameID==3487&&rob.id==3 && ins_in.forward == -2){
//     //         // cerr<<"ins ^ : "<<forward_change<<"-"<<rotate_change<<"\n";
//     //         // cerr<<state.FrameID+cnt<<"\n";
//     //         cerr<<state.FrameID+cnt<<": "<<"epos1: "<<rob.pos.first<<"-"<<rob.pos.second<<" epos2:  "<<tra[cnt].first<<"-"<<tra[cnt].second<<"\n";
//     //         cerr<<tmpDis<<"--"<<pre_dis<<"\n";
//     // }
//     if(gt(tmpDis,pre_dis + 0.04)){
//         forward_change=0;
//         rotate_change=0;
//         // if(state.FrameID==849)
//         // {        
//         //     cerr<<"can inv: "<<state.FrameID+cnt<<"\n";
//         //     printRobotsDis(rob,tra[cnt]);
//         // }
//         // cerr<<state.FrameID+cnt<<": "<<"epos1: "<<rob.pos.first<<"-"<<rob.pos.second<<" epos2:  "<<tra[cnt].first<<"-"<<tra[cnt].second<<"\n";
//         // cerr<<tmpDis<<"--"<<pre_dis<<"\n";
//         return {rob.pos};
//     }
//     if(lt(tmpDis,rob_dis)){
//         // if(state.FrameID>=4330&&state.FrameID>=4360&&rob.id==0){
//         //     cerr<<"ins ^ : "<<forward_change<<"-"<<rotate_change<<"\n";
//         //     cerr<<state.FrameID+cnt<<"\n";
//         // }
//         // cerr<<"vispos1: "<<rob.pos.first<<"-"<<rob.pos.second<<" vispos2:  "<<tra[cnt].first<<"-"<<tra[cnt].second<<"\n";
//         // cerr<<"ins: "<<forward_change<<"-"<<rotate_change<<"\n";
//         // cerr<<calcuDis(rob.pos, tra[cnt])<<" "<<rob_dis<<"\n";
//         // if(state.FrameID==860)
//         // {        
//         //     cerr<<"collo: "<<state.FrameID+cnt<<"\n";
//         //     printRobotsDis(rob,tra[cnt]);
//         // }
//         new_cllo_time=cnt*0.02;
//         return {};
//     }
//     // 撞障碍物，返回空
//     if(checkNearBar(rob.pos, rob.radius)){
//         // printPair(rob.pos);
//         // cerr<<"撞障碍物\n";
//         return {};
//     }
//     cnt++;
    
//     double seta=rob.direction;
//     double w=rob.angular_velocity;
//     double a=return_ac(pay.angular_acceleration,rob.angular_velocity,w_next);
//     double changeAngle=get_at_v_limt(t,pay.angular_acceleration,rob.angular_velocity,w_next,pay.sign)*pay.sign;
//     double v=Calculate_the_projection_speed(rob);
//     double a_v=return_ac(pay.acceleration,v,v_next);
//     rob.pos.first=rob.pos.first+v*cos(seta+changeAngle/2)*t;
//     rob.pos.second=rob.pos.second+v*sin(seta+changeAngle/2)*t;
//     int sign1=ge((rob.angular_velocity+a*t)*w_next,0)?1:-1;
//     int sign2=ge((rob.angular_velocity+a*t),0)?1:-1;
//     double limit_w=0.0;
//     if(lt(a,0)){
//         limit_w=lt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
//     }else{
//         limit_w=gt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
//     }
//     // if(state.FrameID==1){
//     //     cerr<<cnt+1<<" - "<<rob.angular_velocity+a*t<<" "<<w_next<<" "
//     //     <<a<<" "<<changeAngle<<"\n";
//     // }
//     rob.angular_velocity=limit_w;
    
//     // if(state.FrameID==1)cerr<<cnt-1<<" "<<changeAngle<<" "<<rob.direction<<"\n";
//     // rob.xy_pos=return_change_v(w,changeAngle*pay.sign,rob.xy_pos);
//     int signv_1=ge((v+a_v*t)*v_next,0)?1:-1;
//     int signv_2=ge((v+a_v*t),0)?1:-1;
//     double limit_v=gt(fabs(v+a_v*t),fabs(6))?6*sign2:v+a_v*t;
//     if(lt(a_v,0)){
//         limit_v=lt(v+a_v*t,v_next)?v_next:v+a_v*t;
//     }else{
//         limit_v=gt(v+a_v*t,v_next)?v_next:v+a_v*t;
//     }

//     v=limit_v;
//     // rob.direction+=changeAngle;
//     // if(state.FrameID==2962&&rob.id==3){
//     //     cerr<<pay.speed<<" "<<v<<" ^ "<<changeAngle<<" "<<v_next<<" "<<a_v<< endl;
//     //     cerr<<rob.xy_pos.first<<"-"<<rob.xy_pos.second<<"\n";
//     // }
//     double xy_angle=get_Angle_xy(rob);
//     rob.xy_pos.first=v*cos(xy_angle);
//     rob.xy_pos.second=v*sin(xy_angle);
//     double xy_angle_next=get_Angle_xy(rob);
//     double cal_angle=xy_angle_next-xy_angle;
//     vector<vector<double>>mat(4,vector<double>(4,0));
//     cal_matrix(mat,changeAngle,cal_angle);

//     // if(state.FrameID==1800){
//     //     cerr<<"v "<<v<<" old: "<<rob.xy_pos.first<<"-"<<rob.xy_pos.second<<" "<<xy_angle<<" "<<rob.direction<<"-"<< Calculate_the_projection_speed(rob)<< endl;
//     // }
//     rob.direction+=changeAngle;
//     rob.direction=rob.direction>Pi?rob.direction-2*Pi:rob.direction; 
//     // if(rob.direction>Pi)changeAngle=2*Pi-changeAngle;
//     double t1=rob.xy_pos.first,t2=rob.xy_pos.second;
//     rob.xy_pos.first=(t1*mat[0][0]+t2*mat[0][1]);
//     rob.xy_pos.second=(t1*mat[1][0]+t2*mat[1][1]);
//     // rob.xy_pos.first=(t1*cos(changeAngle+cal_angle)-t2*sin(changeAngle+cal_angle));
//     // rob.xy_pos.second=(t1*sin(changeAngle+cal_angle)+t2*cos(changeAngle+cal_angle));
//     //     cerr<<rob.xy_pos.first<<"-"<<rob.xy_pos.second<<"\n";
//     // }
//     // rob.xy_pos.second=v_tmp.y;
//     // if(Flag_sumulate){
//     //     return {rob.pos};
//     // }
//     // if(Flag_sumulate){
//     //     return {rob.pos};
//     // }
    
//     auto res=Calculate_the_trajectory(rob,ins_in,forward_change,rotate_change,tra,cnt,tar,rob_dis,tmpDis);

//     if(res.size()>0)
//         res.push_back(tmp.pos);
//     if(cnt==1){
//         reverse(res.begin(),res.end());
//         rob = tmp;
//     }
//     return res;
// }

// vector<pair<double,double>>Calculate_the_trajectory(Robot rob,int cnt,int tar,int ctrF){
//     // cerr<<"aaaa"<<state.FrameID<<"\n";
//     double t=0.02;
//     Robot tmp=rob;
//     Ins ins=contr_one_rob(rob);
//     PayLoad pay=calPayload(rob,rob.virtual_pos);
//     double w_next=ins.rotate;
//     double v_next=ins.forward;
//     if(cnt>tar){
//         return {rob.pos};
//     }
//     // if(state.FrameID==1800){
//     //     cerr<<" Framid: "<<state.FrameID+cnt<<" tarID: "<<rob.target_id<<" robId: "<<rob.id<<" w_v: "<<rob.angular_velocity<<" dirc: "<<rob.direction
//     //     <<" pos_xy: "<<rob.pos.first<<"-"<<rob.pos.second<<" v_xy "<<rob.xy_pos.first<<"-"<<rob.xy_pos.second<<  endl;
//     //     cerr<<"v: "<<pay.speed<<"\n";
//     // }
//     cnt++;
//     double seta=rob.direction;
//     double w=rob.angular_velocity;
//     double a=return_ac(pay.angular_acceleration,rob.angular_velocity,w_next);
//     double changeAngle=get_at_v_limt(t,pay.angular_acceleration,rob.angular_velocity,w_next,pay.sign)*pay.sign;
//     // if(state.FrameID==1){
//     //     cerr<<changeAngle<<"\n";
//     // }
//     double v=Calculate_the_projection_speed(rob);
//     double a_v=return_ac(pay.acceleration,v,v_next);
//     rob.pos.first=rob.pos.first+v*cos(seta+changeAngle/2)*t;
//     rob.pos.second=rob.pos.second+v*sin(seta+changeAngle/2)*t;
//     int sign1=ge((rob.angular_velocity+a*t)*w_next,0)?1:-1;
//     int sign2=ge((rob.angular_velocity+a*t),0)?1:-1;
//     double limit_w=0.0;
//     if(lt(a,0)){
//         limit_w=lt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
//     }else{
//         limit_w=gt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
//     }
//     // if(state.FrameID==1){
//     //     cerr<<cnt+1<<" - "<<rob.angular_velocity+a*t<<" "<<w_next<<" "
//     //     <<a<<" "<<changeAngle<<"\n";
//     // }
//     rob.angular_velocity=limit_w;
    
//     // if(state.FrameID==1)cerr<<cnt-1<<" "<<changeAngle<<" "<<rob.direction<<"\n";
//     // rob.xy_pos=return_change_v(w,changeAngle*pay.sign,rob.xy_pos);
//     int signv_1=ge((v+a_v*t)*v_next,0)?1:-1;
//     int signv_2=ge((v+a_v*t),0)?1:-1;
//     double limit_v=gt(fabs(v+a_v*t),fabs(6))?6*sign2:v+a_v*t;
//     if(lt(a_v,0)){
//         limit_v=lt(v+a_v*t,v_next)?v_next:v+a_v*t;
//     }else{
//         limit_v=gt(v+a_v*t,v_next)?v_next:v+a_v*t;
//     }

//     v=limit_v;
//     // rob.direction+=changeAngle;
//     // if(state.FrameID==1){
//     //     cerr<<pay.speed<<" "<<v<<" ^ "<<changeAngle<<" "<<v_next<<" "<<a_v<< endl;
//     //     cerr<<rob.xy_pos.first<<"-"<<rob.xy_pos.second<<"\n";
//     // }
//     double xy_angle=get_Angle_xy(rob);
//     rob.xy_pos.first=v*cos(xy_angle);
//     rob.xy_pos.second=v*sin(xy_angle);
//     double xy_angle_next=get_Angle_xy(rob);
//     double cal_angle=xy_angle_next-xy_angle;
//     vector<vector<double>>mat(4,vector<double>(4,0));
//     cal_matrix(mat,changeAngle,cal_angle);

//     // if(state.FrameID==1800){
//     //     cerr<<"v "<<v<<" old: "<<rob.xy_pos.first<<"-"<<rob.xy_pos.second<<" "<<xy_angle<<" "<<rob.direction<<"-"<< Calculate_the_projection_speed(rob)<< endl;
//     // }
//     rob.direction+=changeAngle;
//     rob.direction=rob.direction>Pi?rob.direction-2*Pi:rob.direction; 
//     // if(rob.direction>Pi)changeAngle=2*Pi-changeAngle;
//     double t1=rob.xy_pos.first,t2=rob.xy_pos.second;
//     rob.xy_pos.first=(t1*mat[0][0]+t2*mat[0][1]);
//     rob.xy_pos.second=(t1*mat[1][0]+t2*mat[1][1]);
//     // rob.xy_pos.first=(t1*cos(changeAngle+cal_angle)-t2*sin(changeAngle+cal_angle));
//     // rob.xy_pos.second=(t1*sin(changeAngle+cal_angle)+t2*cos(changeAngle+cal_angle));
//     // if(state.FrameID==514&&rob.id==3){
//     //     cerr<<mat[0][0]<<"-"<<mat[0][1]<<"\n";
//     //     cerr<<mat[1][0]<<"-"<<mat[1][1]<<"\n";
//     // }
//     // rob.xy_pos.second=v_tmp.y;
//     if (Flag_sumulate && ctrF)
//     {
//         return {rob.pos};
//     }
//     auto res=Calculate_the_trajectory(rob,cnt,tar,ctrF);
//     res.push_back(tmp.pos);
//     if(cnt==1) {
//         reverse(res.begin(),res.end());
//         rob = tmp;
//     }
//     return res;
// }
vector<pair<double,double>>Calculate_the_trajectory(Robot rob,Ins ins_in, int forward_change, int rotate_change,const vector<pair<double,double>>&  tra,int cnt,int tar,double rob_dis,double pre_dis){
    //Calculate_the_trajectory_2
    // if(state.FrameID==4330&&state.FrameID==4330&&rob.id==0){
    //         cerr<<"ins ^ : "<<forward_change<<"-"<<rotate_change<<" "<<tra.size()<<"\n";
    //         cerr<<state.FrameID+cnt<<"\n";
    // }
    double t=0.02;
    pre_dis=100;
    vector<pair<double,double>> res;
    res.push_back(rob.pos);
    for( cnt=0;(cnt<=tar&&cnt<tra.size());cnt++){
        rob.node_id = trans_pos_to_nodeID(rob.pos);
        rob.cnt_tar = rob.node_id;
        Ins ins=contr_one_rob(rob);
        // cerr<<"kkk";
        PayLoad pay=calPayload(rob,rob.virtual_pos);
        //倒退中
        if(forward_change && ins_in.forward == -2 && le(pay.speed, 0)) {
            //更新virtual_pos
            pay=calPayload(rob, trans_nodeID_to_pos(next_node[rob.target_id][(rob.get_type != 0)][rob.node_id]));
            ins.rotate = pay.sign * Pi;
        }

        Flag_sumulate=0;
        double w_next=ins.rotate;
        double v_next=ins.forward;
        if(forward_change==1){
            v_next=ins_in.forward;
        }
        if(rotate_change==1){
            w_next=ins_in.rotate;
        }
        double tmpDis=calcuDis(rob.pos,tra[cnt]);    
        if(gt(tmpDis,pre_dis + 0.04)){
            forward_change=0;
            rotate_change=0;
            return res;
        // if(state.FrameID==849)
        // {        
        //     cerr<<"can inv: "<<state.FrameID+cnt<<"\n";
        //     printRobotsDis(rob,tra[cnt]);
        // }
        // cerr<<state.FrameID+cnt<<": "<<"epos1: "<<rob.pos.first<<"-"<<rob.pos.second<<" epos2:  "<<tra[cnt].first<<"-"<<tra[cnt].second<<"\n";
        // cerr<<tmpDis<<"--"<<pre_dis<<"\n";
        }
        if(lt(tmpDis,rob_dis)){
            new_cllo_time=cnt*0.02;
            return {};
        }
            // 撞障碍物，返回空
        if(checkNearBar(rob.pos, rob.radius)){
            // printPair(rob.pos);
            // cerr<<"撞障碍物\n";
            return {};
        }
        double seta=rob.direction;
        double w=rob.angular_velocity;
        double a=return_ac(pay.angular_acceleration,rob.angular_velocity,w_next);
        double changeAngle=get_at_v_limt(t,pay.angular_acceleration,rob.angular_velocity,w_next,pay.sign)*pay.sign;
        double v=Calculate_the_projection_speed(rob);
        double a_v=return_ac(pay.acceleration,v,v_next);
        rob.pos.first=rob.pos.first+v*cos(seta+changeAngle/2)*t;
        rob.pos.second=rob.pos.second+v*sin(seta+changeAngle/2)*t;
        int sign1=ge((rob.angular_velocity+a*t)*w_next,0)?1:-1;
        int sign2=ge((rob.angular_velocity+a*t),0)?1:-1;
        double limit_w=0.0;
        if(lt(a,0)){
            limit_w=lt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
        }else{
            limit_w=gt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
        }
        // if(state.FrameID==1){
        //     cerr<<cnt+1<<" - "<<rob.angular_velocity+a*t<<" "<<w_next<<" "
        //     <<a<<" "<<changeAngle<<"\n";
        // }
        rob.angular_velocity=limit_w;
        
        // if(state.FrameID==1)cerr<<cnt-1<<" "<<changeAngle<<" "<<rob.direction<<"\n";
        // rob.xy_pos=return_change_v(w,changeAngle*pay.sign,rob.xy_pos);
        int signv_1=ge((v+a_v*t)*v_next,0)?1:-1;
        int signv_2=ge((v+a_v*t),0)?1:-1;
        double limit_v=gt(fabs(v+a_v*t),fabs(6))?6*sign2:v+a_v*t;
        if(lt(a_v,0)){
            limit_v=lt(v+a_v*t,v_next)?v_next:v+a_v*t;
        }else{
            limit_v=gt(v+a_v*t,v_next)?v_next:v+a_v*t;
        }

        v=limit_v;
        // rob.direction+=changeAngle;
        // if(state.FrameID==2962&&rob.id==3){
        //     cerr<<pay.speed<<" "<<v<<" ^ "<<changeAngle<<" "<<v_next<<" "<<a_v<< endl;
        //     cerr<<rob.xy_pos.first<<"-"<<rob.xy_pos.second<<"\n";
        // }
        double xy_angle=get_Angle_xy(rob);
        rob.xy_pos.first=v*cos(xy_angle);
        rob.xy_pos.second=v*sin(xy_angle);
        double xy_angle_next=get_Angle_xy(rob);
        double cal_angle=xy_angle_next-xy_angle;
        vector<vector<double>>mat(4,vector<double>(4,0));
        cal_matrix(mat,changeAngle,cal_angle);

        // if(state.FrameID==1800){
        //     cerr<<"v "<<v<<" old: "<<rob.xy_pos.first<<"-"<<rob.xy_pos.second<<" "<<xy_angle<<" "<<rob.direction<<"-"<< Calculate_the_projection_speed(rob)<< endl;
        // }
        rob.direction+=changeAngle;
        rob.direction=rob.direction>Pi?rob.direction-2*Pi:rob.direction; 
        // if(rob.direction>Pi)changeAngle=2*Pi-changeAngle;
        double t1=rob.xy_pos.first,t2=rob.xy_pos.second;
        rob.xy_pos.first=(t1*mat[0][0]+t2*mat[0][1]);
        rob.xy_pos.second=(t1*mat[1][0]+t2*mat[1][1]);
        res.push_back(rob.pos);
        pre_dis=tmpDis;
    }

    return res;
}

vector<pair<double,double>>Calculate_the_trajectory(Robot rob,int cnt,int tar,int ctrF){
    // cerr<<"aaaa"<<state.FrameID<<"\n";
    double t=0.02;
    vector<pair<double,double>> res;
    res.push_back(rob.pos);
    for(cnt=0;(cnt<=tar);cnt++){
        Ins ins=contr_one_rob(rob);
        PayLoad pay=calPayload(rob,rob.virtual_pos);
        double w_next=ins.rotate;
        double v_next=ins.forward;
        double seta=rob.direction;
        double w=rob.angular_velocity;
        double a=return_ac(pay.angular_acceleration,rob.angular_velocity,w_next);
        double changeAngle=get_at_v_limt(t,pay.angular_acceleration,rob.angular_velocity,w_next,pay.sign)*pay.sign;
        double v=Calculate_the_projection_speed(rob);
        double a_v=return_ac(pay.acceleration,v,v_next);
        rob.pos.first=rob.pos.first+v*cos(seta+changeAngle/2)*t;
        rob.pos.second=rob.pos.second+v*sin(seta+changeAngle/2)*t;
        int sign1=ge((rob.angular_velocity+a*t)*w_next,0)?1:-1;
        int sign2=ge((rob.angular_velocity+a*t),0)?1:-1;
        double limit_w=0.0;
        if(lt(a,0)){
            limit_w=lt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
        }else{
            limit_w=gt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
        }
        rob.angular_velocity=limit_w;
    
        // if(state.FrameID==1)cerr<<cnt-1<<" "<<changeAngle<<" "<<rob.direction<<"\n";
        // rob.xy_pos=return_change_v(w,changeAngle*pay.sign,rob.xy_pos);
        int signv_1=ge((v+a_v*t)*v_next,0)?1:-1;
        int signv_2=ge((v+a_v*t),0)?1:-1;
        double limit_v=gt(fabs(v+a_v*t),fabs(6))?6*sign2:v+a_v*t;
        if(lt(a_v,0)){
            limit_v=lt(v+a_v*t,v_next)?v_next:v+a_v*t;
        }else{
            limit_v=gt(v+a_v*t,v_next)?v_next:v+a_v*t;
        }
        v=limit_v;
        double xy_angle=get_Angle_xy(rob);
        rob.xy_pos.first=v*cos(xy_angle);
        rob.xy_pos.second=v*sin(xy_angle);
        double xy_angle_next=get_Angle_xy(rob);
        double cal_angle=xy_angle_next-xy_angle;
        vector<vector<double>>mat(4,vector<double>(4,0));
        cal_matrix(mat,changeAngle,cal_angle);
        rob.direction+=changeAngle;
        rob.direction=rob.direction>Pi?rob.direction-2*Pi:rob.direction; 
        // if(rob.direction>Pi)changeAngle=2*Pi-changeAngle;
        double t1=rob.xy_pos.first,t2=rob.xy_pos.second;
        rob.xy_pos.first=(t1*mat[0][0]+t2*mat[0][1]);
        rob.xy_pos.second=(t1*mat[1][0]+t2*mat[1][1]);
        res.push_back(rob.pos);
    }
    
    return res;
}
// PayLoad calPayload_trajectory(Robot rob,int studioID){
//     Robot robot = rob;
//     Studio studio = studios[studioID];

//     // cerr << robotID << "--"<< robot.target_id<<"\n";

//     double distance = calcuDis(robot.pos, studio.pos);
//     double angular_acceleration = robot.get_type == 0? angular_acceleration_no :angular_acceleration_has;
//     double acceleration = robot.get_type == 0? acceleration_no: acceleration_has;

//     // 计算机器人与目标点构成的向量与x轴正方向夹角
//     pair<double, double> robotToStudio = subVector(studio.pos, robot.pos);
//     double angle1 = calAngle(robotToStudio);

//     double angle2 = ge(robot.direction, 0.0) ? robot.direction: 2 * Pi + robot.direction;
//     // double angle2 = calAngle(robot.xy_pos);

//     double angle = angle2 - angle1;

//     double speed = calVectorSize(robot.xy_pos) * (ge(calVectorProduct(robot.xy_pos, transformVector(robot.direction)), 0.0)? 1: -1);

//     int sign;

//     if(ge(angle, 0) && lt(angle, Pi) || lt(angle, -Pi))
//         sign = -1;
//     else
//         sign = 1;
//     angle = fabs(angle);
//     angle  = gt(angle, Pi)? 2 * Pi - angle: angle;


//     // cerr<<"**"<< angle1<<"**dir:"<<robot.direction<<"**"<<angle2<<"\n";
//     // cerr<<"**"<< angle << "**"<<distance<<"**"<<sign<<"\n";

//     return PayLoad((robot.get_type == 0? 0.45: 0.53), angle, angular_acceleration, acceleration, distance, speed, sign);    
// }
Ins contr_one_rob_1(Robot& robot){
    print_cerr_flag_ta=false;
    Flag_sumulate=0;
    Ins ins_t;
    int istake=robot.get_type==0?0:1;
    int print_rob_id=2;
    if(robot.target_id==-1){
        ins_t.forward=0;
        ins_t.rotate=Pi;
        return ins_t;
    }
    bool print_cerr_flag_ta1=false;
    // if( state.FrameID>1010&&state.FrameID<1289){
    //     // cerr<<robot.need_adjust_statues<<" rob "<<robot.id<<endl;
    //     print_cerr_flag_ta1=true;
    //     print_rob_id=2;
    // }
    adjust_virtual_pos_total(robot);
    PayLoad payload=calPayload(robot,robot.virtual_pos);
    auto p1=get_w_now(robot,payload);
    if(gt(return_v(robot),0.8)&&robot.need_slow&&robot.need_adjust_statues){
        ins_t.forward=0;
        ins_t.rotate=p1.first;
        if(robot.id==print_rob_id&&print_cerr_flag_ta1){
                cerr<<"正在减速\n";
                cerr<<"当前速度 "<<return_v(robot)<<"\n";
        }
        return ins_t;
    }
    if(robot.need_adjust_statues){
        if(robot.id==print_rob_id&&print_cerr_flag_ta1){
            cerr<<"进入位置调整检测\n";
            cerr<<robot.adjust_pos<<" "<<robot.adjust_pos<<" "<<p1.second<<"\n";
        }
        if(robot.adjust_pos){
            if(robot.id==print_rob_id&&print_cerr_flag_ta1){
                cerr<<"姿势1 ing\n";
                cerr<<"摆动速度："<<p1.first<<"\n";
                cerr<<" 目标";
                printPair(robot.virtual_pos);
                cerr<<" 当前坐标 ";
                printPair(robot.pos);
                cerr<<"dis: "<<payload.distance<<"\n";
            }
            ins_t.rotate=p1.first;
            if(lt(payload.distance,0.1)||
            (!illegal_point[istake][robot.node_id]
            &&!dangerous_point[istake][robot.node_id])){
                robot.cnt_tar=ret_next(robot,robot.cnt_tar);
            if(robot.id==print_rob_id&&print_cerr_flag_ta1){
                cerr<<"完成姿势1，下一个目标\n";
                printPair(exist_id[robot.get_type==0?0:1][robot.cnt_tar]);
                cerr<<robot.need_adjust_statues<<"\n";
            }
                robot.adjust_pos=false;
                ins_t.forward=0;                
            }else{
                if(!need_to_step_back(robot)){
                    if(p1.second)
                        ins_t.forward=0.6;
                    else
                        ins_t.forward=0;
                }else{
                    
                    PayLoad tmpPay=calPayload_back(robot,robot.virtual_pos);
                    auto tmpP1=get_w_now(robot,tmpPay);
                    if(tmpP1.second)
                    {  ins_t.forward=-0.6;
                      cerr<<"time "<<state.FrameID<<" back "<<robot.id<<endl;
                    }
                    else
                        ins_t.forward=0;
                }
            }
            return ins_t;
        }else if(robot.need_adjust_statues&&(!robot.adjust_pos)&&(!p1.second)){
             if(robot.id==0&&print_cerr_flag_ta1){
                cerr<<"姿势2，ing\n";
                cerr<<"摆动速度："<<p1.first<<"\n";
                cerr<<" 目标";
                printPair(robot.virtual_pos);
            }
            ins_t.forward=0;
            ins_t.rotate=p1.first;
            return ins_t;
        }else if(robot.need_adjust_statues&&(!robot.adjust_pos)&&p1.second){
            robot.need_adjust_statues=false;
            robot.adjust_w=false;
            ins_t.forward=0.5;
            ins_t.rotate=p1.first;              
            if(robot.id==0&&print_cerr_flag_ta1)
            cerr<<"状态调整完毕\n";
            return ins_t;
        }else{
            cerr<<"未知情况\n";
            ins_t.forward=6;
            return ins_t;
        }
             
    }

    ins_t.rotate=p1.first;
    ins_t.forward=6;
    if(lt(payload.distance,1)){
        ins_t.forward=2;
    }else if(lt(payload.distance,0.5)){
        if(ret_next(robot,robot.cnt_tar)==-1 ){
            ins_t.forward=0.5;
        }else{
            ins_t.forward=2;
        }
    }
    if(lt(payload.distance,1)&&!p1.second){
            ins_t.forward=0;
    } 
    if(!robot.need_adjust_statues&&lt(payload.distance,0.2)){
         int tmpret=ret_next(robot,robot.cnt_tar);
         if(tmpret!=-1)
            robot.cnt_tar=ret_next(robot,robot.cnt_tar);
    }

    bool con_get_type=false;
    if(gt(payload.angle,Pi/8))
        con_get_type=true;
    if(!robot.need_adjust_statues&&gt(payload.distance,1.1)&&!p1.second&&con_get_type)
    {
        double vLimit=3;
        if(gt(payload.angle,Pi/2)){
            double vLimit=3;
        }else{
            double vLimit=6;
        }
        ins_t.forward=vir_v_1(robot,vLimit);
        if(robot.id==0&&print_cerr_flag_ta)
        cerr<<"-"<<state.FrameID<<" 采样速度 "<<ins_t.forward<<"\n";
        if(ins_t.forward==-1){
            ins_t.forward=0;
            robot.need_adjust_statues=true;
          
        }
    }else if(gt(payload.distance,1.3)&&gt(payload.speed,5)&&dangerous_nums[istake][robot.virtual_id]>=1){
        ins_t.forward=min(4.5,ins_t.forward);
    }
    // int istake=robot.get_type==0?0:1;
    // if(robot.is_illegal){
    //     robot.need_adjust_statues=true;
    // }
 
   if(print_cerr_flag_ta1&&robot.id==print_rob_id&&contr_print_flag){
    cerr<<" FrameID "<< state.FrameID<<" "<<robot.virtual_pos.first<<"-"<<robot.virtual_pos.second<<endl;
    cerr<<"forward: "<<ins_t.forward<<endl;
    cerr<<"angle "<<payload.angle<<endl;
    cerr<<"dis "<<payload.distance<<endl;
    cerr<<"rob node_id"<<robot.node_id<<endl;
    cerr<<robot.isVir<<endl;
    printPair(robot.pos);
    printPair(robot.virtual_pos);
    cerr<<getPosID(robot.virtual_pos)<<"\n";
    printPair(exist_id[0][ret_next(robot,robot.cnt_tar)]);
    cerr<<getPosID(exist_id[0][ret_next(robot,robot.cnt_tar)])<<"\n";
    cerr<<p1.second<<"\n";
   }
     
    return ins_t;
}
// Ins contr_new_tar(Robot& robot){
//     Ins ins_t;
//     PayLoad payload=calPayload(robot,robot.virtual_pos);
//     auto p1=get_w_now(robot,payload);
//     ins_t.forward=1;
//     ins_t.rotate=p1.first;
//     if(lt(payload.distance,0.2)){
//             ins_t.forward=0;
//             robot.cnt_tar=ret_next(robot,robot.cnt_tar);
//             robot.is_new_tar_ing=false;
//             robot.need_adjust_statues=true;
//             cerr<<"完成目标点更新\n";
//     } 
//     return ins_t;
// }
Ins contr_one_rob_0(Robot& robot){
    //ta_aaaa
// if(contr_print_flag)
// cerr<<"rob 2"<<endl;
    Flag_sumulate=0;
    int print_rob_id=2;
   bool print_cerr_flag_ta1=false;
    Ins ins_t;
    int istake=robot.get_type==0?0:1;
    if(robot.target_id==-1){
        ins_t.forward=0;
        ins_t.rotate=Pi;
        // cerr<<"-1返回"<<endl;
        return ins_t;
    }
    
    adjust_virtual_pos_total(robot);
    // if( state.FrameID>14401&&state.FrameID<15000&&robot.id==0&&contr_print_flag){
    //         //cerr<<"pirnt status over"<<endl;
    //         print_rob_id=0;
    //         print_cerr_flag_ta1=true;
    //     }


    // if( state.FrameID>3200&&state.FrameID<3600){
    //     // cerr<<robot.need_adjust_statues<<" rob "<<robot.id<<endl;
    //     print_cerr_flag_ta1=true;
    //     print_rob_id=0;
    // }

    PayLoad payload=calPayload(robot,robot.virtual_pos);
    auto p1=get_w_now(robot,payload);
// if(state.FrameID>13000)print_cerr_flag_ta=true;
// cerr<<print_cerr_flag_ta<<endl;
    if(gt(return_v(robot),0.8)&&robot.need_slow&&robot.need_adjust_statues){
        ins_t.forward=0;
        ins_t.rotate=p1.first;
        if(robot.id==print_rob_id&&print_cerr_flag_ta1){
            cerr<<"time "<<state.FrameID<<endl;
                cerr<<"正在减速\n";
                cerr<<"当前速度 "<<return_v(robot)<<"\n";
        }
        return ins_t;
    }
    if(robot.need_adjust_statues){
        
        if(robot.id==print_rob_id&&print_cerr_flag_ta1){
            cerr<<"time "<<state.FrameID<<endl;
            cerr<<"进入位置调整检测\n";
            cerr<<robot.adjust_pos<<" "<<robot.adjust_pos<<" "<<p1.second<<"\n";
        }
        if(robot.adjust_pos){
            if(robot.id==print_rob_id&&print_cerr_flag_ta1){
                cerr<<"姿势1 ing\n";
                cerr<<"摆动速度："<<p1.first<<"\n";
                cerr<<" 目标";
                printPair(robot.virtual_pos);
                cerr<<" 当前坐标 ";
                printPair(robot.pos);
                cerr<<"dis: "<<payload.distance<<"\n";
                cerr<<" 偏差角 "<<payload.angle<<" "<<payload.sign<<"\n";
            }
            ins_t.rotate=p1.first;
             if(lt(payload.distance,0.1)||(!illegal_point[istake][robot.node_id]
            &&!dangerous_point[istake][robot.node_id])){
                robot.cnt_tar=ret_next(robot,robot.cnt_tar);
            if(robot.id==print_rob_id&&print_cerr_flag_ta1){
                cerr<<"完成姿势1，下一个目标\n";
                printPair(exist_id[robot.get_type==0?0:1][robot.cnt_tar]);
                cerr<<robot.need_adjust_statues<<"\n";
            }
                robot.adjust_pos=false;
                ins_t.forward=0;                
            }else{
                if(!need_to_step_back(robot)){
                    if(p1.second)
                        ins_t.forward=0.6;
                    else
                        ins_t.forward=0;
                }else{
                    PayLoad tmpPay=calPayload_back(robot,robot.virtual_pos);
                    auto tmpP1=get_w_now(robot,tmpPay);
                    if(tmpP1.second)
                    {  ins_t.forward=-0.6;
                      cerr<<"time "<<state.FrameID<<" back "<<robot.id<<endl;
                    }
                    else
                        ins_t.forward=0;
                }
            }
            return ins_t;
        }else if(robot.need_adjust_statues&&(!robot.adjust_pos)&&(!p1.second)){
             if(robot.id==print_rob_id&&print_cerr_flag_ta1){
                cerr<<"姿势2，ing\n";
                cerr<<"摆动速度："<<p1.first<<"\n";
                cerr<<" 目标";
                printPair(robot.virtual_pos);
            }
            ins_t.forward=0;
            ins_t.rotate=p1.first;
            return ins_t;
        }else if(robot.need_adjust_statues&&(!robot.adjust_pos)&&p1.second){
            robot.need_adjust_statues=false;
            robot.adjust_w=false;
            ins_t.forward=0.5;
            ins_t.rotate=p1.first;         
            if(robot.id==print_rob_id&&print_cerr_flag_ta1)
                cerr<<"状态调整完毕"<<endl;
            return ins_t;

        }else{
            cerr<<"未知情况\n";
            ins_t.forward=6;
            return ins_t;
        }
    }
//    cerr<<"sha ye meigan "<<endl;
    // print_cerr_flag_ta1=true;
 
    bool isSlope=false;
    int cnt=robot.cnt_tar;
    bool isDanger=dangerous_point[istake][cnt];
    bool con2=false;
    
    if(has_next(robot)){
        int tmp1=ret_next(robot,cnt);
        int tmp2=cnt;
        int i1=tmp1/100,i2=tmp2/100;
        int j1=tmp1%100,j2=tmp2/100;
        if(fabs(i1-i2)+fabs(j1-j2)==2){
            isSlope=true;
        }
    }
    con2=(isSlope&&isDanger);
    ins_t.rotate=p1.first;
    double change=0.2*(state.FrameID/2==0?-1:1);
    ins_t.forward=lt(payload.distance,1.5)?3:6;
    if(lt(payload.distance,1)){
        ins_t.forward=3;
    }else if(lt(payload.distance,0.5)){
        if(ret_next(robot,robot.cnt_tar)==-1 ){
            ins_t.forward=0.5;
        }else{
            ins_t.forward=1;
        }
    }
    int next_tar=next_node[robot.target_id][istake][robot.virtual_id];
    // if(gt(payload.angle,Pi/3)&&(dangerous_point[istake][robot.virtual_id])){
    //     if(dangerous_nums[istake][robot.virtual_id]>1)
    //         ins_t.forward=0.5;
    // }
    if(lt(payload.distance,1)&&(!p1.second)){
            ins_t.forward=0;
    }
    
    // if(lt(payload.distance,0.3)&&(con2)){
    //     robot.need_adjust_statues=true;
    // } 
    if(!robot.need_adjust_statues&&lt(payload.distance,0.2)){
        int next_tar=ret_next(robot,robot.cnt_tar);
        robot.cnt_tar=next_tar;
        
    }
    if(gt(payload.angle,1.2)&&lt(payload.distance,5)&&gt(return_v(robot),0.5)){
        if(gt(fabs(ins_t.forward),fabs(change)+0.2))
            ins_t.forward+=change;
    }
    // bool con_get_type=false;
    // if(gt(payload.angle,Pi/3))
    //     con_get_type=true;
    // if(!robot.need_adjust_statues&&gt(payload.distance,1.1)&&!p1.second&&con_get_type)
    // {
    //     ins_t.forward=vir_v_0(robot);
    //     if(robot.id==0&&print_cerr_flag_ta)
    //         cerr<<"-"<<state.FrameID<<" 采样速度 "<<ins_t.forward<<endl;
    //     if(ins_t.forward==-1){
    //         ins_t.forward=0;
    //         if(robot.id==0&&print_cerr_flag_ta){
    //             cerr<<"状态调整准备开始"<<endl;
    //             cerr<<"当前速度 "<<return_v(robot)<<endl;
    //         }
    //         robot.need_adjust_statues=true;
    //     }
        
    // }else{
        // Robot tmpRob=robot;
        // cerr<<"预测是否会撞墙： "<<(!can_trajectory_virpos_0(tmpRob,2,5))<<endl;
        // cerr<<robot.is_illegal<<endl;
        // cerr<<(!robot.need_adjust_statues)<<endl;
        // if(!robot.need_adjust_statues&&!can_trajectory_virpos_0(tmpRob,2,15)){
        //     robot.need_adjust_statues=true;
        // }
    // }
// print_cerr_flag_ta=true;
   if(robot.id==print_rob_id&&print_cerr_flag_ta1){
    cerr<<"robot.id "<<robot.id<<endl;
    cerr<<" FrameID "<< state.FrameID<<" "<<robot.virtual_pos.first<<"-"<<robot.virtual_pos.second<<endl;
    cerr<<"forward: "<<ins_t.forward<<endl;
    cerr<<"angle "<<payload.angle<<endl;
    cerr<<"dis "<<payload.distance<<endl;
    cerr<<robot.isVir<<endl;
    cerr<<"rob node_id"<<robot.close_node<<endl;
    cerr<<" robot.cnt_tar "<<robot.cnt_tar<<endl;
    cerr<<" robot.virtual_id "<<robot.virtual_id<<endl;
    cerr<<"next tar "<<next_tar<<" "<<dangerous_nums[istake][next_tar]<<endl;
    cerr<<dangerous_nums[0][robot .node_id]<<endl;
    cerr<<dangerous_nums[0][robot.cnt_tar]<<endl;
    cerr<<dangerous_nums[0][next_tar]<<endl;
    printPair(robot.pos);
    printPair(robot.virtual_pos);
    cerr<<"合法？："<<robot.is_illegal <<endl;
    cerr<<p1.second<<endl;
    cerr<<"可以到达？"<<check_can_arrival(istake,robot.close_node,robot.cnt_tar)<<" "<<robot.virtual_id<<endl;
    cerr<<" robot.cnt_tar "<<robot.cnt_tar<<endl;
    cerr<<robot.is_new_tar_ing<<endl;
    cerr<<ins_t.forward<<endl;
   }
     
    return ins_t;
}
Ins contr_one_rob(Robot& robot){
    print_cerr_flag_ta=false;
    if(robot.target_id==-1){
        Ins ins_t;
        ins_t.forward=0;
        ins_t.rotate=0;
        init_rob_status(robot);
        return ins_t;
    }
    // if(robot.id==1){
    //     cerr<<"进入运动控制函数"<<endl;
    // }

    if(robot.is_illegal&&!robot.need_adjust_statues){
        if(robot.id==2&&print_cerr_flag_ta){
            cerr<<"机器人2在非法位置：\n";
            printPair(robot.pos);
            cerr<<"标准坐标\n";
            printPair(exist_id[robot.get_type==0?0:1][robot.node_id]);
            cerr<<"目标";
            printPair(robot.virtual_pos);
            cerr<<getPosID(exist_id[robot.get_type==0?0:1][robot.node_id])<<"-"
            <<getPosID(robot.virtual_pos)<<"\n";
        }
        //adjust_illegal_pos(robot);
    }else if(robot.is_dangerous){
    }
    // if(robot.id==2)
    // cerr<<robot.id<<" "<<robot.get_type<<endl;
    // print_cerr_flag_ta=false;
    return robot.get_type==0?contr_one_rob_0(robot):contr_one_rob_1(robot);
}

pair<double,bool> get_w_now(const Robot& robot, const PayLoad& payload){
    int robStuID=robot.target_id;
    if(robStuID==-1){
         robStuID=0;
    }
    int robID=robot.id;
    double Dev_val=get_at_stop(0.02,payload.angular_acceleration
    ,robot.angular_velocity,payload.sign);
    double rateAngle_fabs=0;
    if(!robot.need_adjust_statues){
        if(gt(payload.angle,0.25)){
            rateAngle_fabs=Pi;
        }else if(gt(payload.angle,0.2)){
            rateAngle_fabs=Pi/2;
        }else if(gt(payload.angle,0.075)){
            rateAngle_fabs=Pi/4;
        }else{
            rateAngle_fabs=Pi/8;
        }
    }else{
        if(gt(payload.angle,0.25)){
            rateAngle_fabs=Pi;
        }else if(gt(payload.angle,0.1)){
            rateAngle_fabs=Pi/3;
        }else if(gt(payload.angle,0.075)){
            rateAngle_fabs=Pi/4;
        }else{
            rateAngle_fabs=Pi/8;
        }        
    }
    double angle=get_at_v_limt(0.02,payload.angular_acceleration
    ,robot.angular_velocity,rateAngle_fabs,payload.sign);
    double StopA=0;
    double real_angle=angle;
    int can_stop_flag=0;
    bool con1=gt(Dev_val,payload.angle);
    if(con1){
        real_angle=get_at_v_limt(0.02,payload.angular_acceleration
            ,robot.angular_velocity,0,payload.sign);
        can_stop_flag=1;
        StopA=0;
    } 
    double cmpAngle=fabs(payload.angle-real_angle);
    int ctr=1;
    if(robot.need_adjust_statues){
        ctr=0;
    }
    bool can_st=can_stop(robot.pos,robot.virtual_pos,cmpAngle,false,ctr);
    if(can_st){
        can_stop_flag=1;
        StopA=0;        
    }
    // if(robot.need_adjust_statues){
    //     can_st|=can_stop_flag;
    //     can_st|=con1;
    // }
    // if(state.FrameID>=161&&state.FrameID<=1452){
    //     cerr<<"ID: "<<state.FrameID<<" "<<robot.id<<" "<<robot.virtual_pos.first<<"-"<<robot.virtual_pos.second<<" "<<payload.distance<<"\n";
    //     cerr<<can_st<<"\n";
    // }
    double tmpAngle =can_stop_flag?StopA:rateAngle_fabs*payload.sign;
    return {tmpAngle,can_st} ;
}
// double get_v_now(const Robot& robot, const PayLoad& payload){
//     double res_v=6;
//     if(lt(payload.distance,1)){
//         res_v=1;
//     }else if(lt(payload.distance,0.5)){
//         res_v=0.5;
//     }
//     return res_v;
// }

void print_robot_infor(Robot r) {
    cerr<<"robot:"<<r.id<<" get_type:"<<r.get_type<<" tar:"<<r.target_id<<"-"<<studios[r.target_id].type<<"\n";
    cerr<<"node_id:"<<r.node_id<<" close_node:"<<r.close_node<<"\n";
    cerr<<"xy_";
    printPair(r.xy_pos);
    cerr<<"speed:"<<payloads[r.id].speed<<" rotate:"<<r.angular_velocity<<" dir:"<<r.direction<<"\n";
    printPair(r.pos);
    if(r.target_id != -1) cerr<<"dis to tar:" << dis_to_studios[r.target_id][(r.get_type!=0)][r.close_node]<<"\n";
    cerr<<"virtual_";
    printPair(r.virtual_pos);
}

bool change_target(int id1, int id2) {
    // if(robots[id1].loc_id != -1){
    //     if(lt(calcuDis(robots[id1].pos,studios[robots[id1].loc_id].pos),0.4))
    //         return false;
    // }
    // if(robots[id2].loc_id != -1){
    //     if(lt(calcuDis(robots[id2].pos,studios[robots[id2].loc_id].pos),0.4))
    //         return false;
    // }
    if(robots[id1].last_target_id != robots[id1].target_id){
        return false;
    }
    if(robots[id2].last_target_id != robots[id2].target_id){
        return false;
    }
    // if(lt(calcuDis(robots[id1].pos,studios[robots[id1].target_id].pos),0.7)|| lt(calcuDis(robots[id2].pos,studios[robots[id2].target_id].pos),0.7))
    //     return false;
    //keyouhua
    
    if(robots[id1].target_id != -1 && le(get_dis(robots[id1], robots[id2]), 0)) return false;
    if(robots[id2].target_id != -1 && le(get_dis(robots[id2], robots[id1]), 0)) return false;
    if(robots[id1].target_id == robots[id2].target_id) return false;
    // cerr<<"change target\n";
    int cnt;
    if(robots[id1].get_type==robots[id2].get_type && robots[id1].get_type == 0){
        // cerr<<" studio rid : "<<studios[robots[id1].target_id_buy].r_id<<" - "<<studios[robots[id2].target_id_buy].r_id <<"\n";
        if(robots[id1].target_id_buy != -1 ){
            studios[robots[id1].target_id_buy].r_id = id2;
        }
        if(robots[id2].target_id_buy != -1 ){
            studios[robots[id2].target_id_buy].r_id = id1;
        }
        // cerr<<" studio rid : "<<studios[robots[id1].target_id_buy].r_id<<" - "<<studios[robots[id2].target_id_buy].r_id <<"\n";
    }
    // cerr<<"material_rid "<<studios_rid[robots[id1].target_id_send][studios[robots[id1].target_id_buy].type]<<" - "<<studios_rid[robots[id2].target_id_send][studios[robots[id2].target_id_buy].type]<<"\n";
    if(robots[id1].target_id_send != -1){
        if(robots[id1].get_type == 0){
            studios_rid[robots[id1].target_id_send][studios[robots[id1].target_id_buy].type] = id2;
        }
        else{
            studios_rid[robots[id1].target_id_send][robots[id1].get_type] = id2;
        }
    }
    if(robots[id2].target_id_send != -1){
        if(robots[id2].get_type == 0){
            studios_rid[robots[id2].target_id_send][studios[robots[id2].target_id_buy].type] = id1;
        }
        else{
            studios_rid[robots[id2].target_id_send][robots[id1].get_type] = id1;
        }
    }
    // cerr<<"material_rid "<<studios_rid[robots[id1].target_id_send][studios[robots[id1].target_id_buy].type]<<" - "<<studios_rid[robots[id2].target_id_send][studios[robots[id2].target_id_buy].type] <<"\n";
    // cerr<<" target_send : "<<robots[id1].target_id_send<<" - "<<robots[id2].target_id_send<<"\n";
    cnt = robots[id1].target_id_send;
    robots[id1].target_id_send = robots[id2].target_id_send;
    robots[id2].target_id_send = cnt;
    // cerr<<" target_send : "<<robots[id1].target_id_send<<" - "<<robots[id2].target_id_send<<"\n";
    // cerr<<" target_buy : "<<robots[id1].target_id_buy<<" - "<<robots[id2].target_id_buy<<"\n";
    cnt = robots[id1].target_id_buy;
    robots[id1].target_id_buy = robots[id2].target_id_buy;
    robots[id2].target_id_buy = cnt;
    // cerr<<" target_id : "<<robots[id1].target_id<<" - "<<robots[id2].target_id<<"\n";
    cnt = robots[id1].target_id;
    robots[id1].target_id = robots[id2].target_id;
    robots[id1].cnt_tar = robots[id1].node_id;
    robots[id2].target_id = cnt;
    robots[id2].cnt_tar = robots[id2].node_id;
    // cerr<<" target_id : "<<robots[id1].target_id<<" - "<<robots[id2].target_id<<"\n";
    return true;
}

bool cmp_robot(Robot a, Robot b) {
    if(a.target_id == -1) 
        return true;
    if(b.target_id == -1) 
        return false;
    if(a.get_type == b.get_type) {
        return dis_to_studios[a.target_id][(a.get_type != 0)][a.close_node] > dis_to_studios[b.target_id][(b.get_type != 0)][b.close_node];
    }
    return a.get_type < b.get_type;
}

bool check_speed(Robot ro_a, Robot ro_b, double mindis) {
    // pair<double, double> speed = subVector(ro_a.xy_pos, ro_b.xy_pos);
    // Vec pos = subVector(ro_b.pos, ro_a.pos);
    // double t = (payloads[ro_a.id].speed + payloads[ro_b.id].speed) / payloads[ro_a.id].acceleration + 0.04;
    // Vec next_pos = calVectorProduct(speed, t);
    // Vec path_dir = norm(next_pos);
    // double dd = path_dir * pos;
    // if(lt(dd, 0)) return false;
    // if(gt(dd, len(next_pos))) return false;
    // if(gt(pos * pos - dd * dd, mindis * mindis)) return false;

    // Vec speed = subVector(ro_a.xy_pos, ro_b.xy_pos);
    Vec speed = ro_a.xy_pos;
    Vec pos = subVector(ro_b.pos, ro_a.pos);
    double dd = speed * pos;
    if(le(calcuDis(ro_a.pos, ro_b.pos), mindis + 0.5)) return true;
    // printPair(speed);
    // printPair(pos);
    // cerr<<"dd:"<<dd<<"\n";
    
    if(lt(dd, 0)) return false;


    return true;
}

void collision_solve(int frame){
    int i, j, k, z;
    vector<pair<double,double>> trajectory[4];
    vector<pair<double,double>> tmp_tra, tra;
    vector<Robot> ro;
    double mindis;
    double dis, dis_tmp;
    int ans, tmp;
    int ins_num;
    vector<int> coll[4];
    int coll_time[4][4] = {0};
    int vis[4] = {0};
    int flag_avoid[4] = {0};
    int reachTime[4], stopID, goID;
    int choose_id = -1;
    int min_size;
    int x;
    bool flag;
    double speed_limit[4];



    // if(state.FrameID >= 5712 && state.FrameID <= 5712 && 999==999)
        // collision_cerr_flag = true;


    for(i = 0; i < 4; ++i)
        ro.emplace_back(robots[i]);
    sort(ro.begin(), ro.end(), cmp_robot);
    for(i = 0; i < 4; ++i) trajectory[i] = Calculate_the_trajectory(ro[i], 0, frame, 0);


    // if(state.FrameID == 2) {
    //     cerr<<"predict\n";
    //     for(i = 0;i<25;++i){
    //         cerr<<i+2+1<<"\n";
    //         for(j=0;j<4;++j){
    //             cerr<<ro[j].id<<":"<<trajectory[j][i].first<<","<<trajectory[j][i].second<<"\n";
    //         }
    //     }
    //     cerr<<"\n";
    // }

    // if(state.FrameID > 13000) cerr<<"lll\n";

    for (i = 0; i < 4; i++)
    {
        for (j = i + 1; j < 4; j++)
        {
            mindis = ro[i].radius + ro[j].radius + 0.3;
            tmp = checkNoCollision(trajectory[i], trajectory[j], mindis);
            // if(state.FrameID == 1588 && ((ro[i].id == 3 && ro[j].id == 0) || (ro[i].id == 0 && ro[j].id == 3)))
            //     cerr<<"mindis:"<<mindis<<"\n";
            coll_time[i][j] = tmp;
            coll_time[j][i] = tmp;
            if(ro[i].get_type != ro[i].last_get_type){
                last_solution[ro[i].id][ro[j].id] = -1;
                last_solution[ro[j].id][ro[i].id] = -1;
            }
                
            if(tmp == 9000 || gt(calcuDis(ro[i].pos, ro[j].pos), 8)) {
                //同个目标
                // if(ro[i].target_id == ro[j].target_id && ro[i].target_id != -1) {
                //     reachTime[ro[i].id] = getTimeToStudio(ro[i].id, trajectory[i]);
                //     reachTime[ro[j].id] = getTimeToStudio(ro[j].id, trajectory[j]);
                    
                //     if(reachTime[ro[i].id] == reachTime[ro[j].id] && reachTime[ro[i].id]==1000) continue;
                //     stopID = (reachTime[ro[i].id]<reachTime[ro[j].id])? ro[j].id: ro[i].id;
                //     goID = (stopID == ro[i].id)? ro[j].id: ro[i].id;
                //     ins[stopID].forward = min(payloads[stopID].distance / (reachTime[goID]*0.02 + 0.5), ins[stopID].forward);

                //     // if(collision_cerr_flag) {
                //     //     cerr<<"time:"<<state.FrameID<<"\n";
                //     //     // cerr<<ro[i].id<<"reach time:"<<reachTime[ro[i].id]<<"\n";
                //     //     // cerr<<ro[j].id<<"reach time:"<<reachTime[ro[j].id]<<"\n";
                //     //     cerr<<stopID<<"stop:"<<ins[stopID].forward<<"\n";
                //     // }

                // }
                last_solution[ro[i].id][ro[j].id] = -1;
                last_solution[ro[j].id][ro[i].id] = -1;
                continue;
            }
            coll[i].emplace_back(j);
            coll[j].emplace_back(i);
            
            if(collision_cerr_flag) {
                cerr<<"time:"<<state.FrameID<<"\n";
                cerr<<ro[i].id<<"-"<<ro[j].id<<"collison"<<tmp<<"\n";
                print_robot_infor(ro[i]);
                cerr<<"*\n";
                print_robot_infor(ro[j]);
                cerr<<"****\n";
            }
        }
    }

    int pre_x = -1;
    x = -1;
    for(i = 0; i < 2; ++i) {
        choose_id = -1;
        //选择碰撞最多的小球改变状态
        for(j = 0; j < 4; ++j){
            if(vis[j] || coll[j].size() == 0)
                continue;
            if(choose_id == -1 || coll[j].size()> coll[choose_id].size()) {
                choose_id = j;
                break;
            }
        }
        
        //No collision
        if(choose_id == -1 || coll[choose_id].size() == 0)
            break;

        

        // if(collision_cerr_flag) {
        //     cerr<<state.FrameID<<"\n";
        //     cerr<<check_wall_r(1)<<"\n";
        //     for(j=0;j<4;++j){
        //         cerr<<ro[j].id<<":"<<payloads[ro[j].id].speed<<" target:"<<ro[j].target_id<<"\n";
        //     }
        // }



        tmp = 9000;
        //避让最zao发生的碰撞
        for(j = 0; j < coll[choose_id].size(); ++j) {
            if(coll_time[choose_id][coll[choose_id][j]] < tmp) {
                x = coll[choose_id][j];
                tmp = coll_time[choose_id][coll[choose_id][j]];
            }
        }

        if(x == -1) {
            // if(collision_cerr_flag)
            // {cerr<<choose_id<<"*"<<coll[choose_id].size();
            // cerr<<"xx"<<x<<"\n";}
            break;
        }

        // if(cmp_robot(ro[x], ro[choose_id]) && vis[x] != 1) {
        //     tmp = x;
        //     x = choose_id;
        //     choose_id = tmp;
        // }

        vis[choose_id] = 1;

        
        vector<int>::iterator it = find(coll[choose_id].begin(), coll[choose_id].end(), x);
        if(it != coll[choose_id].end()) {
            coll[choose_id].erase(it);
            if(collision_cerr_flag) cerr<<"x delete success\n";
        }
        it = find(coll[x].begin(), coll[x].end(), choose_id);
        if(it != coll[x].end()) {
            coll[x].erase(it);
            if(collision_cerr_flag) cerr<<"choose_id delete success\n";
        }


        if(collision_cerr_flag) {
            cerr<<ro[choose_id].id<<"avoid"<< ro[x].id<<"\n";
        }

        if(ro[choose_id].real_get_type == ro[x].real_get_type) {
            //交换目标
            if(change_target(ro[choose_id].id, ro[x].id)){
                if(collision_cerr_flag) cerr<<"change target\n"<<"----------\n";
                continue;
            }
        }

        if(ro[choose_id].target_id != -1 && ro[x].target_id != -1) {
            int tar1 = ro[choose_id].target_id;
            int tar2 = ro[x].target_id;
            int is_take1 = (ro[choose_id].get_type != 0);
            int is_take2 = (ro[x].get_type != 0);
            int node1 = ro[choose_id].close_node;
            int node2 = ro[x].close_node;
            if(gt(dis_to_studios[tar2][is_take2][node2] - dis_to_studios[tar1][is_take1][node1], 20)) {
                if(collision_cerr_flag) {
                    cerr<<"change choose x\n";
                    cerr<<dis_to_studios[tar1][is_take1][node1]<<"* x:"<<dis_to_studios[tar2][is_take2][node2]<<"\n";
                }
                vis[choose_id] = 0;
                tmp = x;
                x = choose_id;
                choose_id = tmp;
                vis[choose_id] = 1;
            }
        }


        ans = -1;
        dis = 10000;
        min_size = 27;
        mindis = ro[choose_id].radius + ro[x].radius + 0.1;
        ins_num = 4;
        for(k = 0; k < ins_num; ++k) {
            if(k < 3) {
                tmp_tra = Calculate_the_trajectory(ro[choose_id], ins_set[k], 0, 1, trajectory[x], 0, 25, mindis + 0.2, 100);
            }
            // else if(k < 4) {
            //     tmp_tra = Calculate_the_trajectory(ro[choose_id], ins_set[k], 1, 1, trajectory[x], 0, 25, mindis + 0.2, 100);
            // }
            else {
                tmp_tra = Calculate_the_trajectory(ro[choose_id], ins_set[k], 1, 0, trajectory[x], 0, 25, mindis, 100);
            }

            // if(collision_cerr_flag) cerr<<k<<"-"<<tmp_tra.size()<<"\n";
            if(tmp_tra.size() == 0) continue;
            flag = false;

            //检测是否会和其他小球发生碰撞
            // for(j = 0; j < 4; ++j){
            //     if(j == choose_id) continue;
            //     if(checkNoCollision(tmp_tra, trajectory[j], ro[choose_id].radius + ro[j].radius) == 9000) {
            //         flag =true;
            //         break;
            //     }
            // }
            //若和其他小球碰撞则更换策略
            // if(flag) continue;
            if(ro[choose_id].target_id == -1)
                dis_tmp = 1;
            else dis_tmp = calcuDis(tmp_tra[tmp_tra.size() - 1], studios[ro[choose_id].target_id].pos);
            // if(collision_cerr_flag) cerr<<"dis:"<<dis_tmp<<"\n";

            if((le(dis_tmp, dis) && min_size >= tmp_tra.size()) || min_size > tmp_tra.size() ) {
                min_size = tmp_tra.size();
                dis = dis_tmp;
                ans = k;
                tra = tmp_tra;
            }
        }

        // if(state.FrameID > 13000) cerr<<"kkk\n";
        if(ans != -1 && gt(payloads[ro[choose_id].id].speed, 1) && (!eq(ins[ro[choose_id].id].forward, 0) || ro[choose_id].target_id == -1 
                        || le(calcuDis(ro[choose_id].pos, studios[ro[choose_id].target_id].pos), 1))
                        && (ans != 3|| !check_nead_slow_down(ro[x], ro[choose_id], mindis, coll_time[choose_id][x]))) {
            // if(state.FrameID > 13000) cerr<<"jjj\n";
            if(collision_cerr_flag) {
                cerr<<payloads[ro[choose_id].id].angle<<"-"<<payloads[ro[choose_id].id].sign<<"\n";
                cerr<<"old solution:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<"\n";
            }
            trajectory[choose_id] = tra;
            if(collision_cerr_flag) cerr<<"ans"<<ans<<"\n";
            if(vis[x] && lt(ins[ro[x].id].forward, 0)) {
                do_back(ro[choose_id].id, ro[x].pos);
            }
            else updateIns(ro[choose_id].id, ans);
            coll_time[x][choose_id] = 9000;
            coll_time[choose_id][x] = 9000;
            pre_x = x;
            last_solution[ro[choose_id].id][ro[x].id] = ans;
            last_solution[ro[x].id][ro[choose_id].id] = -1;


            // if(ro[choose_id].target_id == ro[x].target_id && ro[choose_id].target_id != -1) {

            //     reachTime[ro[x].id] = getTimeToStudio(ro[x].id, trajectory[x]);

            //     stopID = ro[choose_id].id;
            //     goID = ro[x].id;
            //     ins[stopID].forward = min(payloads[stopID].distance / (reachTime[goID]*0.02 + 0.5), ins[stopID].forward);
                    

            //         if(collision_cerr_flag) {
            //             // cerr<<"time:"<<state.FrameID<<"\n";
            //             // cerr<<ro[i].id<<"reach time:"<<reachTime[ro[i].id]<<"\n";
            //             // cerr<<ro[j].id<<"reach time:"<<reachTime[ro[j].id]<<"\n";
            //             cerr<<stopID<<"stop:"<<ins[stopID].forward<<"\n";
            //         }
            // }

            if(collision_cerr_flag) {
                cerr<<ans<<"\n";
                cerr<<ro[x].id<<"-"<<ro[choose_id].id<<":"<<last_solution[ro[x].id][ro[choose_id].id]<<"\n";
                if(ans<3) {
                    cerr<<"chose solution01:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<"\n";
                }
                // else if(ans==3) {
                //     cerr<<"chose solution11:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<"\n";
                // }
                else {
                    cerr<<"chose solution10:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<"\n";
                }
            }
        }
        else {
            // if(collision_cerr_flag) updateIns(ro[choose_id].id, 4);
            // else
            // adjust_collo_new(ro[choose_id].id, ro[x].id, payloads[ro[choose_id].id].sign);
            // solveNoSolution(ro[choose_id].id, ro[x].id);
            
            // double dis = calcuDis(ro[choose_id].pos, ro[x].pos);
            // bool back_flag = lt(dis, mindis + 0.2);
            // if(state.FrameID > 13000) cerr<<"hhh\n";
            if(check_nead_slow_down(ro[choose_id], ro[x], mindis + 0.1, coll_time[choose_id][x])) {

                do_back(ro[choose_id].id, ro[x].pos);
                // if(flag_avoid[choose_id] && eq(ins[ro[choose_id].id].forward, -2)) {
                //     ins[ro[choose_id].id].forward = max(ins[ro[choose_id].id].forward, speed_limit[choose_id]);
                // }
                if(collision_cerr_flag){
                    cerr<<"choose back\n";
                    cerr<<"ins:"<<ins[ro[choose_id].id].forward<<" "<<ins[ro[choose_id].id].rotate<<"\n";
                }
                double speed = calVectorProduct(ro[choose_id].xy_pos, subVector(ro[choose_id].pos, ro[x].pos));
                if(collision_cerr_flag) {
                    // cerr<<"dis:"<<get_dis(ro[choose_id], ro[x])<<"\n";
                    cerr<<"speed:"<<speed<<"\n";
                }
                if(ro[choose_id].target_id != ro[x].target_id
                     && check_nead_slow_down(ro[x], ro[choose_id], mindis + 0.2, coll_time[choose_id][x]) 
                     && check_speed(ro[x], ro[choose_id], mindis)) {
                    vis[x] = 1;
                    // if(le(fabs(get_dis(ro[choose_id], ro[x])), 4) && gt(speed, -1)){
                    //     do_back(ro[x].id, ro[choose_id].pos);
                    //     if(collision_cerr_flag) {
                    //         cerr<<"x doback\n";
                    //         cerr<<"ins:"<<ins[ro[x].id].forward<<" "<<ins[ro[x].id].rotate<<"\n";
                    //     }
                    // }
                    // else {
                        vis[x] = 1;
                        ins[ro[x].id].forward = min(1.5, ins[ro[x].id].forward);
                        if(collision_cerr_flag) {
                            cerr<<"x 减速\n";
                            cerr<<"ins:"<<ins[ro[x].id].forward<<" "<<ins[ro[x].id].rotate<<"\n";
                        }
                    // }
                }
            }
            else if(check_nead_slow_down(ro[x], ro[choose_id], mindis  + 0.1, coll_time[choose_id][x])) {
                // if(state.FrameID > 13000) cerr<<"zzz"<<endl;
                // do_back(ro[x].id, ro[choose_id].pos);
                vis[x] = 1;
                ins[ro[x].id].forward = 0;
                if(collision_cerr_flag){
                    cerr<<"choose not back\n";
                    cerr<<"ins:"<<ins[ro[x].id].forward<<" "<<ins[ro[x].id].rotate<<"\n";
                }
                // vis[choose_id] = 0;
                // vis[x] = 1;
                // flag_avoid[choose_id] = 1;
                // if(gt(ins[ro[x].id].forward * payloads[ro[x].id].speed, 0) || lt(payloads[ro[x].id].speed, 0))
                //     speed_limit[choose_id] = payloads[ro[x].id].speed;
                // else
                //     speed_limit[choose_id] = 0;
            }
            else {
                vis[x] = 1;
            }
            
            // if(state.FrameID > 13000) cerr<<"qqq\n";
            
            // if(collision_cerr_flag) {
            //     cerr<<"~old solution:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<"\n";
            //     // cerr<<ro[choose_id].id<<"-"<<ro[x].id<<":"<<"last_solution[choose_id][x]"<<last_solution[ro[choose_id].id][ro[x].id]<<"\n";
            // }
            // // cerr<<state.FrameID<<"no solution to avoid collision"<<ro[choose_id].id<<"-"<<ro[x].id<<"*"<<coll_time[choose_id][x]<<"\n";

            // if(false && last_solution[ro[choose_id].id][ro[x].id] != -1) {
            //     if(le(payloads[ro[choose_id].id].speed, 0) && le(ro[choose_id].angular_velocity, 0))
            //         updateIns(ro[choose_id].id, 7);
            //     else updateIns(ro[choose_id].id, last_solution[ro[choose_id].id][ro[x].id]);
                
            //     if(collision_cerr_flag)
            //     {
            //         if(last_solution[ro[choose_id].id][ro[x].id]<3) {
            //             cerr<<"~chose solution01:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<"\n";
            //         }
            //         else if(last_solution[ro[choose_id].id][ro[x].id] == 4) {
            //             cerr<<"~chose solution11:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<"\n";
            //         }
            //         else {
            //             cerr<<"~chose solution10:"<<ins[ro[choose_id].id].forward<<"**"<<ins[ro[choose_id].id].rotate<<"\n";
            //         }
            //     }
            // }
            // else {
            //     if(vis[x] && vis[choose_id]) {
            //         // ins[ro[choose_id].id].forward = -2;
            //         if(collision_cerr_flag) cerr<<"no way\n";
            //         // adjust_collo_new(ro[choose_id].id, ro[x].id, payloads[ro[choose_id].id].sign);
            //         continue;
            //     }
            //     choose_id = x;
            //     x = -1;
            //     i--;
            // }            
        }
        if(collision_cerr_flag) cerr<<"------------\n";
        
    }
    

    // if(state.FrameID >= 11931 && state.FrameID <= 11960) {
    //     int a,b;
    //     for(i = 0; i < 4; ++i){
    //         if(ro[i].id == 1) a = i;
    //         if(ro[i].id == 3) b =i;
    //     }
    //     cerr<<state.FrameID;
    //     printPredictRobotsDis(trajectory[a], trajectory[b]);
    //     cerr<<"time:"<<state.FrameID<<"\n";
    //     print_robot_infor(ro[a]);
    //     cerr<<"*\n";
    //     print_robot_infor(ro[b]);
    // }

    // if(state.FrameID >= 1693 && state.FrameID <= 1730) {
    //     cerr<<state.FrameID;
    //     printRobotsDis(0,2);
    // }
    
    updateGetType();
}



// void printRobotsDis(int i, int j){
//     cerr<<"&pos:("<<robots[i].pos.first<<", "<<robots[i].pos.second<<")--("<<robots[j].pos.first<<", "<<robots[j].pos.second<<") dis:"<<calcuDis(robots[i].pos, robots[j].pos)<<"\n";
// }

// void printRobotsDis(Robot ro, pair<double,double> a){
//     cerr<<"pos:("<<ro.pos.first<<", "<<ro.pos.second<<")--("<<a.first<<", "<<a.second<<") dis:"<<calcuDis(ro.pos, a)<<"\n";
// }

// void printPredictRobotsDis(const vector<pair<double,double>> &a, const vector<pair<double,double>> &b) {
//     int count = min(a.size(), b.size());
//     for(int i = 0; i < count; ++i) {
//         cerr<<state.FrameID+i;
//         cerr<<"pos:("<<a[i].first<<", "<<a[i].second<<")--("<<b[i].first<<", "<<b[i].second<<") dis:"<<calcuDis(a[i], b[i])<<"\n";
//     }
//     cerr<<"-----------\n";
// }

// int getTimeToStudio(int id, const vector<pair<double,double>> &a) {
//     int len = a.size();
//     for(int i = 0; i < len; ++i) {
//         if(lt(calcuDis(studios[robots[id].target_id].pos, a[i]), 0.4))
//             return i;
//     }
//     return 1000;
// }

// bool check_will_collision(const Robot &ro1, const Robot &ro2) {
//     int node1 = ro1.node_id;
//     int node2 = ro2.node_id;

//     if(!is_connected(node1, node2)) return false;
//     if(ro1.target_id == -1 && ro2.target_id == -1) return false;

//     if(gt(fabs(get_dis(ro1, ro2)), 10)) return false;
    

//     return false;
// }

int check_avoid(const Robot &ro_back, const Robot &ro_go, double mindis) {
    int node = ro_back.close_node;
    priority_queue<Graph_node, vector<Graph_node>, cmp_Graph_node> q;
    int from, pre_id, num, i, to, is_take;
    double dis, new_dis, angle_sum;
    int s, danger_sum;
    unordered_map<int, int> vis_node;
    unordered_map<int, int> pre_node;
    unordered_map<int, double> angle_node;
    bool cerr_flag = false;
    q.push(Graph_node{node, 0, node});
    while(!q.empty()) {
        Graph_node now_node = q.top();
        q.pop();
        if(vis_node[now_node.id]) continue;
        from = now_node.id;
        dis = now_node.dis;
        pre_id = now_node.pre_id;
        vis_node[now_node.id] = 1;
        pre_node[from] = pre_id;

        if (check_node_safe(from, mindis, ro_go))
        {
            to = from;
            while (pre_node[to] != to)
            {
                if (pre_node[to] == node)
                    return to;
                to = pre_node[to];
            }
        }

        num = graph_edge[is_take][from].size();
        if(cerr_flag) {
            cerr<<"node_id:"<<from<<" dis:"<<dis<<" pre_id:"<<pre_id<<"danger_sum:"<<now_node.dangerous_sum<<"\n";
            cerr<<"edge-num:"<<num<<"\n";
        }
        for(i = 0; i < num; ++i) {
            to = graph_edge[is_take][from][i].id;
            if(vis_node.count(to) || to == pre_id) continue;
            if(le(calcuDis(exist_id[is_take][to], ro_go.pos), mindis)) continue;
            angle_sum = now_node.angle_sum + calAngleToDis(pre_id, from, to);
            new_dis = dis + graph_edge[is_take][from][i].dis;
            q.push(Graph_node{to, new_dis, from, angle_sum});
        }
    }
    return -1;
}

void updateIns(int id, int i) {
    if(i<3) {
        ins[id].rotate = ins_set[i].rotate;
        // cerr<<"chose solution11:"<<ins[id].forward<<"**"<<ins[id].rotate<<"\n";
    }
    // else if(i == 3) {
    //     ins[id].forward = ins_set[i].forward;
    //     ins[id].rotate = ins_set[i].rotate;
    //     // cerr<<"chose solution01:"<<ins[id].forward<<"**"<<ins[id].rotate<<"\n";
    // }
    else {
        ins[id].forward = ins_set[i].forward;
        // cerr<<"chose solution10:"<<ins[id].forward<<"**"<<ins[id].rotate<<"\n";
    }
    robots[id].cnt_tar = robots[id].node_id;
}

double get_stop_time(double x, double v0, int sign, double acceleration) {
    double vt2 = 2 * acceleration * x - v0 * v0 * sign;
    v0 = fabs(v0);
    if(eq(vt2, 0)) return v0 / acceleration;
    if(lt(vt2, 0)) return (sqrt(fabs(vt2)) + v0) / acceleration;
    // cerr<<"vt2"<<vt2<<endl;
    return (sqrt(vt2) - v0 * sign) / acceleration;
}

double get_rotation_stop_time(const Robot &ro, PayLoad pay) {
    int sign = ge(ro.angular_velocity * pay.sign, 0)? 1: -1;
    return get_stop_time(pay.angle, ro.angular_velocity, sign, pay.angular_acceleration);
}


bool do_back(int id, pair<double, double> pos) {
    int to = choose_best_to(robots[id], pos);
    if(to == -1) return false;
    pair<double, double> to_pos = exist_id[(robots[id].get_type != 0)][to];
    PayLoad pay_back = calPayload_back(robots[id], to_pos);
    PayLoad pay = calPayload(robots[id], to_pos);
    double time, time_back;
    
    robots[id].cnt_tar = robots[id].node_id;

    

    time = get_rotation_stop_time(robots[id], pay);
    time_back = get_rotation_stop_time(robots[id], pay_back);

    if(collision_cerr_flag) {
        cerr<<"forward angle:"<<pay.angle<<"*"<<pay.sign
            <<"\ntime:"<<time
            <<"\nback angle:"<<pay_back.angle<<"*"<<pay_back.sign
            <<"\nback time:"<<time_back<<"\n";
    }

    if(lt(time, time_back - 0.04)) {
        if(robots[id].get_type)
            ins[id].forward = 2.5;
        else
            ins[id].forward = 4;
        ins[id].rotate = get_w_now(robots[id], pay).first;
    }
    else {
        pay = pay_back;
        ins[id].forward = -2;
        if(le(payloads[id].speed, linear_velocity[(robots[id].get_type != 0)])) {
            ins[id].rotate = get_w_now(robots[id], pay).first;
        }
    }
    return true;
}

PayLoad calPayload_back(Robot robot, pair<double, double> virtual_pos) {
    double distance = calcuDis(robot.pos, virtual_pos);
    double angular_acceleration = robot.get_type == 0? angular_acceleration_no :angular_acceleration_has;
    double acceleration = robot.get_type == 0? acceleration_no: acceleration_has;
    double speed = calVectorSize(robot.xy_pos) * (ge(calVectorProduct(robot.xy_pos, transformVector(robot.direction)), 0.0)? 1: -1);

    // 计算机器人与目标点构成的向量与x轴正方向夹角
    pair<double, double> robotToStudio = subVector(virtual_pos, robot.pos);
    double angle1 = calAngle(robotToStudio);

    double angle2 = ge(robot.direction, 0.0)? robot.direction: 2 * Pi + robot.direction;
    angle2 += Pi;
    angle2 = gt(angle2, 2 * Pi)? angle2 - Pi * 2: angle2;
    // double angle2 = calAngle(robot.xy_pos);

    double angle = angle2 - angle1;

    
    // if(state.FrameID==7010&& robotID==2) {
    //     printPair(robot.xy_pos);
    //     cerr<<"payload-speed:"<<speed<<"\n";
    // }
    int sign;

    if(ge(angle, 0) && lt(angle, Pi) || lt(angle, -Pi))
        sign = -1;
    else
        sign = 1;
    angle = fabs(angle);
    angle  = gt(angle, Pi)? 2 * Pi - angle: angle;


    // cerr<<"**"<< angle1<<"**dir:"<<robot.direction<<"**"<<angle2<<"\n";
    // cerr<<"**"<< angle << "**"<<distance<<"**"<<sign<<"\n";

    return PayLoad((robot.get_type == 0? 0.45: 0.53), angle, angular_acceleration, acceleration, distance, speed, sign);
}

bool check_node_illegal(int x, int y) {
    if(x < 0 || x >= 100 || y < 0 || y>= 100)
        return true;
    return false;
}

bool check_nead_slow_down(const Robot &ro, const Robot &ro_static, double mindis, int coll_frame) {
    int tar = ro.target_id;
    int is_take = (ro.get_type != 0);
    int node1 = ro.close_node;
    int cnt = 0;
    // int node2 = choose_close_node(is_take, ro_static.pos);
    if(tar == -1) {
        for(int i = 0; i < studios.size(); ++i) {
            if(!eq(dis_to_studios[i][is_take][node1], 10000)){
                tar = i;
                break;
            }
        }
        if(tar == -1) return true;
    }
    // if(collision_cerr_flag) {
    //     cerr<<"###########\n"<<"mindis:"<<mindis<<"\n";
    //     cerr<<ro.id<<"\n";
    // }
    while(next_node[tar][is_take][node1] != node1) {
        node1 = next_node[tar][is_take][node1];
        cnt++;
        if(lt(calcuDis(exist_id[is_take][node1], ro_static.pos), mindis))
            return true;
        // if(collision_cerr_flag) {
        //     cerr<<"node"<<node1<<" ddd:"<< calcuDis(exist_id[is_take][node1], ro_static.pos) <<"\n";
        // }
    }

    if(cnt < coll_frame && is_take == 0 && studios[tar].pStatus == 0 && studios[tar].r_time > coll_frame) {
        return true;
    }

    // if(collision_cerr_flag) {
    //     // cerr<<"node2:"<<node2<<"\n";
    //     cerr<<"studio state:"<<studios[tar].pStatus<<" "<<studios[tar].r_time<<"\n";
    //     cerr<<"###########\n";
    // }

    return false;
}

bool check_node_safe(int node_id, double mindis, const Robot &ro) {
    int tar = ro.target_id;
    int is_take = (ro.get_type != 0);
    int node1 = ro.close_node;

    if(ro.target_id == -1) return true;

    while(next_node[tar][is_take][node1] != node1) {
        node1 = next_node[tar][is_take][node1];
        if(le(calcuDis(exist_id[is_take][node1], ro.pos), mindis))
            return false;
        // if(collision_cerr_flag) {
        //     cerr<<"node"<<node1<<" ddd:"<< calcuDis(exist_id[is_take][node1], ro_static.pos) <<"\n";
        // }
    }
    return true;
}




double get_dis(const Robot &ro1, const Robot &ro2) {
    int is_take = (ro1.get_type != 0);
    int tar = ro1.target_id;
    // cerr<<ro1.node_id<<"-"<<ro2.node_id<<" *";
    int node1 = ro1.close_node;
    int node2 = choose_close_node(is_take, ro2.pos);
    // cerr<<node1<<"-"<<node2<<" ";
    if(tar == -1) {
        for(int i = 0; i < studios.size(); ++i) {
            if(!eq(dis_to_studios[i][is_take][node1], 10000)){
                tar = i;
                break;
            }
        }
    }

    // if(state.FrameID == 304) {
    //     cerr<<"tar:"<<tar<<"\n";
    //     cerr<<"node1:"<<choose_close_node(tar, is_take, ro1.pos)<<"\n";
    //     cerr<<"node2:"<<choose_close_node(tar, is_take, ro2.pos)<<"\n";
    //     cerr<<"ro"<<ro1.id<<":"<<dis_to_studios[tar][is_take][node1]<<"\n";
    //     cerr<<"ro"<<ro2.id<<":"<<dis_to_studios[tar][is_take][node2]<<"\n";
    // }
    
    return dis_to_studios[tar][is_take][node1] - dis_to_studios[tar][is_take][node2];
}

int choose_close_node(int is_take, pair<double, double> pos) {
    int close_node = -1;
    double dis, tmp, dis_to_tar;
    int node_id = trans_pos_to_nodeID(pos);
    dis = 10000;
    // cerr<<node_id<<":";
    for(int i = (node_id / 100)-1; i < (node_id / 100) + 2; ++i) {
        for(int j = (node_id % 100) -1; j < (node_id % 100) +2; ++j) {
            if(check_node_illegal(i, j)) continue;
            // cerr<<i*100+j<<" exit:"<<exist_id[is_take].count(i*100+j)<<"\n";
            if(graph_edge[is_take].count(i*100+j) == 0 || graph_edge[is_take][i * 100 + j].size() == 0) continue;
            tmp = calcuDis(pos, exist_id[is_take][i * 100 + j]);
            if(lt(tmp, dis)) {
                dis = tmp;
                close_node = i * 100 + j;
                // cerr<<" "<<close_node;
            }
        }
    }
    // cerr<<"\n";
    if(close_node == -1) cerr<<"choose close node error\n";
    return close_node;
}




int choose_best_to(Robot &ro, pair<double, double> pos) {
    int is_take = (ro.get_type != 0);
    int node_id = ro.close_node;
    int to, to_max = -1;
    double dis, tmp, dis_old;
    int tmp_size, dangerous = 10, danger;

    // if(next_node[tar][is_take][ro.node_id] != -1) 
    //     return calPayload(ro, trans_nodeID_to_pos(next_node[tar][is_take][ro.node_id]));
    dis = 0;
    dis_old = calcuDis(ro.pos, pos);

    for (int i = 0; i < graph_edge[is_take][node_id].size(); ++i)
    {
        to = graph_edge[is_take][node_id][i].id;
        if (graph_edge[is_take].count(to) == 0 || graph_edge[is_take][to].size() <= 1)
            continue;
        danger = dangerous_nums[is_take][to];
        tmp = calcuDis(pos, exist_id[is_take][to]);
        if (collision_cerr_flag)
        {
            cerr << "to_tmp:" << to << " dis:" << calcuDis(pos, exist_id[is_take][to]) << " danger:" << danger << endl;
            cerr << "dis-rob:" << calcuDis(exist_id[is_take][to], ro.pos) << endl;
            cerr << "dis-old:" << dis_old << "\n";
            cerr << ro.radius << endl;
            printPair(exist_id[is_take][to]);
        }
        if (gt(tmp, dis_old) && (danger < dangerous || gt(tmp, dis) && danger <= dangerous))
        {
            dis = tmp;
            to_max = to;
            dangerous = danger;
        }
    }
    if(to_max == -1) return -1;
    //点在机器人中
    if(le(calcuDis(exist_id[is_take][to_max], ro.pos), ro.radius + 0.001)) {
        node_id = to_max;
        dangerous = 1;
        for(int i = 0; i < graph_edge[is_take][node_id].size(); ++i) {
            to = graph_edge[is_take][node_id][i].id;
            danger = dangerous_point[is_take].count(to);
            tmp = calcuDis(pos, exist_id[is_take][to]);
            if(gt(tmp, dis) && danger <= dangerous) {
                if(collision_cerr_flag) cerr<<"&to_tmp:"<<to<<" dis:"<<calcuDis(pos, exist_id[is_take][to])<<endl;
                dis = tmp;
                to_max = to;
                dangerous = danger;
            }
        }
    }
    if(collision_cerr_flag) cerr<<"to:"<<to_max<<"\n";
    return to_max;
}



int checkNoCollision(const vector<pair<double,double>> &a, const vector<pair<double,double>> &b, double mindis) {
    int count = min(a.size(), b.size());
    double dis;
    bool flag = 0;
    for(int i = 0; i < count; ++i) {
        dis = calcuDis(a[i], b[i]);
        if(le(dis, mindis + flag * 0.1)) {
            if(i == 0) {
                mindis = dis;
                flag = 1;
            }
            else return i;
        }
    }
    return 9000;
}

void updateGetType(){
    for(int i = 0; i < 4; ++i){
        robots[i].last_get_type = robots[i].get_type;
    }
}

pair<double ,double> return_change_v(double w,double changeSeta,pair<double,double>v){
    // cerr<<v.first<<' '<<v.second<<"\n";

    double v_value = sqrt(v.first*v.first+v.second*v.second);
    double r = (v_value/fabs(w));
    double l = fabs(w)*r;
    // cerr<<(v.first/v_value)<<"\n";
    // cerr<<(v.second/v_value)<<"\n";
    double direct1 = acos(v.first/v_value);
    // cerr<<"direct1 = "<<direct1<<"\n";
    double direct2;
    if(asin(v.second/v_value)<0)direct1 += Pi;
    direct2 = direct1 + changeSeta;
    // cerr<<"direct2 = "<<direct2<<"\n";
    if(direct2>(2*Pi))direct2 -= (2*Pi);
    if(direct2<0)direct2 += (2*Pi);
    // cerr<<"direct2 = "<<direct2<<"\n";
    double v_new = l/0.02;
    return pair<double,double>((v_new*cos(direct2)),(v_new*sin(direct2)));

}
// double will_Collo_new(int i1,int i2){
//     new_cllo_time=-8;
//     Ins ins;
//     auto res1= Calculate_the_trajectory(robots[i2],0,25,0);
//     auto res2= Calculate_the_trajectory(robots[i1],ins,0,0,res1,0,25,getRobotRadius(i1)+getRobotRadius(i2)+0.2);
    
//     return new_cllo_time;
// }
// void adjust_collo_new(int i1,int i2,int baseSign){
//     double tmpDis=calcuDis(robots[i1].pos,robots[i2].pos);
//     int sel=i1,sel_1=i2;
//     if(lt(tmpDis,5)){
//         int sign=return_line_dire(sel,sel_1,baseSign);
//         //cerr<<"FrameID  "<<state.FrameID<<" collosion: "<<sel_1<<"-> "<<sel<<" "<<sign<<"\n";
//         if(sign==0)return;
//         ins[sel_1].rotate=Pi/4*sign;    
//     }
// }
// bool check_wall_r(int i){
//     Robot robot=robots[i];
//     vector<double> tmp=get_T_limits(robot);
//     bool con1=robot.need_rote_wall;
//     bool con2=!eq(tmp[0],-7)&&(!is_range(robot.direction,tmp))&&ge(payloads[i].angle,Pi/6);
//     if(con1&&con2){
//         return true;
//     }    
//     return false;
// }
double get_Angle_xy(Robot& rob){
    if(lt(fabs(rob.xy_pos.first),0.1)&&lt(fabs(rob.xy_pos.second),0.1)){
        return rob.direction;
    }
    Vec v1;
    v1.x=1;
    v1.y=0;
    Vec v2(rob.xy_pos);
    double angle=acos(cos_t(v1,v2));
    int sign=lt(rob.xy_pos.second,0)?-1:1;
    return angle*sign;
}
double Calculate_the_projection_speed(Robot& rob){
    Vec v1;
    v1.x=cos(rob.direction);
    v1.y=sin(rob.direction);;
    Vec v2(rob.xy_pos);
    return v1*v2;  
}
void cal_matrix(vector<vector<double>>&c,double angle1_w,double angle2){
     double a[2][2];
     double b[2][2];
    a[0][0]=cos(angle1_w);
    a[0][1]=-1*sin(angle1_w);
    a[1][0]=sin(angle1_w);
    a[1][1]=cos(angle1_w);
    b[0][0]=cos(angle2);
    b[0][1]=-1*sin(angle2);
    b[1][0]=sin(angle2);
    b[1][1]=cos(angle2);
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }    
}
// bool check_will_colloWithWall(const Robot& rob){
//     int tarid=rob.target_id==-1?0:rob.target_id;
//     double dis=calcuDis(rob.pos,studios[tarid].pos);
//     double stop_dis=(rob.xy_pos.first*rob.xy_pos.first+rob.xy_pos.second*rob.xy_pos.second)
//         /(2*payloads[rob.id].acceleration);
//     if(isWall(tarid)&&gt(stop_dis+0.2,dis)){
//         return true;
//     }
//     return false;
// }
// bool check_fall_into_scope(double k1,double b1,double b2,double k2,double b3,double b4,pair<double,double> pos){
//     if(gt((k1*pos.first+b1),pos.second)){
//         if (lt((k1 * pos.first + b2), pos.second)){
//             if (gt((k2 * pos.first + b3), pos.second)){
//                 if (lt((k2 * pos.first + b4), pos.second)){
//                     return true;
//                 }
//             }
//         }
//     }
//     return false;
// }
// bool check_barrier(pair<double,double> start,pair<double,double> end,int carry){
//     double r, k, b, offset,k1,b1,b2,k2,b3,b4;
//     pair<double,double>zs,zx,ys,yx;
//     double offset_x;
//     double offset_y;
//     if(carry==1)r = 0.45;
//     else r = 0.53;
//     if (!eq((end.first - start.first),0 )){
//         k =(double) (end.second - start.second) / (end.first - start.first);
//     }
//     else if (gt((end.second - start.second),0)){
//         k = 100;
//     }
//     else if (lt((end.second - start.second), 0)){
//         k = -100;
//     }
//     else k = 0;
//     b = (start.second - k * start.first);
//     k1 = k;
//     offset_x =fabs((double)r*(cos(atan(k))));
//     offset_y =fabs((double)r*(cos(atan(k))));
//     if(lt(start.first,end.first) && lt(start.second,end.second)){
//         zs.first = start.first-offset_x;
//         zx.first = start.first+offset_x;
//         ys.first = end.first-offset_x;
//         yx.first = end.first+offset_x;
//         zs.second = start.second+offset_y;
//         zx.second = start.second-offset_y;
//         ys.second = end.second+offset_y;
//         yx.second = end.second-offset_y;
//     }
//     else if(gt(start.second,end.second)){
//         zs.first = start.first-offset_x;
//         zx.first = start.first+offset_x;
//         ys.first = end.first-offset_x;
//         yx.first = end.first+offset_x;
//         zs.second = end.second+offset_y;
//         zx.second = end.second-offset_y;
//         ys.second = start.second+offset_y;
//         yx.second = start.second-offset_y;
//     }
//     else if(gt(start.first,end.first) && lt(start.second,end.second)){
//         zs.first = end.first-offset_x;
//         zx.first = end.first+offset_x;
//         ys.first = start.first-offset_x;
//         yx.first = start.first+offset_x;
//         zs.second = start.second+offset_y;
//         zx.second = start.second-offset_y;
//         ys.second = end.second+offset_y;
//         yx.second = end.second-offset_y;
//     }
//     else{
//         zs.first = end.first-offset_x;
//         zx.first = end.first+offset_x;
//         ys.first = start.first-offset_x;
//         yx.first = start.first+offset_x;
//         zs.second = end.second+offset_y;
//         zx.second = end.second-offset_y;
//         ys.second = start.second+offset_y;
//         yx.second = start.second-offset_y;
//     }
//     printPair(zs);
//     printPair(zx);
//     printPair(ys);
//     printPair(yx);
//     if(gt(offset,0)){
//         b1 = b + offset;
//         b2 = b - offset;
//     }
//     else{
//         b1 = b - offset;
//         b2 = b + offset;
//     }
//     k2 = (double)(-1/k1);
//     b3 = zs.second - k2*zs.first;
//     b4 = ys.second - k2*ys.first;
//     // if (gt(end.second, start.second)){
//     //     b3 = end.second - k2 * (end.first);
//     //     b4 = start.second - k2 * (start.first);
//     // }
//     // else{
//     //     b3 = end.second - k2 * (end.first);
//     //     b4 = start.second - k2 * (start.first);
//     // }
//     if (state.FrameID == 10113)
//     {
//         cerr << "~~~~~~~~" << endl;
//         cerr << " offset = "<<offset<<" cos = "<<cos(Pi-atan(k))<<endl;
//         cerr << " y = " << k1 << "*x"
//              << "+" << b1 << endl;
//         cerr << " y = " << k1 << "*x"
//              << "+" << b2 << endl;
//         cerr << " y = " << k2 << "*x"
//              << "+" << b3 << endl;
//         cerr << " y = " << k2 << "*x"
//              << "+" << b4 << endl;
//     }
//     for(int i = 0; i<101; i++){
//         for(int j=0; j<101; j++){
//             if(check_fall_into_scope(k1,b1,b2,k2,b3,b4,pair<double,double>(j*0.5,i*0.5))){
//                 if(state.FrameID == 10113){
//                     cerr<<"i = "<<i;
//                     cerr<<"j = "<<j<<endl;
//                 }
//                 if(wail[i][j]==-2){
//                     if(state.FrameID == 10113) cerr<<"~~~~~~~~"<<endl;
//                     return false;
//                 }
//             }
//         }
//     }
//     if(state.FrameID == 10113) cerr<<"~~~~~~~~"<<endl;
//     return true;

// }
// void init_dis(){
//     int i,j;
//     for(i=0;i<10000;i++){
//         for(j=i;j<10000;j++){
//             if (i == j)
//                 dis[j][i] = 0;
//             else{
//                 if (check_barrier(i,j,0)){
//             //         // cerr<<"cc\n";
//                     dis[i][j] = calcuDis(panes[i].pos, panes[j].pos);
//                     dis[j][i] = dis[i][j];
//                     // if(j==9999){
//                     //     cerr<<i<<' '<<j<<' '<<dis[i][j]<<"\n";
//                     //     dis[j][i] = dis[i][j];
//                     // }
//                     target_sequence[i][j] = j;

//                     target_sequence[j][i] = i;
//                 }
//                 else{
//                     dis[i][j] = 1000;
//                     dis[j][i] = dis[i][j];
//                     target_sequence[i][j] = -1;
//                     target_sequence[j][i] = -1;
//                 }
//             //     // cerr<<"bb\n";
//             }
//             // if(i>=3728)
//             // cerr<<"j = "<<j<<"\n";
//         }
//         cerr<<i<<"\n";
//     }
// }

// void floyd(){
//     // init_dis();
//     cerr<<"AA\n";
//     // for(int i = 0;i<10000;i++){
//     //     for(int j = 0;j<10000;j++){
//     //         for(int k =j+1 ;k<10000;k++){
//     //             if (gt(dis[j][k],dis[j][i]+dis[i][k])){
//     //                 dis[j][k] = dis[j][i]+dis[i][k];
//     //                 dis[k][j] = dis[j][k];
//     //                 target_sequence[j][k] = target_sequence[j][i];
//     //                 target_sequence[k][j] = target_sequence[k][i];
//     //             }
//     //         }
//     //     }
//     // }

// }
// void print_queue(){
//     for(int i=0;i<10000;i++){
//         for(int j = 0; j<10000;j++){
//             cerr<<i<<' '<<j<<' '<<dis[i][j]<<"\n";
//             cerr<<target_sequence[i][j];
//         }
//         cerr<<"\n";
//     }
// }
//  bool check_side(int i,int j,int flag,int is_take){
//     int count1,count2,k;
//     j = j + flag;
//     count1=99-j+1;
//     count2=99-j+1;
//     for(k=j;k<99;k++){
//         if (count1 == (99-j+1) && graph[i - 1][k] != graph[i - 1][k+1]){
//             count1 = k-j+1;
//         }
//         if (count2 == (99-j+1) && graph[i][k] != graph[i][k + 1])
//         {
//             count2 = k - j + 1;
//         }
//         if(count1!=(99-j+1)&&count2!=(99-j+1))break;
//     }
//     // if(graph[i-1][j]==21)
//         // cerr<<" abs(count1-count2) "<<i<<' '<<j<<' '<<abs(count1-count2)<<' '<<count1<<' '<<count2<<"\n";
//     if(count1>(is_take+3)&&count2>(is_take+3)){
//         if(abs(count1-count2)>(is_take+2)){
//             // if(graph[i-1][j]==21)
//                 // cerr<<"kkk"<<' '<<i<<' '<<j<<"\n";
//             return false;
//         }
//     }
//     else{
//         if(abs(count1-count2)>0){
//             // if(graph[i-1][j]==21)
//                 // cerr<<"kkk"<<' '<<i<<' '<<j<<"\n";
//             return false;
//         }
//     }
//     // cerr<<"flag = "<<flag<<"\n";
//     if(flag == 0){
//         count1 = j;
//         for (k = j; k >=1; k--)
//         {
//             if (count1 == j && graph[i - 1][k-1] != graph[i - 1][k])
//             {
//                 count1 = j - k + 1;
//                 break;
//             }
//         }
//         // if(graph[i-1][j]==21)
//             // cerr<<" count1 "<<i<<' '<<count1<<"\n";
//         if(count2>(is_take+3)){
//             if(count1>(is_take+2))return false;
//         }
//         else{
//             if(count1>0)return false;
//         }
//     }
//     return true;
//  }
//  pair<double, double> check_wail_change(int i, int j, int type)
//  {
//     double x, y;
//     int flag=0;
//     if (graph[i][j] != type)
//     {
//         for(int k = j;k<99;k++){
//             if(graph[i][k]!=-2){
//                 if(graph[i][k]!=type){
//                     break;
//                 }
//                 else{
//                     j= k+1;
//                     flag = 1;
//                     break;
//                 }
//             }
//         }
//         if(flag == 0){
//             for(int k = j;k>0;k--){
//                 if(graph[i][k]!=-2){
//                     if(graph[i][k]!=type){
//                         break;
//                     }
//                     else{
//                         j= k-1;
//                         flag = 1;
//                         break;
//                     }
//                 }
//             }
//         }
//     }
//     else flag = 1;
//     // if(flag == 0)cerr<<"can not find in this level\n";
//     x = j * 0.5 + 0.25;
//     y = i * 0.5 + 0.25;
//     // cerr<<"virtual_target_type "<<i<<' '<<j<<' '<<x<<' '<<y<<' '<<graph[i][j]<<"\n";
//     return pair<double, double>(x, y);
//  }

// void deal_graph(){
//     int count1,count2;
//     for(int i = 0;i<2;i++){
//         for(int j=0;j<100;j++){
//             for(int k =0 ;k<100;k++){
//                 graphs[i][j][k]=graph[j][k];
//             }
//         }
//     }
//     for(int i=0;i<100;i++){
//         count1 = 0;
//         count2 = 0;
//         for(int j=0;j<100;j++){
//             if(graphs[0][i][j]!=-2){
//                 count1++;
//             }
//             if(count1>0 && graphs[0][i][j] == -2){
//                 if(count1 < 2 )
//                     graphs[0][i][j-1]=-2;
//                     else count1 = 0;
//             }
//             if(graphs[1][i][j]!=-2){
//                 count2++;
//             }
//             if(count2>0 && graphs[1][i][j] == -2){
//                 if(count2 < 3 ){
//                     graphs[1][i][j-1]=-2;
//                     graphs[1][i][j-2]=-2;
//                 }
//                 else count2 = 0;
//             }
//         }
//     }
//     for(int j=0;j<100;j++){
//         count1 = 0;
//         count2 = 0;
//         for(int i=0;i<100;i++){
//             if(graphs[0][i][j]!=-2){
//                 count1++;
//             }
//             if(count1>0 && graphs[0][i][j] == -2){
//                 if(count1 < 2 )
//                     graphs[0][i-1][j]=-2;
//                     else count1 = 0;
//             }
//             if(graphs[1][i][j]!=-2){
//                 count2++;
//             }
//             if(count2>0 && graphs[1][i][j] == -2){
//                 if(count2 < 3 ){
//                     graphs[1][i-1][j]=-2;
//                     graphs[1][i-2][j]=-2;
//                 }
//                 else count2 = 0;
//             }
//         }
//     }
//     divide_space(0);
//     divide_space(1);
//     analyze_space(0);
//     analyze_space(1);
// }

// void divide_space(int is_take){
//     int i,j,k,type = 0,count1,count2,count;
//     int flag =0;
//     double first,second;
//     pair<double,double>temp;
//     for(i=0;i<100;i++){
//         for(j=0;j<100;j++){
//             // cerr << "ffff" << endl;
//             if(graphs[is_take][i][j]==-2)continue;
//             flag = 0;
//             // cerr<<"aaaa\n";
//             if(i==0||graphs[is_take][i-1][j]==-2){
//                 // cerr<<"bbbb\n";
//                 if(j==0||graphs[is_take][i][j-1]==-2){
//                     if (graphs[is_take][i][j - 1] == -2 &&j<98){
//                         if (graphs[is_take][i - 1][j + 1] != -2 ){ 
//                             flag = 1;
//                             if(check_side(i,j,1,is_take)){
//                                 graphs[is_take][i][j] = graphs[is_take][i - 1][j + 1];
//                                 continue;
//                             }
//                         }
//                         else if (graphs[is_take][i-1][j +2] != -2 ){
//                             flag = 2;
//                             if (check_side(i, j, 2,is_take))
//                             {
//                                 graphs[is_take][i][j] = graphs[is_take][i - 1][j + 2];
//                                 continue;
//                             }
//                         }
//                     }
//                     // cerr<<"111\n";
//                     graphs[is_take][i][j] = type;
//                     // cerr<<type<<"\n";
//                     type_area temp;
//                     temp.type = type;
//                     temp.height=1;
//                     types[is_take].push_back(temp);
//                     type ++;
//                 }
//                 else{
//                     graphs[is_take][i][j]=graphs[is_take][i][j-1];
//                 }
//             }
//             else{
//                 if(j==0||graphs[is_take][i][j-1]==-2){
//                     if(graphs[is_take][i-1][j] != -2){
//                         if(check_side(i,j,0,is_take)){
//                             graphs[is_take][i][j] = graphs[is_take][i - 1][j];
//                             types[is_take][graphs[is_take][i][j]].height++;
//                         }
//                         else{
//                             graphs[is_take][i][j]=type;
//                             type_area temp;
//                             temp.type = type;
//                             temp.height=1;
//                             types[is_take].push_back(temp);
//                                 // cerr<<type<<"\n";
//                             type ++;
//                         }
//                     }
//                 }
//                 else{
//                     if(graphs[is_take][i][j-1]!=graphs[is_take][i-1][j] && graphs[is_take][i][j-1] != -2){
//                         graphs[is_take][i][j]=graphs[is_take][i][j-1];
//                     }
//                     else graphs[is_take][i][j]=graphs[is_take][i-1][j];
//                 }
//             }
//         }
//         // for(j=0;j<100;j++){
//         //     fprintf(stderr,"%4d",graph[i][j]);
//         // }
//         // cerr<<"\n";
//     }
//     for(i=99;i>=0;i--){
//         for(j=0;j<100;j++){
//             fprintf(stderr,"%4d",graphs[is_take][i][j]);
//             // cerr<<graph[i][j]<<' ';
//         }
//         cerr<<"\n";
//     }
//     cerr<<"\n";
// }

// void analyze_space(int is_take){
//     int i,j,k,type = 0,count1,count2,count;
//     int flag =0;
//     double first,second;
//     pair<double,double>temp;
//     for(i=0;i<100;i++){
//         for(j=0;j<100;j++){
//             if(graphs[is_take][i][j]==-2)continue;
//             if(i>0&& graphs[is_take][i][j]!=graphs[is_take][i-1][j]&& graphs[is_take][i-1][j]!=-2){
//                 if(j==0 || graphs[is_take][i][j] !=graphs[is_take][i][j-1]|| graphs[is_take][i-1][j] !=graphs[is_take][i-1][j-1]){
//                     count = 1;
//                     for(k=j+1;k<100;k++){
//                         // cerr<<graph[i][k]<<' '<<graph[i][k-1]<<' '<<graph[i-1][k]<<' '<<graph[i-1][k-1]<<"\n";
//                         if(graphs[is_take][i][k] !=graphs[is_take][i][k-1]|| graphs[is_take][i-1][k] != graphs[is_take][i-1][k-1]) break;
//                         count ++;
//                     }
//                     // cerr<<"count = "<<count<<"\n";
//                     if(count>(is_take+2)){
//                         first = (j+count/2)*0.5+0.25;
//                         // cerr<<"type = "<<graph[i][j]<<"\n";
//                         // cerr<<"type = "<<graph[i-1][j]<<"\n";
//                         if(types[is_take][graphs[is_take][i][j]].height>2){
//                             second = ((i+1)*0.5+0.25);
//                             temp = check_wail_change(i+1,(j + count / 2), graphs[is_take][i][j]);
//                         }
//                         else {
//                             second = ((i)*0.5+0.25);
//                             temp = check_wail_change(i, (j + count / 2), graphs[is_take][i][j]);
//                         }
//                         if (types[is_take][graphs[is_take][i][j]].entrance.count(graphs[is_take][i-1][j]) == 0)
//                         {
//                             types[is_take][graphs[is_take][i][j]].entrance.insert({graphs[is_take][i-1][j],temp});
//                         }
//                         if(types[is_take][graphs[is_take][i-1][j]].height>2){
//                             second = ((i-2)*0.5+0.25);
//                             temp = check_wail_change(i - 2, (j + count / 2), graphs[is_take][i-1][j]);
//                         }
//                         else{
//                             second = ((i-1)*0.5+0.25);
//                             temp = check_wail_change(i - 1, (j + count / 2), graphs[is_take][i-1][j]);
//                         }
//                         if(types[is_take][graphs[is_take][i-1][j]].entrance.count(graphs[is_take][i][j])==0){
//                             types[is_take][graphs[is_take][i-1][j]].entrance.insert({graphs[is_take][i][j],temp});
//                         }
//                     }
//                     else{
//                         // cerr<<"graph[i][j] type = "<<graph[i][j]<<"graph[i-1][j] = " <<graph[i-1][j]<<"\n";
//                         types[is_take][graphs[is_take][i-1][j]].entrance.erase(graphs[is_take][i][j]);
//                         types[is_take][graphs[is_take][i][j]].entrance.erase(graphs[is_take][i-1][j]);
//                     }
//                     // cerr<<"count = "<<count<<"\n";
//                 }
//                 // if(types[graph[i][j]].entrance.count(graph[i-1][j])==0){
//                 //     types[graph[i][j]].entrance.insert({graph[i-1][j],pair<double,double>(0,0)});
//                 // }
//                 // if(types[graph[i-1][j]].entrance.count(graph[i][j])==0){
//                 //     types[graph[i-1][j]].entrance.insert({graph[i][j],pair<double,double>(0,0)});
//                 // }
//             }
//         }
//         // cerr<<i<<"\n";
//     }
//     // for(i=0;i<types.size();i++){
//     //     cerr<<"type : "<<types[i].type<<' '<<types[i].entrance.size()<<"\n";
//     //     for (auto iter = types[i].entrance.begin(); iter != types[i].entrance.end(); ++iter) {
//     //         cerr << iter->first << ' '<<iter->second.first<<' '<<iter->second.second<<' ';
//     //     }
//     //     cerr<<"\n";
//     // }
//     floyd_area(is_take);
//     // for(int i=0;i<types.size();i++){
//     //     for(int j=0;j<types.size();j++){
//     //         cerr<<i<<' '<<j<<' '<< "dis = "<<dis_area[i][j]<<"\n";
//     //     }
//     // }
//     // print_target(0, 93);
//     studio_distance(is_take);
// }

// void print_target(int i, int j,int is_take) {
//     int k;
//     int start = i;
//     double dist=0;
//     cerr<<i;
//     if(eq(dis_area[is_take][i][j], 1000)) return;
//     while(i != j) {
//         k = target_sequence[is_take][i][j];
//         cerr<<"->"<<k;
//         if(k==j) dist += dis_area[is_take][i][k];
//         else
//             dist += dis_area[is_take][i][k] + calcuDis(types[is_take][k].entrance[i], types[is_take][k].entrance[target_sequence[is_take][k][j]]);
//         // cerr<<"dist "<<start<<"->"<<k<<" = "<<dist<<"\n";
//         i = k;
//     }
//     // cerr<<"\n";
//     // cerr<<dist<<"*"<<dis_area[start][j]<<"\n";
// }
// void init_area(int is_take){
//     for(int i=0;i<types[is_take].size();i++){
//         for(int j=i+1;j<types[is_take].size();j++){
//             if(types[is_take][i].entrance.count(types[is_take][j].type)!=0){
//                 dis_area[is_take][i][j]=calcuDis(types[is_take][i].entrance[types[is_take][j].type],types[is_take][types[is_take][j].type].entrance[i]);
//                 dis_area[is_take][j][i]=dis_area[is_take][i][j];
//                 target_sequence[is_take][i][j]=j;
//                 target_sequence[is_take][j][i]=i;
//             }
//             else{
//                 dis_area[is_take][i][j]=1000;
//                 dis_area[is_take][j][i]=dis_area[is_take][i][j];
//                 target_sequence[is_take][i][j]=-1;
//                 target_sequence[is_take][j][i]=-1;
//             }
//         }
//     }
// }
// void floyd_area(int is_take){
//     double dist;
//     init_area(is_take);
//     for(int k=0;k<types[is_take].size();k++){
//         for(int j= 0;j<types[is_take].size();j++){
//             for(int i=0;i<types[is_take].size();i++){
//                 if(j==k || i==k || i==j)continue;
//                 if(lt(dis_area[is_take][i][k], 1000) && lt(dis_area[is_take][k][j], 1000)){
//                     // cerr<<j<<"\n";
//                     // printPair(types[i].entrance[j]);
//                     // cerr<<k<<"\n";
//                     // printPair(types[i].entrance[k]);
                    
//                     dist = dis_area[is_take][i][k] + dis_area[is_take][k][j] + calcuDis(types[is_take][k].entrance[target_sequence[is_take][k][i]], types[is_take][k].entrance[target_sequence[is_take][k][j]]);
//                     if(gt(dis_area[is_take][i][j], dist)) {
//                         dis_area[is_take][i][j] = dis_area[is_take][j][i] = dist;
//                         target_sequence[is_take][i][j] = target_sequence[is_take][i][k];
//                         target_sequence[is_take][j][i] = target_sequence[is_take][j][k];
//                     }


//                     // dist = calcuDis(types[i].entrance[j],types[i].entrance[k]);
//                     // // cerr<<"dist = "<<dist<<"\n";
//                     // if(gt(dis_area[j][k],dis_area[j][i]+dis_area[i][k]+dist)){
//                     //     dis_area[j][k] = dis_area[j][i]+dis_area[i][k]+dist;
//                     //     target_sequence[j][k]=target_sequence[j][i];
//                     // }
//                 }
//             }
//         }
//     }
// }
// void studio_distance(int is_take){
//     int row,col;
//     int x, y;
//     for(int i=0;i<studios.size();i++){
//         col = (studios[i].pos.first-0.25)/0.5;
//         row = (studios[i].pos.second-0.25)/0.5;
//         studios[i].studio_area_type[is_take] = graphs[is_take][row][col];
//         // cerr <<"kkk"<<studios[i].pos.first << ' ' << studios[i].pos.second << ' ' << col<<' '<<row<<' '<<studios[i].studio_area_type << endl;
//     }
//     for(int i=0;i<robots.size();i++){
//         col = (robots[i].pos.first-0.25)/0.5;
//         row = (robots[i].pos.second-0.25)/0.5;
//         // cerr<<col<<' '<<row<<"\n";
//         robots[i].robot_area_type[0] = graphs[0][row][col];
//     }
//     for(int i=0;i<studios.size();i++){
//         x = studios[i].studio_area_type[is_take];
//         cerr << studios.size()<<"\n";
//         for (int j = i + 1; j < studios.size(); j++)
//         {
//             y = studios[j].studio_area_type[is_take];
//             // cerr<<"a\n";
//             if(lt(dis_area[is_take][studios[i].studio_area_type[is_take]][studios[j].studio_area_type[is_take]],1000)){
//                 // cerr << i << ' ' << j << endl;
//                 // cerr<<x<<' '<<y<<"\n";
//                 if(studios[i].studio_area_type[is_take]!=studios[j].studio_area_type[is_take]){
//                     studio_dis[is_take][i][j] = calcuDis(studios[i].pos, types[is_take][x].entrance[target_sequence[is_take][x][y]]) + dis_area[is_take][x][y] + calcuDis(studios[j].pos, types[is_take][y].entrance[target_sequence[is_take][y][x]]);
//                 }
//                 else{
//                     studio_dis[is_take][i][j]=calcuDis(studios[i].pos,studios[j].pos);
//                 }
//                 // cerr << "b" << endl;
//             }
//             else{
//                 // cerr<<i<<' '<<j<<"\n";
//                 studio_dis[is_take][i][j] = 1000;
//                 // cerr << "c" << endl;
//             }
//             studio_dis[is_take][j][i] = studio_dis[is_take][i][j];
//             // cerr << j << endl;
//         }
//     }
//     for(int i=0;i<robots.size();i++){
//         x = robots[i].robot_area_type[0];
//         for(int j=0;j<studios.size();j++){
//             y = studios[j].studio_area_type[is_take];
//             if(lt(dis_area[is_take][x][y],1000)){
//                 if(x != y){
//                     init_robot_dis[i][j] = calcuDis(robots[i].pos, types[is_take][x].entrance[target_sequence[is_take][x][y]]) + dis_area[is_take][x][y] + calcuDis(studios[j].pos, types[is_take][y].entrance[target_sequence[is_take][y][x]]);
//                 }
//                 else{
//                     init_robot_dis[i][j]=calcuDis(robots[i].pos,studios[j].pos);
//                 }
//             }
//             else{
//                 init_robot_dis[i][j] = 1000;
//             }
//         }
//     }
//     // for (int i = 0; i < studios.size(); i++)
//     // {
//     //     for(int j=0;j<studios.size();j++){
//     //         cerr<<i<<' '<<j<<' '<< "studio_distance = "<<studio_dis[i][j]<<"\n";
//     //     }
//     // }

//     // cerr<<"type"<<robots[0].robot_area_type<<"\n";

// }

void init_trans(){
    for(int i=0;i<100;i++){
        for(int j=0;j<100;j++){
           graph_trans[i][j]=( graph[i][j]==-2?-2:0);
        }
    }
}//将原来的地图中不是-2的部分全部更改为0
double Angle_conversion(double angle){
    return fabs(angle)/Pi;
}//将角度转换为距离
bool check_4(int i,int j){
    if(i<0||j<0||i>99||j>99)return false;
    if(i+1>99||j-1<0)return false;
    return graph_trans[i][j]!=-2&&graph_trans[i][j-1]!=-2&&graph_trans[i+1][j-1]!=-2&&graph_trans[i+1][j]!=-2;
}//检查坐标i,j是否是一个四个格子的合法点
pair<int,pair<double,double>> check_8(int i,int j){
    if(check_4(i,j)&&check_4(i,j+1)&&check_4(i-1,j)&&check_4(i-1,j+1)){
        return {1,make_pair<double,double>(0.5*j+0.25,0.5*i+0.25)};
    }else if((!check_4(i,j))&&check_4(i,j+1)&&check_4(i-1,j)&&check_4(i-1,j+1)){
        return {2,make_pair<double,double>(0.5*j+0.47,0.5*i+0.03)};
    }else if(check_4(i,j)&&check_4(i,j+1)&&(!check_4(i-1,j))&&check_4(i-1,j+1)){
        return {3,make_pair<double,double>(0.5*j+0.47,0.5*i+0.47)};
    }else if(check_4(i,j)&&(!check_4(i,j+1))&&check_4(i-1,j)&&check_4(i-1,j+1)){
        return {4,make_pair<double,double>(0.5*j+0.03,0.5*i+0.03)};
    }else if(check_4(i,j)&&check_4(i,j+1)&&check_4(i-1,j)&&(!check_4(i-1,j+1))){
        return {5,make_pair<double,double>(0.5*j+0.03,0.5*i+0.47)};
    }else{
        return {0,make_pair<double,double>(0,0)};
    }
    
}//检查坐标i,j是否是一个八个格子的合法点
void Translation_graph_no(){
    for(int i=0;i<100;i++){
        for(int j=0;j<100;j++){
            if(check_4(i,j)){
                int id=100*i+j;
                auto pos=make_pair<double,double>(0.5*j,0.5*i+0.5);
                exist_id[0][id]=pos;
            }
        }
    }
    for(int t=0;t<studios.size();t++){
        int id1=studios[t].node_id;
        int i1=id1/100,j1=id1%100;
        // cerr<<"12"<<endl;
        for(int i=i1-1;i<=i1+1;i++){
            for(int j=j1-1;j<=j1+1;j++){
                if(i<0||j<0||i>99||j>99)continue;
                if(i==i1&&j==j1)continue;
                int tmpId=i*100+j;
                bool isSlope=  (fabs(i1-i)+fabs(j1-j)==2)?true:false;
                bool con1= isSlope?check_slope_studios(tmpId,id1):true;
                // if(t==12) cerr<<id1<<" "<<tmpId<<"-"<<isSlope<<"-"<<con1<<"-"<<exist_id[0].count(tmpId)<<" "<<((!isSlope||con1)&&exist_id[0].count(tmpId))<<endl;
                if((!isSlope||con1)&&exist_id[0].count(tmpId)){
                    // if(t==12)cerr<<tmpId<<endl;
                    double dis= (abs(i-(studios[t].node_id/100))+abs(j-(studios[t].node_id%100))==2)?pow(2,0.5):1;
                    studio_edge[0][t].push_back(Graph_node(tmpId,dis,studios[t].node_id));
                 }
            }
        }
    }
}
void Translation_graph_has(){
    for(int i=0;i<100;i++){
        for(int j=0;j<100;j++){
            auto tmp=check_8(i,j);
            auto pos=tmp.second;
            int id=100*i+j;
            if(tmp.first!=0){
                exist_id_type[1][id]=tmp.first;
                exist_id[1][id]=tmp.second; 
            }
        }
    }
    for(int t=0;t<studios.size();t++){
        int id1=studios[t].node_id;
        int i1=id1/100,j1=id1%100;
        for(int i=i1-1;i<=i1+1;i++){
            for(int j=j1-1;j<=j1+1;j++){
                if(i<0||j<0||i>99||j>99)continue;
                if(i==i1&&j==j1)continue;
                int tmpId=i*100+j;
                bool isSlope=  (fabs(i1-i)+fabs(j1-j)==2)?true:false;
                bool con1= isSlope?check_slope(tmpId,id1):true;
                if((!isSlope||con1)&&exist_id[1].count(tmpId)){
                    double dis= (abs(i-(studios[t].node_id/100))+abs(j-(studios[t].node_id%100))==2)?pow(2,0.5):1;
                    studio_edge[1][t].push_back(Graph_node(tmpId,dis,studios[t].node_id));
                 }
            }
        }
    }    
}//转换机器人带物品的原始图
void getEdgeRalative(){
    for(auto& it:exist_id[0]){
        int idi=it.first/100;
        int idj=it.first-idi*100;
        for(int i=idi-1;i<=idi+1;i++){
            for(int j=idj-1;j<=idj+1;j++){
                if(i<0||j<0||i>99||j>99)continue;
                if(i==idi&&j==idj)continue;
                int tmpId=i*100+j;
                int ckeck_id=idi*100+j;
                // if(it.first==studios[0].node_id){
                //     cerr<<graph_edge[0][it.first].size()<<" "<<exist_id[0].count(tmpId)<<" "<<exist_id[0].count(ckeck_id)<<"\n";
                //     cerr<<idi<<"-"<<idj<<" "<<i<<"-"<<j<<"\n";
                // }
                // if((it.first==4656&&tmpId==4557) ||(it.first==4557&&tmpId==4656) ){
                //     cerr<<"****\n";
                //     cerr<<check_slope(tmpId,it.first)<<"\n";
                //     cerr<<idi<<" "<<j<<"\n";
                //     cerr<<graph_trans[idi][j+1]<<"\n";
                //     cerr<<"****\n";
                // }
                bool isSlope=  (fabs(i-idi)+fabs(j-idj)==2)?true:false;
                int SlopeCheckId1=idi*100+j;
                int SlopeCheckId2=i*100+idj;
                //不带货物时，不考虑是否在墙角。
                if(exist_id[0].count(tmpId)&&check_slope(tmpId,it.first) &&it.first!=tmpId){
                    // if((tmpId==5359&&it.first==5458)||(it.first==5359&&tmpId==5458))
                    // cerr<<"edge err -----------"<<endl;
                    double dis= (abs(i-idi)+abs(j-idj)==2)?pow(2,0.5):1;
                    graph_edge[0][it.first].push_back(Graph_node(tmpId,dis,it.first));
                }
            }
        }
    }
    for(auto& it:exist_id[1]){
        int idi=it.first/100;
        int idj=it.first-idi*100;
        for(int i=idi-1;i<=idi+1;i++){
            for(int j=idj-1;j<=idj+1;j++){
                if(i<0||j<0||i>99||j>99)continue;
                if(i==idi&&j==idj)continue;
                int tmpId=i*100+j;
                if(is_corner(it.first)||is_corner(tmpId)){
                    continue;
                }
                int ckeck_id=idi*100+j;
                bool isSlope= (fabs(i-idi)+fabs(j-idj)==2)?true:false;
                bool isInCorner=(exist_id_type[1][tmpId]>1||exist_id_type[1][it.first]>1);
                int SlopeCheckId1=idi*100+j;
                int SlopeCheckId2=i*100+idj;
                //带货物时，考虑是否在墙角。
                if(isInCorner&&isSlope&&(exist_id[1].count(SlopeCheckId1)||exist_id[1].count(SlopeCheckId2))){

                }else if(exist_id[1].count(tmpId)&&it.first!=tmpId&&check_slope(tmpId,it.first)){
                    double dis= (abs(i-idi)+abs(j-idj)==2)?pow(2,0.5):1;
                    graph_edge[1][it.first].push_back(Graph_node(tmpId,dis,it.first));
                }
            }
        }
    }
}

void init_vector() {
    // vector<int> next_node[50][2];//next_node[node_id][studio_id][2]:node_id去往studio_id工作台的下一个点
    // vector<double> dis_to_studios[50][2];
    for(int i = 0; i < studios.size(); ++i) {
        next_node[i][0].resize(10000, -1);
        next_node[i][1].resize(10000, -1);
        dis_to_studios[i][0].resize(10000, 0);
        dis_to_studios[i][1].resize(10000, 0);
    }
}

double calAngleToDis(int x, int y, int z) {
    if(x == y) return 0;
    if(y == z) return 0;

    Vec vec1 = Vec((y / 100) - (x / 100), (y % 100) - (x % 100));
    Vec vec2 = Vec((z / 100) - (y / 100), (z % 100) - (y % 100));

    double angle = acos(cos_t(vec1, vec2));
    return Angle_conversion(angle);
}

int transID(int from_id, int is_robot, int to_id) {
    return (from_id + is_robot * 50) * 54 + to_id;
}

void Dijkstra(int studio_id, int is_take) {
    priority_queue<Graph_node, vector<Graph_node>, cmp_Graph_node> q;
    int from, pre_id, num, i, to;
    double dis, new_dis, angle_sum;
    int s, danger_sum;
    int vis_node[10000];
    // int danger_node[10000];
    double angle_node[10000];

    s = studios[studio_id].node_id;

    bool cerr_flag = false;
    // if(studio_id == 0 && is_take == 0) cerr_flag = true;

    if(cerr_flag) {
        cerr<<"start-studio:"<<studio_id<<"\n";
    }

    for(i = 0; i < 10000; ++i) {
        vis_node[i] = 0;
        // danger_node[i] = 10000;
        dis_to_studios[studio_id][is_take][i] = 10000;
    }

    // q.push(Graph_node(s, 0, s, 0, 0));
    dis_to_studios[studio_id][is_take][s] = 0;
    next_node[studio_id][is_take][s] = s;
    vis_node[s] = 1;
    // danger_node[s] = 0;

    num = studio_edge[is_take][studio_id].size();
    for(i = 0; i < num; ++i) {
        to = studio_edge[is_take][studio_id][i].id;
        dis = studio_edge[is_take][studio_id][i].dis;
        q.push(Graph_node{to, dis, s, 0});
        next_node[studio_id][is_take][to] = s;
        dis_to_studios[studio_id][is_take][to] = dis;
        angle_node[to] = 0;
    }
    
    while(!q.empty()) {
        Graph_node now_node = q.top();
        q.pop();
        if(vis_node[now_node.id]) continue;
        from = now_node.id;
        dis = now_node.dis;
        pre_id = now_node.pre_id;
        vis_node[now_node.id] = 1;

        if(cerr_flag) 
            cerr<<"node_id:"<<from<<" dis:"<<dis<<" pre_id:"<<pre_id<<"danger_sum:"<<now_node.dangerous_sum<<"\n";


        num = graph_edge[is_take][from].size();
        if(cerr_flag) 
            cerr<<"edge-num:"<<num<<"\n";
        for(i = 0; i < num; ++i) {
            to = graph_edge[is_take][from][i].id;
            if(vis_node[to] || to == pre_id) continue;

            angle_sum = now_node.angle_sum + calAngleToDis(pre_id, from, to);
            new_dis = dis + graph_edge[is_take][from][i].dis;
            // danger_sum = now_node.dangerous_sum + dangerous_point[is_take].count(to);
            // cerr<<"to_id:"<<to<<" new-dis:"<<dis<<" old-dis:"<<dis_node[to]<<"\n";
            // if(danger_node[to] >= danger_sum && (lt(new_dis, dis_to_studios[studio_id][is_take][to]) || (eq(new_dis, dis_to_studios[studio_id][is_take][to]) && lt(angle_sum, angle_node[to])))) {
            if (lt(new_dis, dis_to_studios[studio_id][is_take][to]) || (eq(new_dis, dis_to_studios[studio_id][is_take][to]) && lt(angle_sum, angle_node[to]))) {
                q.push(Graph_node{to, new_dis, from, angle_sum, danger_sum});
                if(cerr_flag) cerr<<"update-to_id:"<<to<<" new-dis:"<<new_dis<<" old-dis:"<<dis_to_studios[studio_id][is_take][to]<<"angle"<<angle_sum<<" danger_sum:"<<danger_sum<<"\n";
                dis_to_studios[studio_id][is_take][to] = new_dis;
                next_node[studio_id][is_take][to] = from;
                angle_node[to] = angle_sum;
                // danger_node[to] = danger_sum;
            }
            // else if(is_take && eq(new_dis, dis_to_studios[to][studio_id][is_take]) && eq(angle_sum, angle_node[to])) {
            //     q.push(Graph_node{to, new_dis, from, angle_sum});
            //     if(cerr_flag) cerr<<"update-to_id:"<<to<<" new-dis:"<<new_dis<<" old-dis:"<<dis_to_studios[to][studio_id][is_take]<<"\n";
            //     dis_to_studios[to][studio_id][is_take] = new_dis;
            //     next_node[to][studio_id][is_take] = from;
            //     angle_node[to] = angle_sum;
            // }
        }
    }
}

int trans_pos_to_nodeID(pair<double, double> pos) {
    return (int)(pos.second / 0.5) * 100 + (int)(pos.first / 0.5);
}

pair<double, double> trans_nodeID_to_pos(int nodeID) {
    return make_pair((nodeID % 100) * 0.5 + 0.25, (nodeID / 100 * 0.5 + 0.25));
}

int trans_pos_to_nodeID(int robot_id) {
    return trans_pos_to_nodeID(robots[robot_id].pos);
}

// bool is_connected(int node_id_a, int node_id_b) {
//     return get_bar_num(node_id_a, node_id_b) == 0;
// }

// int get_bar_num(int node_id_a, int node_id_b) {
//     int x1, x2, y1, y2;
//     x1 = min(node_id_a / 100, node_id_b / 100);
//     x2 = max(node_id_a / 100, node_id_b / 100);
//     y1 = min(node_id_a % 100, node_id_b % 100);
//     y2 = max(node_id_a % 100, node_id_b % 100);
//     if(x2 == 0 && y2 == 0)
//         return bar_sum[x2][y2];
//     if(x2 == 0)
//         return bar_sum[x2][y2] - bar_sum[x2][y1];
//     if(y2 == 0)
//         return bar_sum[x2][y2] - bar_sum[x1][y2];
//     return bar_sum[x2][y2] - bar_sum[x2][y1] - bar_sum[x1][y2] + bar_sum[x1][y1];
// }

// void init_bar_sum() {
//     for(int i = 0; i < 100; ++i) {
//         // if(stu_id != 0 && get_bar_num(studios[stu_id].node_id, max((studios[stu_id].node_id / 100) - 1, 0) * 100 + max((studios[stu_id].node_id % 100) - 1, 0), 0)  == 0)
//         //         continue;
//         for(int j = 0; j < 100; ++j) { 
//             if( i == 0 && j == 0) {
//                 bar_sum[i][j] = (graph[i][j] == -2);
//                 // bar_sum[i][j][1] = (next_node[stu_id][1][i * 100 + j] == -1);
//             }
//             if(i > 0) {
//                 bar_sum[i][j] += bar_sum[i - 1][j];
//                 // bar_sum[i][j][1] += bar_sum[i - 1][j][1];
//             }
//             if(j > 0) {
//                 bar_sum[i][j] += bar_sum[i][j - 1];
//                 // bar_sum[i][j][1] += bar_sum[i][j - 1][1] - bar_sum[i - 1][j - 1][1];
//             }
//             if(i > 0 && j > 0) {
//                 bar_sum[i][j] -= bar_sum[i - 1][j - 1];
//                 // bar_sum[i][j][1] += bar_sum[i][j - 1][1] - bar_sum[i - 1][j - 1][1];
//             }
//         }
//     }
// }


void print_dijkstra(int studio_id, int is_take, int is_path) {
    for(int i = 0; i < 100; ++i) {
        for(int j = 0; j< 100; ++j) {
            if(is_path) {
                if(next_node[studio_id][is_take][(99-i) * 100 +j] == -1)
                    cerr<<"- ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-i) * 100 +j + 1)
                    cerr<<"→ ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-i) * 100 +j -1)
                    cerr<<"← ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-(i+1)) * 100 +j)
                    cerr<<"↓ ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-(i-1)) * 100 +j)
                    cerr<<"↑ ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-(i+1)) * 100 +j-1)
                    cerr<<"↙️ ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-(i+1)) * 100 +j+1) 
                    cerr<<"↘️ ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-(i-1)) * 100 +j+1) 
                    cerr<<"↗️ ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-(i-1)) * 100 +j-1) 
                    cerr<<"↖️ ";
                else if(next_node[studio_id][is_take][(99-i) * 100 +j] == (99-i) * 100 +j)
                    cerr<<"* ";
                else cerr<<(99-i) * 100 +j<<"-"<<next_node[studio_id][is_take][(99-i) * 100 +j]<<" ";
                    
            }
            else 
                cerr<<setw(10)<<dis_to_studios[studio_id][is_take][(99-i) * 100 +j]<<" ";
        }
        cerr<<"\n";
    }
}

bool is_corner(int id){
    int i=id/100;
    int j=id-i*100;
    bool leg1= (j==0||graph_trans[i][j-1]==-2)?true:false;
    bool leg2= (i==99||graph_trans[i+1][j]==-2)?true:false;
    bool leg3= (j==99||graph_trans[i][j+1]==-2)?true:false;
    bool leg4= (i==0||graph_trans[i-1][j]==-2)?true:false;
    return (leg1&&leg2) || (leg2&&leg3) || (leg3&&leg4) || (leg4&&leg1);
}//判断工作台是不是在墙角
void init_data(){
    init_trans();
    Translation_graph_no();
    Translation_graph_has();
    getEdgeRalative();
    // trans_studio_rob_toID();
    // cerr<<exist_id[0].count(5358)<<"-"<<exist_id[0].count(5458)<<"-"<<exist_id[0].count(5359)<<"-"<<exist_id[0].count(5459)<<endl;
    for(int j=0;j<100;j++){
        for(int i=0;i<100;i++){
            int id=i*100+j;
            sum_matrix[0][i][j]=(i-1>=0?sum_matrix[0][i-1][j]:0)+ (exist_id[0].count(id));
            sum_matrix[1][i][j]=(i-1>=0?sum_matrix[1][i-1][j]:0)+ (exist_id[1].count(id));
        }
    }
    // cerr<<exist_id[0].count(5358)<<"-"<<exist_id[0].count(5458)<<"-"<<exist_id[0].count(5459)<<"-"<<exist_id[0].count(5359)<<endl;
    // cerr<<check_slope(5359,5458)<<endl;
    get_point_type();
    // cerr<<exist_id[0].count(5358)<<"-"<<exist_id[0].count(5458)<<"-"<<exist_id[0].count(5359)<<"-"<<exist_id[0].count(5459)<<endl;
    for(int i=0;i<studios.size();i++){
        int id=studios[i].node_id;
        if(studios[i].has_suspicious_spots==1)
            exist_id[0][id]=studios[i].pos;
    }
    // cerr<<exist_id[0].count(5358)<<"-"<<exist_id[0].count(5458)<<"-"<<exist_id[0].count(5359)<<"-"<<exist_id[0].count(5459)<<endl;
    // cerr<<sum_matrix[0][54][58]<<"-"<<sum_matrix[0][52][58]<<" "<<sum_matrix[0][54][59]<<"-"<<sum_matrix[0][52][59]<<endl;

}
// void printMap(int f){
//     // for(int i=0;i<100;i++){
//     //     cerr<<i<<" ";
//     // }
//     cerr<<"\n";
//         for(int i=100;i>=0;i--){
//         for(int j=0;j<100;j++){
//             int id=i*100+j;
//             if(exist_id[f].count(id)){
//                 if(f==1)
//                 cerr<<check_8(i,j).first<<" ";
//                 else
//                 cerr<<1<<" ";
//             }else{
//                 cerr<<"-"<<" ";
//             }
//         }
//         cerr<<"\n";
//     }
// }
// void printEdge(int id){
//     vector<int> vis_rob_edge(100*100,-1);
//     // cerr<<studios[4].node_id/100<<"-"<<(studios[4].node_id-studios[4].node_id/100*100)<<"\n";
//     cerr<<studios[5].node_id<<"\n";
//     //unordered_map<int,vector<Graph_node>> graph_edge[2];
//     queue<int>Q;
//     Q.push(robots[id].node_id);
//     int cnt=0;
//     vis_rob_edge[robots[id].node_id]=0;
//     while(!Q.empty()){
//         int tmpId=Q.front();
//         Q.pop();
//         for(auto it:graph_edge[0][tmpId]){
//             if(vis_rob_edge[it.id]==-1){
//                  Q.push(it.id);
//                  vis_rob_edge[it.id]=(vis_rob_edge[tmpId]+1)%10;
//             }
            
//         }
//     }
//     int tarStu=4;
//     // vector<Graph_node> path = road[0][transID(3, 0, tarStu)];
//     // for(auto it:path){
//     //     int tmpId=it.id;
//     //     if(vis_rob_edge[tmpId]==-1){
//     //         cerr<<" 错误 \n";
//     //     }
//     //     vis_rob_edge[tmpId]=-7;
       
        
//     // }
//     // cerr<<"edge-print \n";
//     // for(int i=0;i<100;i++){
//     //     cerr<<i<<" ";
//     // }
//     // cerr<<"\n";
//     for(int i=99;i>=0;i--){
//     for(int j=0;j<100;j++){
//             int id=i*100+j;
//             if(vis_rob_edge[id]!=-1&&vis_rob_edge[id]!=-7&&id!=studios[tarStu].node_id){
//                 cerr<<vis_rob_edge[id]<<" ";
//             }else if(vis_rob_edge[id]==-7&&id!=studios[tarStu].node_id){
//                 cerr<<"+"<<" ";
//             }else if(id==studios[tarStu].node_id){
//                 cerr<<"^"<<" ";
//             }else {
//                 cerr<<"-"<<" ";
//             }
//         }
//         cerr<<"\n";
//     }
// }
bool check_slope(int id1,int id2){
    int y1=id1/100,y2=id2/100;
    int x1=id1%100,x2=id2%100;
    bool isSlope= (fabs(x1-x2)+fabs(y1-y2)==2)?true:false;
    if(!isSlope)return true;
    id1=y2*100+x1;
    id2=y1*100+x2;
    return exist_id[0].count(id1)&&exist_id[0].count(id2);
}

// void printPath(int from_id, int is_robot, int to_id, int is_take) {
//     int id, x, y;
//     // vector<Graph_node> path = road[is_take][transID(from_id, is_robot, to_id)];
//     // for(int i =0;i< path.size(); ++i) {
//     //     id = path[i].id;
//     //     x = id / 100;
//     //     y = id % 100;
//     //     cerr << "to:(" << x <<", "<<y<<")\n";
//     // }
//     // cerr<<"\n";
// }

// vector<int> get_future_node(int robot_id) {
//     vector<int> v;
//     int t = 2;
//     int node_id = robots[robot_id].node_id;
//     while(t--) {
//         node_id = next_node[robots[robot_id].target_id][(robots[robot_id].get_type != 0)][node_id];
//         v.emplace_back(node_id);
//     }
//     // cerr<<"kkk"<<v.size()<<"\n";
//     return v;
// }
// bool is_need_slow(Robot& robot,pair<double,double> pos,pair<double,double> pos1){
//     if(lt(pos1.first,0))return false;
//     auto p1=subVector(robot.virtual_pos,robot.pos);
//     auto p2=subVector(pos,robot.pos);
//     auto p3=subVector(pos1,robot.pos);
//     auto v=robot.xy_pos;
//     Vec v_p1(p1);
//     Vec v_p2(p2);
//     Vec v_p3(p3);
//     Vec v_v(v);
//     double cmpNums1= cos_t(v_v,v_p1),cmpNums2= cos_t(v_v,v_p2),cmpNums3= cos_t(v_v,v_p3);
//     // if(contr_print_flag&&state.FrameID>=1354&&state.FrameID<=1450&&robot.id==0){
//     //     cerr<<"cmpAngle "<<cmpNums1<<" "<<cmpNums2<<"\n";
//     // }
//     if(ge(cmpNums1,0.9)&&ge(cmpNums2,0.8)&&ge(cmpNums3,0.8))return false;
//     return true;
// }
void adjust_virtual_pos(Robot& robot){
    if(exist_id_type[1][robot.virtual_id]!=1){
        return;
    }
    int now_j= robot.pos.first/0.5;
    int now_i= robot.pos.second/0.5;
    int tar_i=robot.virtual_id/100;
    int tar_j=robot.virtual_id-100*tar_i;
    if(fabs(now_i-tar_i)+fabs(now_j-tar_j)!=2)return;
    int tmpID1=now_i*100+tar_j,tmpID2=tar_i*100+now_j;
    if(robot.get_type==0){
    //    int tar1= robot.cnt_tar;
    //    if(exist_id[1].count(tar1)==0){
    //     for(int i=-1;i<=1;i++){
    //         int id1=tar1+100*i;
    //         int id2=tar1+i;
    //         if(exist_id[1].count(id1)){
    //         robot.virtual_pos.first=exist_id[1][id1].first;
    //         robot.virtual_pos.second=exist_id[1][id1].second;
    //         // robot.virtual_id=getPosID(robot.virtual_pos);
    //         }else if(exist_id[1].count(id2)){
    //         robot.virtual_pos.first=exist_id[1][id2].first;
    //         robot.virtual_pos.second=exist_id[1][id2].second;
    //         // robot.virtual_id=getPosID(robot.virtual_pos);
    //         }
    //         // cerr<<robot.id<<" adust1"<<endl;
    //     }
    //    }else{
    //     // cerr<<"原pos ";
    //     // printPair(robot.virtual_pos);
    //     // cerr<<robot.id<<" adust2"<<endl;
    //     robot.virtual_pos.first=exist_id[1][tar1].first;
    //     robot.virtual_pos.second=exist_id[1][tar1].second;
    //     // robot.virtual_id=getPosID(robot.virtual_pos);
    //    }
       return ;
    }
    double arr[][2]={{0,0},{0,0},{0,-0.2},{0.2,0},{0,-0.2},{-0.2,0}};
    if((exist_id_type[1].count(tmpID1)==0||exist_id_type[1].count(tmpID2)==0) ||exist_id_type[1][tmpID1]!=1&&exist_id_type[1][tmpID2]!=1){
        return;
    }else if(exist_id_type[1][tmpID1]!=1){
        int type=exist_id_type[1][tmpID1];
        robot.virtual_pos.first=exist_id[1][robot.virtual_id].first+arr[type][0];
        robot.virtual_pos.second=exist_id[1][robot.virtual_id].second+arr[type][1];
    }else if(exist_id_type[1][tmpID2]!=1){
        int type=exist_id_type[1][tmpID2];
        robot.virtual_pos.first=exist_id[1][robot.virtual_id].first+arr[type][0];
        robot.virtual_pos.second=exist_id[1][robot.virtual_id].second+arr[type][1];
    }
    return ;
}
void adjust_virtual_pos_total(Robot& rob){
    if(rob.cnt_tar==-1){
        init_rob_status(rob);
    }
    print_cerr_flag_ta=false;
    int istake=rob.get_type==0?0:1;
    if(!rob.is_new_tar_ing&&!rob.need_adjust_statues&&
    (!check_can_arrival(istake,rob.close_node,rob.cnt_tar))){
        // if(rob.id==0){
        //     cerr<<"重设状态"<<endl;
        //     cerr<<illegal_point[istake][rob.node_id]<<endl;
        //     cerr<<rob.node_id<<endl;
        // }
        int next_tar=next_node[rob.target_id][istake][rob.close_node];
        if(empty_pos(rob)&&next_tar!=-1){
            rob.virtual_pos=exist_id[istake][next_tar];
            rob.virtual_id=next_tar;
            rob.cnt_tar=next_tar;
            // cerr<<"使用空白点代替了重置状态"<<endl;
            // cerr<<"time"<<state.FrameID<<endl;
            // cerr<<"机器人编号"<<rob.id<<endl;
        }else{
            rob.is_new_tar_ing=true;
            rob.need_adjust_statues=true;
        }        
    }
    if(rob.need_collison){
            rob.need_adjust_statues=false;
            rob.adjust_pos=false;
            rob.adjust_w=false;
            rob.need_slow=false;
            rob.target_id_pre=rob.target_id; 
            rob.is_new_tar_ing=false; 
            setVirPos(rob);
    }
    else if(state.FrameID!=1&&(rob.real_get_type!=rob.get_type)){
        // if(rob.id==0)cerr<<"target变化导致重新调整"<<endl;
        int next_tar=next_node[rob.target_id][istake][rob.close_node];
        if(empty_pos(rob)){
            // cerr<<"使用空白点代替了工作台重置状态"<<endl;
            // cerr<<"time"<<state.FrameID<<endl;
            // cerr<<"机器人编号"<<rob.id<<endl;
            // cerr<<rob.target_id_pre<<"---->"<<rob.target_id<<endl;
            rob.virtual_pos=exist_id[istake][next_tar];
            rob.virtual_id=next_tar;
            rob.cnt_tar=next_tar;
            rob.target_id_pre=rob.target_id;
        }else{
            rob.need_adjust_statues=true;
            rob.adjust_pos=true;
            rob.adjust_w=true;
            rob.need_slow=true;
            rob.target_id_pre=rob.target_id;
            init_rob_status(rob);
        }
        // if(rob.id==2)cerr<<0<<"\n";
    }else if(rob.need_adjust_statues){
        PayLoad pay=calPayload(rob,rob.virtual_pos);
        if(gt(1,pay.distance)){
            init_rob_status(rob);
        }
        if(lt(return_v(rob),0.8)&&rob.need_slow&&!rob.is_new_tar_ing){
            if(rob.id==0&&print_cerr_flag_ta){
                cerr<<"状态调整正式开始\n";
                cerr<<"v "<<return_v(rob)<<"\n";
                printPair(rob.xy_pos);
                cerr<<"调整至标准坐标\n";
                printPair(exist_id[rob.get_type==0?0:1][rob.node_id]);
                cerr<<" ---"<<rob.node_id<<"\n";
            }
            init_rob_status(rob);
            rob.need_slow=false;
            
        }else if(gt(return_v(rob),0.8)&&rob.adjust_pos&&!rob.is_new_tar_ing){
            rob.need_adjust_statues=true;
            rob.adjust_pos=true;
            rob.adjust_w=true;
            rob.need_slow=true;
            rob.target_id_pre=rob.target_id;
            
            if(rob.id==0&&print_cerr_flag_ta){
                cerr<<"调整速度\n";
            }
            
            // if(rob.id==2)cerr<<1<<endl;
        }else if(!rob.adjust_pos&&!rob.is_new_tar_ing){
            setVirPos(rob);
        }else if(rob.is_new_tar_ing){
            rob.need_adjust_statues=true;
            rob.adjust_pos=true;
            rob.adjust_w=true;
            rob.need_slow=true;
            rob.target_id_pre=rob.target_id; 
            rob.is_new_tar_ing=false; 
        if(rob.id==0&&print_cerr_flag_ta){
                cerr<<"调整new_tar_\n";
            }
            // if(rob.id==2)cerr<<2<<"\n";
        }
    }else{
        setVirPos(rob);
    }
    // print_cerr_flag_ta=true;
    adjust_virtual_pos(rob);
}
bool check_can_arrival_z(int id1,int id2){
    int i1=min(id1/100,id2/100),i2=max(id1/100,id2/100);
    int j1=min(id1-id1/100*100,id2-id2/100*100),j2=max(id1-id1/100*100,id2-id2/100*100);
    // if(!eq(i1-i2,0)&&!eq(j1-j2,0)){
    //     i1=max(i1-1,0);
    //     i2=min(i2+1,99);
    //     j1=max(j1-1,0);
    //     j2=min(j2+1,99);
    // }
    if(i1<0||j1<0||i2<0||j2<0){
        // cerr<<" 错误\n";
        return false;
    }
    for(int j=j1;j<=j2;j++){
        int tmp=sum_matrix[0][i2][j]-(i1>0?sum_matrix[0][i1-1][j]:0);
        if(tmp<(i2-i1+1)){
            
            return false;
        }
    }

    return true;    
}
bool check_can_arrival(int istake,int id1,int id2){

    if(illegal_point[istake][id1]||illegal_point[istake][id2]){
        return false;
    }
    int i1=min(id1/100,id2/100),i2=max(id1/100,id2/100);
    int j1=min(id1%100,id2%100),j2=max(id1%100,id2%100);
    // if(!eq(i1-i2,0)&&!eq(j1-j2,0)){
    //     i1=max(i1-1,0);
    //     i2=min(i2+1,99);
    //     j1=max(j1-1,0);
    //     j2=min(j2+1,99);
    // }
    if(i1<0||j1<0||i2<0||j2<0){
        // cerr<<" 错误\n";
        return false;
    }
    for(int j=j1;j<=j2;j++){
        int tmp=sum_matrix[istake][i2][j]-(i1-1>=0?sum_matrix[istake][i1-1][j]:0);
        if(tmp<(i2-i1+1)){
            return false;
        }
    }

    return true;

}
// set<int> getEqID(int istake,int id1) {
//     int i1=id1/100;
//     int j1=id1-i1*100;
//     set<int> ans;
//     for(int i=i1-3 ;i<=i1+3;i++){
//         for(int j=j1-3;j<=j1+3;j++){
//             int tmpID=i*100+j;
//             if(check_can_arrival(istake,id1,tmpID)){
//                 ans.insert(tmpID);
//             }
//         }
//     }
//     return ans;
// }
void setVirPos(Robot& robot){
    int tarID=robot.target_id;
    int istake=robot.get_type==0?0:1;
    robot.cnt_tar=get_best_pos(robot);
    int now_id=robot.close_node;
    if(robot.cnt_tar==-1){
        robot.cnt_tar=robot.close_node;
    }
    int cnt=robot.cnt_tar;
    // if(robot.id==2){
    //     cerr<<"now_id "<<now_id<<"\n";
    //     cerr<<"now_tar "<<cnt<<"\n";
    // }
    bool isSlope=false;
    bool isDanger=dangerous_point[istake][cnt];
    bool con2=false;
    int tmp1=ret_next(robot,cnt);
    if(has_next(robot)){
        
        int tmp2=cnt;
        int i1=tmp1/100,i2=tmp2/100;
        int j1=tmp1%100,j2=tmp2/100;
        if(fabs(i1-i2)+fabs(j1-j2)==2){
            isSlope=true;
        }
    
    }

    int cnt_num=5;
    //进入通道以及从通道出来不采样
    if(dangerous_nums[istake][cnt]>=4&&dangerous_nums[istake][robot.close_node]<4
    || (tmp1!=-1&&dangerous_nums[istake][tmp1]>=4&&dangerous_nums[istake][robot.close_node]<4)
    ||(dangerous_nums[istake][cnt]<4&&dangerous_nums[istake][cnt]>0&&dangerous_nums[istake][robot.close_node]>=4)
    ){
        // if(robot.id==3){
        //           cerr<<"防止了合并"<<endl;
        // cerr<<dangerous_nums[istake][cnt]<<" "<<dangerous_nums[istake][robot.close_node]<<endl;
        // cerr<<tmp1<<" "<<dangerous_nums[istake][tmp1]<<endl;
        // printPair(trans_nodeID_to_pos(tmp1)); 
        // }
 
    
        cnt_num=0;
    }else if(dangerous_nums[istake][cnt]>=4){
        if(robot.get_type!=0)cnt_num=3;
    }
    for(int i=ret_next(robot,cnt);i!=ret_next(robot,i) && i!=-1;i=ret_next(robot,i)){
        int tmpId=i;
 
        // if(i<0)cerr<<"i<0错误"<<endl;
        if(check_can_arrival(istake,now_id,tmpId)&&cnt_num>0){
         
            con2=true;
            robot.cnt_tar=tmpId;
            cnt_num--;
            // if(robot.id==3&&state.FrameID>=200&&state.FrameID>=350 ){
            //     cerr<<endl<<state.FrameID<<" 更新后的tar "<<robot.cnt_tar<<endl;
            //     printPair(exist_id[istake][robot.cnt_tar]);
            // }
        }else{

            break;
        }
        
    }
        if(isSlope&&con2&&isDanger){
            cerr<<"合并了危险的点"<<endl;
        }
        // if(robot.id==0&&state.FrameID>=3092){
        //     cerr<<"time "<<state.FrameID<<endl;
        //     cerr<<robot.cnt_tar<<"->"<<cnt<<endl;
        //     printPair(trans_nodeID_to_pos(robot.cnt_tar));
        //     printPair(trans_nodeID_to_pos(cnt));
        //     cerr<<check_can_arrival(istake,now_id,cnt)<<endl;
        // }
    if(robot.cnt_tar==-1){
        robot.cnt_tar=robot.close_node;
    }
    robot.virtual_id=robot.cnt_tar;
    robot.virtual_pos=exist_id[istake][robot.virtual_id];
    // if(robot.id==2){
       
    //     cerr<<"new_tar "<<tar1<<"\n";
    //     cerr<<"-----------\n";
    // }


}
// pair<double,double>select_visPos(Robot& robot,vector<int> range,int tar3){
//     int now_j= robot.pos.first/0.5;
//     int now_i= robot.pos.second/0.5;
//     int now_id=robot.node_id;
//     int tarID=robot.target_id;
//     int istake=robot.get_type==0?0:1;
//     int tar1=robot.cnt_tar;
//     // if(exist_id[istake].count(tar1)==0){
//     //     cerr<<tar1<<"\n";
//     //     cerr<<"路径选点错误 \n";
//     // }
//     auto virPos=exist_id[istake][tar1];
//     for(auto it: range){
//         if(check_can_arrival(istake,now_id,it)){
//             robot.isVir=true;
//             robot.virtual_id=it;
//             return exist_id[istake][it];;
//         }
//     }
//     return virPos;
// }
int ret_next(Robot& robot,int tar_cnt){
    
    int tarID=robot.target_id;
    if(tarID==-1){
        return -1;
        // cerr<<"tarID==-1错误\n";
    }
    if(robot.target_id==-1)cerr<<"robot.target_id==-1错误\n";
    int istake=robot.get_type==0?0:1;
    return next_node[tarID][istake][tar_cnt];
}
// bool at_least_three(Robot& robot,int tar_cnt){
//     int cnt1=3;
//     int cnt=robot.cnt_tar;
    
//     for(int i=ret_next(robot,cnt);i!=ret_next(robot,i)&&i!=-1;i=ret_next(robot,i)){
//         cnt1--;
//         if(cnt1==0)return true;
//     }
//     return false;
// }
// bool calMinAngle(Robot& robot,pair<double,double>pos){
//     int now_j= robot.pos.first/0.5;
//     int now_i= robot.pos.second/0.5;
//     int tar_j=pos.first/0.5;
//     int tar_i=pos.second/0.5; 
//     auto p1=make_pair<double,double>(0,tar_j-now_j);
//     auto p2=make_pair<double,double>(tar_i-now_i,0);
//     auto p3=make_pair<double,double>(cos(robot.direction),sin(robot.direction));
//     Vec v1(p1);
//     Vec v2(p2);
//     Vec v3(p3);
//     if(lt(cos_t(v3,v1),0)|| lt(cos_t(v2,v1),0))return false;
//     return true;;
// }
double vir_v_1(Robot rob,int v_limit){
    int cnt=gt(return_v(rob),3)?35:25;
   for(int v=v_limit;v>=1;v--){
        if(can_trajectory_virpos(rob,v,cnt)){
            return v;
        }
   }
   return -1;
}
// double vir_v_0(Robot rob){
//    for(int v=6;v>=1;v--){
//         if(can_trajectory_virpos_0(rob,v,10)){
//             return v;
//         }
//    }

//    return -1;
// }
// bool can_trajectory_virpos_0(Robot rob,double v,int cnt){
//     double t=0.02;
//     double v_next=v;
//     for(int i=0;i<cnt;i++){

//         auto pay=calPayload(rob,rob.virtual_pos);
//         if(checkNearBar(rob.pos,pay.radius*0.9)){
//             if(print_cerr_flag_ta&&rob.id==0&&state.FrameID>=200&&state.FrameID<=400&&contr_print_flag){
//                 cerr<<"采样失败速度 "<<v<<" 失败时间 "<<state.FrameID+ i<<" 失败原因 :  撞墙\n";
                
//                 printPair(rob.pos);
//             }
//             return false;
//         };
//         auto tmpPair=get_w_now(rob,pay);
//         double w_next=tmpPair.first;
//         if(rob.cnt_tar<0){
//             // cerr<<state.FrameID<<"\n";
//             // cerr<<"cnt_tar errr1\n";
//             // cerr<<i<<"\n";
//             // cerr<<cnt<<"\n";
//             return true;
//         }
//         if(exist_id[rob.get_type==0?0:1].count(rob.cnt_tar)==0){
//             // cerr<<"exist_id errr\n";
//             return true;
//         }
 
//         double tar_dis=calcuDis(rob.pos,exist_id[rob.get_type==0?0:1][rob.cnt_tar]);
//         int tarID=getPosID(rob.virtual_pos);
//         int now_id=getPosID(rob.pos);
//         if(tarID<0||now_id<0|| tarID>100*99+99|| now_id>100*99+99){
//             cerr<<"id errr\n";
//             return false;
//         }
//         double seta=rob.direction;
//         double w=rob.angular_velocity;
//         double a=return_ac(pay.angular_acceleration,rob.angular_velocity,w_next);
//         double changeAngle=get_at_v_limt(t,pay.angular_acceleration,rob.angular_velocity,w_next,pay.sign)*pay.sign;
//         double v=Calculate_the_projection_speed(rob);
//         double a_v=return_ac(pay.acceleration,v,v_next);
//         rob.pos.first=rob.pos.first+v*cos(seta+changeAngle/2)*t;
//         rob.pos.second=rob.pos.second+v*sin(seta+changeAngle/2)*t;
//         int sign1=ge((rob.angular_velocity+a*t)*w_next,0)?1:-1;
//         int sign2=ge((rob.angular_velocity+a*t),0)?1:-1;
//         double limit_w=0.0;
//         if(lt(a,0)){
//             limit_w=lt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
//         }else{
//             limit_w=gt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
//         }
//         rob.angular_velocity=limit_w;
//         int signv_1=ge((v+a_v*t)*v_next,0)?1:-1;
//         int signv_2=ge((v+a_v*t),0)?1:-1;
//         double limit_v=gt(fabs(v+a_v*t),fabs(6))?6*sign2:v+a_v*t;
//         if(lt(a_v,0)){
//             limit_v=lt(v+a_v*t,v_next)?v_next:v+a_v*t;
//         }else{
//             limit_v=gt(v+a_v*t,v_next)?v_next:v+a_v*t;
//         }
//         v=limit_v;
//         double xy_angle=get_Angle_xy(rob);
//         rob.xy_pos.first=v*cos(xy_angle);
//         rob.xy_pos.second=v*sin(xy_angle);
//         double xy_angle_next=get_Angle_xy(rob);
//         double cal_angle=xy_angle_next-xy_angle;
//         vector<vector<double>>mat(4,vector<double>(4,0));
//         cal_matrix(mat,changeAngle,cal_angle);
//         rob.direction+=changeAngle;
//         rob.direction=rob.direction>Pi?rob.direction-2*Pi:rob.direction; 
//         double t1=rob.xy_pos.first,t2=rob.xy_pos.second;
//         rob.xy_pos.first=(t1*mat[0][0]+t2*mat[0][1]);
//         rob.xy_pos.second=(t1*mat[1][0]+t2*mat[1][1]);
//     }  
//     int tarID=getPosID(rob.virtual_pos);
//     int now_id=getPosID(rob.pos);
//     int istake=rob.get_type==0?0:1;
//     return true;
       
  
// }
bool can_trajectory_virpos(Robot rob,double v,int cnt){

    double t=0.02;
    double v_next=v;
    for(int i=0;i<cnt;i++){

        auto pay=calPayload(rob,rob.virtual_pos);
        if(checkNearBar(rob.pos,pay.radius)){
            if(print_cerr_flag_ta&&rob.id==0&&state.FrameID>=200&&state.FrameID<=400&&contr_print_flag){
                cerr<<"采样失败速度 "<<v<<" 失败时间 "<<state.FrameID+ i<<" 失败原因 :  撞墙\n";
            }
            return false;
        };
        auto tmpPair=get_vir_w(rob,pay);
        double w_next=tmpPair.first;
        if(rob.cnt_tar<0){
            cerr<<"cnt_tar errr\n";
            return false;
        }
        if(exist_id[rob.get_type==0?0:1].count(rob.cnt_tar)==0){
            cerr<<"exist_id errr\n";
            return false;
        }
 
        double tar_dis=calcuDis(rob.pos,exist_id[rob.get_type==0?0:1][rob.cnt_tar]);
        int tarID=getPosID(rob.virtual_pos);
        int now_id=getPosID(rob.pos);
        if(tarID<0||now_id<0|| tarID>100*99+99|| now_id>100*99+99){
            cerr<<"id errr\n";
            return false;
        }
    
        int istake=rob.get_type==0?0:1;
        int posID_tmp=getPosID(rob.pos);
        if(illegal_point[istake][posID_tmp])return false;
        if(check_can_arrival(istake,now_id,tarID)
        &&(check_tar_line(rob,0.3))){
            if(print_cerr_flag_ta&&rob.id==0&&state.FrameID>=200&&state.FrameID<=400&&contr_print_flag){
            cerr<<"采样速度 "<<v<<" 时间 "<<state.FrameID+ i<<" 原因 :  到达目标\n";
            }
            return true;
        }
   
        if(tmpPair.second){
            if(check_can_arrival(istake,now_id,tarID)){
                // if(print_cerr_flag_ta&&rob.id==0){
                //     cerr<<now_id<<" "<<tarID<<"\n";
                //     cerr<<"检查到的对齐点: \n";
                //     printPair(rob.pos);
                //     printPair(rob.virtual_pos);
                //     cerr<<state.FrameID+ i<<"\n";
                // }
   
                return true;
            }
            else{
                 if(print_cerr_flag_ta&&rob.id==0&&state.FrameID>=200&&state.FrameID<=400&&contr_print_flag){
                    cerr<<"采样失败速度 "<<v<<" 失败时间 "<<state.FrameID+ i<<" 失败原因 :  对起点不能到达目标\n";
                 }

                return false;
            }
                
        }
         
        double seta=rob.direction;
        double w=rob.angular_velocity;
        double a=return_ac(pay.angular_acceleration,rob.angular_velocity,w_next);
        double changeAngle=get_at_v_limt(t,pay.angular_acceleration,rob.angular_velocity,w_next,pay.sign)*pay.sign;
        double v=Calculate_the_projection_speed(rob);
        double a_v=return_ac(pay.acceleration,v,v_next);
        rob.pos.first=rob.pos.first+v*cos(seta+changeAngle/2)*t;
        rob.pos.second=rob.pos.second+v*sin(seta+changeAngle/2)*t;
        int sign1=ge((rob.angular_velocity+a*t)*w_next,0)?1:-1;
        int sign2=ge((rob.angular_velocity+a*t),0)?1:-1;
        double limit_w=0.0;
        if(lt(a,0)){
            limit_w=lt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
        }else{
            limit_w=gt(rob.angular_velocity+a*t,w_next)?w_next:rob.angular_velocity+a*t;
        }
        rob.angular_velocity=limit_w;
        int signv_1=ge((v+a_v*t)*v_next,0)?1:-1;
        int signv_2=ge((v+a_v*t),0)?1:-1;
        double limit_v=gt(fabs(v+a_v*t),fabs(6))?6*sign2:v+a_v*t;
        if(lt(a_v,0)){
            limit_v=lt(v+a_v*t,v_next)?v_next:v+a_v*t;
        }else{
            limit_v=gt(v+a_v*t,v_next)?v_next:v+a_v*t;
        }
        v=limit_v;
        double xy_angle=get_Angle_xy(rob);
        rob.xy_pos.first=v*cos(xy_angle);
        rob.xy_pos.second=v*sin(xy_angle);
        double xy_angle_next=get_Angle_xy(rob);
        double cal_angle=xy_angle_next-xy_angle;
        vector<vector<double>>mat(4,vector<double>(4,0));
        cal_matrix(mat,changeAngle,cal_angle);
        rob.direction+=changeAngle;
        rob.direction=rob.direction>Pi?rob.direction-2*Pi:rob.direction; 
        double t1=rob.xy_pos.first,t2=rob.xy_pos.second;
        rob.xy_pos.first=(t1*mat[0][0]+t2*mat[0][1]);
        rob.xy_pos.second=(t1*mat[1][0]+t2*mat[1][1]);
    }  
    int tarID=getPosID(rob.virtual_pos);
    int now_id=getPosID(rob.pos);
    int istake=rob.get_type==0?0:1;
    if(check_can_arrival(istake,now_id,tarID)){
        // if(rob.id==0){
        // cerr<<"未检查到对齐\n";
        // }
        if(print_cerr_flag_ta&&rob.id==0&&state.FrameID>=200&&state.FrameID<=400&&contr_print_flag){
                cerr<<"采样失败速度 "<<v<<" 失败时间 "<<state.FrameID+ cnt<<" 失败原因 :  对起点不能到达目标\n";
        }
        return true;
    }    
    else{
        return false;
    }
        
}
int getPosID(pair<double,double>pos){
    return int(pos.second/0.5)*100+int(pos.first/0.5);
}
pair<double,bool> get_vir_w(Robot& robot,PayLoad& payload){
    if(has_next(robot)){
        int istake=robot.get_type==0?0:1;
        int tar=robot.cnt_tar;
        int tar1=next_node[robot.target_id][istake][robot.cnt_tar];
        auto p1=subVector(exist_id[istake][tar1],exist_id[istake][tar]);
        Vec v1(p1);
        Vec v2(robot.xy_pos);
        double angle=acos(cos_t(v1,v2));
        double rateAngle_fabs=0;
        if(!robot.need_adjust_statues){
            if(gt(angle,0.3)){
                rateAngle_fabs=Pi;
            }else if(gt(angle,0.15)){
                rateAngle_fabs=Pi/2;
            }else if(gt(angle,0.075)){
                rateAngle_fabs=Pi/4;
            }else{
                rateAngle_fabs=Pi/8;
            }
        }else{
            if(gt(angle,0.3)){
                rateAngle_fabs=Pi;
            }else if(gt(angle,0.2)){
                rateAngle_fabs=Pi/3;
            }else if(gt(angle,0.1)){
                rateAngle_fabs=Pi/4;
            }else{
                rateAngle_fabs=Pi/8;
            }        
        }
        bool can_st=lt(fabs(angle),0.1)?true:false;
        double tmpAngle =rateAngle_fabs*payload.sign;
        return {tmpAngle,can_st} ;
    }else{
        return get_w_now(robot,payload);
    }
}

void init_rob_status(Robot& rob){
    int istake=rob.get_type==0?0:1;
    rob.cnt_tar=rob.close_node;
    rob.virtual_pos=exist_id[istake][rob.close_node];
    rob.virtual_id=rob.close_node;

}
bool has_next(Robot& rob){
    int tarID=rob.target_id;
    if(tarID==-1){
        return false;
        // cerr<<"tarID==-1错误\n";
    }
    return next_node[rob.target_id][rob.get_type==0?0:1][rob.cnt_tar]!=-1?true:false;
}
bool check_tar_line(Robot& robot,double dis){
    if(!has_next(robot))return false;
    int istake=robot.get_type==0?0:1;
    int tar=robot.cnt_tar;
    int tar1=next_node[robot.target_id][istake][robot.cnt_tar];
    auto p1=subVector(exist_id[istake][tar1],exist_id[istake][tar]);   
    Line_Ta line( exist_id[istake][tar],p1);
    Point_Ta point(robot.pos);
    double tmpdis= dis_lp(point,line);
    return lt(tmpdis,dis);
}
// unordered_map<int,bool> illegal_point[2];//判断是否是非法点
// unordered_map<int,bool> dangerous_point[2];//判断是否是危险点
void get_point_type(){
    for(int i=0;i<100;i++){
        for(int j=0;j<100;j++){
            int id=i*100+j;
            if(exist_id[0].count(id)==0&&stu_transID.count(id)==0){
                illegal_point[0][id]=true;
            }
            if(exist_id[1].count(id)==0&&stu_transID.count(id)==0){
                illegal_point[1][id]=true;
            }
            // for(int i1=i-1;i1<=i+1;i1++){
            //     for(int j1=j-1;j1<=j+1;j1++){
            //         int tmpID=i1*100+j;
            //         if(illegal_point[0][id]&&exist_id[0].count(tmpID)&&!illegal_point[0][tmpID]){
            //             exist_id[0][id]=exist_id[0][tmpID];
                
            //         }
            //         if(illegal_point[1][id]&&exist_id[1].count(tmpID)&&!illegal_point[1][tmpID]){
            //             exist_id[1][id]=exist_id[1][tmpID];
            //         }
            //     }
            // }
        }
    }

    for(int i=0;i<100;i++){
        for(int j=0;j<100;j++){
            int id=i*100+j;
         
            if(i-1>=0&&i+1<=99){
                if(illegal_point[0][id-100]&&illegal_point[0][id+100]){
                    dangerous_nums[0][id]+=1<<2;
                }
                if(illegal_point[1][id-100]&&illegal_point[1][id+100]){
                    dangerous_nums[1][id]+=1<<2;
                }
                dangerous_nums[0][id]+=(illegal_point[0][id-100]?1:0)+(illegal_point[0][id+100]?1:0);
                dangerous_nums[1][id]+=(illegal_point[1][id-100]?1:0)+(illegal_point[1][id+100]?1:0);
            }
            if(j-1>=0&&j+1<=99){
                if(illegal_point[0][id-1]&&illegal_point[0][id+1]){
                    dangerous_nums[0][id]+=1<<2;
                }
                if(illegal_point[1][id-1]&&illegal_point[1][id+1]){
                    dangerous_nums[1][id]+=1<<2;
                }
                dangerous_nums[0][id]+=(illegal_point[0][id-1]?1:0)+(illegal_point[0][id+1]?1:0);
                dangerous_nums[1][id]+=(illegal_point[1][id-1]?1:0)+(illegal_point[1][id+1]?1:0);
            }
            // if(id==6379)cerr<<dangerous_nums[0][id]<<endl;
            int con1=false,con2=false;
            for(int i1=i-1;i1<=i+1;i1++){
                bool need_ad1=true;
                bool need_ad2=true;
                for(int j1=j-1;j1<=j+1;j1++){
                    if(i1<0||j1<0||i1>99 ||j1>99|| (i==i1&&j==j1) )continue;
                    int tmpID=i1*100+j1;
                
                    if(need_ad1&&illegal_point[0][id]&&exist_id[0].count(tmpID)
                    ){
                        con1=true;
                        exist_id[0][id]=exist_id[0][tmpID];
                        if(exist_id[1].count(tmpID)){
                            exist_id[0][id]=exist_id[1][tmpID];
                            need_ad1=false;
                        }
                
                    }
                    if(need_ad2&&illegal_point[1][id]&&exist_id[1].count(tmpID)){
                        exist_id[1][id]=exist_id[1][tmpID];
                    }
                }
            } 
            if(con1==false&&graph[i][j]!=-2&&illegal_point[0][id]){
                cerr<<"未知错误"<<" "<<i*100+j<<"\n";
                cerr<<exist_id[0].count(101)<<"\n";
                cerr<<illegal_point[0][1]<<"\n";
            }           
        }
    }
    
    for(int i=0;i<100;i++){
        for(int j=0;j<100;j++){
            int id=i*100+j;
            for(int i1=i-1;i1<=i+1;i1++){
                for(int j1=j-1;j1<=j+1;j1++){
                    int tmpID=i1*100+j;
                     if(i1<0||j1>99||j1<0||i1>99)continue;
                    if(!illegal_point[0][id]&&exist_id[0].count(id)&&exist_id[0].count(tmpID)==0){
                        dangerous_point[0][id]=true;
                    }
                    if(!illegal_point[1][id]&&exist_id[1].count(id)&&exist_id[1].count(tmpID)==0){
                        dangerous_point[1][id]=true;
                    }
                }
            }
        }
    }
   

  
    //  cerr<<"pos 4321";
    //  printPair(exist_id[0][4321]);
    //  cerr<<illegal_point[0].count(4321)<<endl;

}
void check_robot_pos_status(Robot& robot){
    int id=robot.close_node;
    int istake=robot.get_type==0?0:1;
    if(illegal_point[istake][id]){
        robot.is_illegal=true;
    }else if(dangerous_point[istake][id]){
        robot.is_dangerous=true;
    }
}
// void adjust_illegal_pos(Robot& robot){
//     // robot.cnt_tar =0;
//     //在机器人四周寻找一个与目标的直达pos;
//     // int id=robot.node_id;
//     // int istake=robot.get_type==0?0:1;
//     // for(int i=-1;i<=1;i++){
//     //     for(int j=-1;j<=1;j++){
//     //         int tmpID=id+100*i+j;
     
//     //         if(exist_id[istake].count(tmpID) &&check_can_arrival(istake,tmpID,robot.cnt_tar)){
//     //             robot.virtual_id=tmpID;
//     //             robot.virtual_pos=exist_id[istake][tmpID];
//     //             robot.cnt_tar=tmpID;
//     //             robot.is_new_tar_ing=true;
//     //             return;
//     //         }
//     //     }
//     // }
//     // return;
// }
// void  select_the_standard_id(Robot& robot){
//     int id=robot.node_id;
//     int i=id/100,j=id%100;
//     double minDis=100;
//     int istake=robot.get_type==0?0:1;
//     if(istake==1)return ;
//     int chose_id=id;
//     for(int i1=i-1;i1<=i+1;i1++){
//         for(int j1=j-1;j1<=j+1;j1++){
//             int tmpId=i1*100+j1;
//             double cmpDis=calcuDis(robot.pos,exist_id[istake][tmpId]);
//             if(lt(cmpDis,minDis)){
//                 minDis=cmpDis;
//                 chose_id=tmpId;
//             }
//         }
//     }
//     if(state.FrameID==100&&robot.id==0){
//         cerr<<robot.node_id<<" --- "<<chose_id<<endl;
//     }
//     robot.node_id=chose_id;
// }
bool empty_pos(const Robot& robot){
    int id=robot.get_type==0?robot.close_node:robot.node_id;
    int i=id/100,j=id%100;
    double minDis=100;
    int istake=robot.get_type==0?0:1;
    int chose_id=id;
    for(int i1=i-1;i1<=i+1;i1++){
        for(int j1=j-1;j1<=j+1;j1++){
            int tmpId=i1*100+j1;
            if(illegal_point[istake][tmpId]||dangerous_point[istake][tmpId]){
                return false;
            }
        }
    }
    return true;
}
bool  need_to_step_back(const Robot& rob){
    int istake=rob.get_type==0?0:1;
    auto p1=subVector(rob.virtual_pos,rob.pos);
    auto p2=subVector(exist_id[istake][rob.close_node],rob.pos);
    Vec v1(p1);
    Vec v2(p2);
    Vec v3(rob.xy_pos);
    if(lt(cos_t(v3,v2),0)&& gt(cos_t(v3,v1),0))return true;
    return false;
}
int get_best_pos(const Robot& robot){
    int id=robot.cnt_tar;
    int istake=robot.get_type==0?0:1;

    if(!illegal_point[istake][id] &&!dangerous_point[istake][id]){
        return id;
    }
    int i=id/100,j=id%100;
    double minDis=100;
    int now_id=robot.close_node;
    int chose_id=id;
    int tar=robot.target_id;
    double cmpDis= dis_to_studios[tar][istake][now_id]+1.5;
    int choseId=id;
    for(int i1=i-1;i1<=i+1;i1++){
        for(int j1=j-1;j1<=j+1;j1++){
            if(i1<0||j1<0||i1>99||j1>99)continue;
            int tmpID=i1*100;
            if(!illegal_point[istake][id] &&!dangerous_point[istake][id]&&
            check_can_arrival(istake,id,tmpID)&& lt(dis_to_studios[tar][istake][tmpID],cmpDis)
            &&check_can_arrival(istake,now_id,tmpID)
            ){
                choseId= tmpID;
                cmpDis=dis_to_studios[tar][istake][tmpID];
            }
        }
    }
    if(chose_id!=robot.cnt_tar&&robot.id==1){
        cerr<<"time "<<state.FrameID<<endl;
        cerr<<"机器人："<<robot.id<<"更新了最优点：";
        printPair(trans_nodeID_to_pos(robot.close_node));
        cerr<<"机器人："<<robot.id<<"原来的目标：";
        printPair(trans_nodeID_to_pos(robot.cnt_tar));
    }
    return choseId;
}
bool check_slope_studios(int id1,int id2){
    int i1=min(id1/100,id2/100),i2=max(id1/100,id2/100);
    int j1=min(id1%100,id2%100),j2=max(id1%100,id2%100);
    for(int i=i1;i<=i2;i++){
        for(int j=j1;j<=j2;j++){
            if(graph_trans[i][j]==-2)return false;
        }
    }
    return true;
}