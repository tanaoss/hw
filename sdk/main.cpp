#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include "class.h"
using namespace std;
extern vector<Studio> studios;
extern vector<Robot> robots;
extern State state;//当前帧数，全局可见
extern vector<PayLoad> pl_g;;
extern vector<PayLoad> payloads;
extern int class_map;
int main()
{
    std::ios::sync_with_stdio(false);   
    std::cin.tie(0);    // IO
    // cerr<<"sss"<<endl;
    readMapUntilOK();
    // cerr << "sss" << endl;
    // divide_space();
    // deal_graph();
    // cerr << "sss" << endl;
    
    init_data();
    init_vector();
    // printEdge(0);
    // printMap(1);
    for(int i = 0; i < studios.size(); ++i) {
        Dijkstra(i, 0);
        Dijkstra(i, 1);
        // if(i==1) print_dijkstra();
    }
    // print_dijkstra(1, 0);
    // for(int i = 0; i < 4; ++i) {
    //     Dijkstra(i, 0, 1);
    // }
    // cerr<<studios[12].node_id<<endl;
    // printPath(1, 0, 12, 1);
    // floyd();
    // print_queue();
    // cerr<<robots.size();
    // printEdge(1);
    cout<<"OK\n";
    cout.flush();
    int count = 0;
    initrobotInfo();
    init_studio_parameter();
    
     while (cin >> state.FrameID)
    {
        // cerr<<" time "<<state.FrameID<<endl;
        readStatusUntilOK() ;
        
        cout<<state.FrameID<<endl;
        if(count == 0)first_action();
        else robot_action();
        adjust_virtual_pos_total();
        payloads.clear();
        for(int i=0;i<4;++i){
            payloads.push_back(calPayload(robots[i], robots[i].virtual_pos));
        }
        
        for(int i=0;i<4;++i) robots[i].radius = payloads[i].radius;
        pl_g=payloads;
        cerr<<"222"<<endl;
        control(payloads);
        count++;
        cerr<<"333"<<endl;

        if(state.FrameID == 3053) {
            for(int i = 0; i < 4; ++i) {
                cerr<<i<<"-target:"<<robots[i].target_id<<endl;
            }
        }
    }
    return 0;
   
}
