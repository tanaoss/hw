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
    // cerr << "sss" << endl;
    
    init_data();
    printEdge(0);
    for(int i = 0; i < studios.size(); ++i) {
        Dijkstra(studios[i].node_id, 0, 0);
        Dijkstra(studios[i].node_id, 1, 0);
    }
    for(int i = 0; i < 4; ++i) {
        Dijkstra(robots[i].node_id, 0, 1);
    }
    // floyd();
    // print_queue();
    // cerr<<robots.size();
    cout<<"OK\n";
    cout.flush();
    int count = 0;
    initrobotInfo();
    init_studio_parameter();
     while (cin >> state.FrameID)
    {
        cerr<<" time "<<state.FrameID<<endl;
        readStatusUntilOK() ;
        cout<<state.FrameID<<endl;
        if(count == 0)first_action();
        else robot_action();
        payloads.clear();
        for(int i=0;i<4;++i){
            payloads.push_back(calPayload(i));
        }
        for(int i=0;i<4;++i) robots[i].radius = payloads[i].radius;
        pl_g=payloads;
        control(payloads);
        count++;
    }
    return 0;
   
}
