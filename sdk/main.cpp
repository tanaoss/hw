#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include "class.h"
using namespace std;
extern vector<Studio> studios;
extern vector<Robot> robots;
extern State state;//当前帧数，全局可见
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
    // cerr << "bbb" << endl;
    // cerr<<studios[10].node_id<<endl;
    // print_dijkstra(0, 0, 1);
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
    // cerr<<"kkk"<<endl;
    init_studio_parameter();
    // cerr<<"eee"<<endl;
    // print_dijkstra(0,1,1);
     while (cin >> state.FrameID)
    {
        cerr<<" time "<<state.FrameID<<endl;
        readStatusUntilOK() ;
        
        cout<<state.FrameID<<endl;
        if(count == 0)new_first_action();
        else new_robot_action();

        // if(state.FrameID == 4064) {
        //     cerr<<robots[1].target_id<<endl;
        // }
        control();

        count++;

        // if(state.FrameID >= 4143 && state.FrameID < 4302) {
        //     cerr<<" time "<<state.FrameID<<endl;
        //     for(int i = 0; i < 1; ++i) {
        //         cerr<<i<<"-target:"<<robots[i].target_id<<" type:"<<studios[robots[i].target_id].type<<endl;
        //     }
        // }
    }
    return 0;
   
}

// ./Robot -f -m maps/4.txt -c ./sdk "./main" 2>1.txt
