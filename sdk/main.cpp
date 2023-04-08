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
extern int cerr_flag_j;
extern int start_time;
extern int end_time;




void mock_fram_skip_after(){
    int money;
    string line;
    cin >> money;
    cin.ignore();
    int K;
    cin >> K;
    cin.ignore();
    while (K--)
    {
        vector<double> tmp(6, 0);
        for (int i = 0; i < tmp.size(); i++)
        {
            cin >> tmp[i];
        }
    }
    for (int i = 0; i < 4; i++)
    {
        vector<double> tmp(10, 0);
        for (int i = 0; i < tmp.size(); i++)
        {
            cin >> tmp[i];
        }
    }
    cin >> line;
    // if (line[0] == 'O' && line[1] == 'K')
    // {
    //     cout<<state.FrameID<<endl;
    //     out_put();
    // }
}

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
    
    // cerr<<studios[10].node_id<<endl;
    // print_dijkstra(7, 1, 1);
    // cerr<<"\n";
    // print_dijkstra(7, 1, 0);
    
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
        // if(state.FrameID>start_time && state.FrameID<end_time &&cerr_flag_j){
            // cerr<<" time "<<state.FrameID<<endl;
        // }
        // cerr<<" time "<<state.FrameID<<endl;
        // if(state.FrameID == 95 
        //     || state.FrameID == 97
        //     || state.FrameID == 99
        //     || state.FrameID == 2725
        //     || state.FrameID == 2759
        //     || state.FrameID == 2851
        //     || state.FrameID == 8422) {
        //     mock_fram_skip();
        //     continue;
        // }

        // if(state.FrameID == 96 
        //     || state.FrameID == 98
        //     || state.FrameID == 100
        //     || state.FrameID == 2726
        //     || state.FrameID == 2760
        //     || state.FrameID == 2852
        //     || state.FrameID == 8423) {
        //         mock_fram_skip_after();
        // }

        // else readStatusUntilOK() ;

        readStatusUntilOK() ;
        cout<<state.FrameID<<endl;
        if(count == 0)new_first_action();
        else new_robot_action();


        

        control();

        count++;

        // if(state.FrameID >= 13200 && state.FrameID < 13300) {
        //     cerr<<" time "<<state.FrameID<<endl;
        //     for(int i = 0; i < 1; ++i) {
        //         cerr<<i<<"-target:"<<robots[i].target_id<<" type:"<<studios[robots[i].target_id].type<<endl;
        //     }
        // }
    }
    return 0;
   
}



// ./Robot -f -m maps/4.txt -c ./sdk "./main" 2>1.txt
