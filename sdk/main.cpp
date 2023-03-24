#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include "class.h"
using namespace std;
extern vector<vector<double>> dis;
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
    readMapUntilOK();
    if(eq(studios[0].pos.first,1.25)&&eq(studios[0].pos.second,48.75)) class_map = 1;
    if(eq(studios[0].pos.first,4.75)&&eq(studios[0].pos.second,48.75)) class_map = 2;
    if(eq(studios[0].pos.first,25.25)&&eq(studios[0].pos.second,47.75)) class_map = 3;
    if(eq(studios[0].pos.first,24.75)&&eq(studios[0].pos.second,49.25)) class_map = 4;
    for(int i = 0;i<studios.size();i++)studios[i].wait_time = 0;
    // cerr<<studios[0].pos.first<<' '<<studios[0].pos.second<<endl;
    // cerr<<class_map<<endl;
    calcuStudioDis();
    // cerr<<robots.size();
    cout<<"OK\n";
    cout.flush();
    int count = 0;
    initrobotInfo();
    init_studio_parameter();
    while (cin>>state.FrameID){
        //cerr<<" time "<<state.FrameID<<endl;
        // cerr<<" time "<<state.FrameID<<endl;
        readStatusUntilOK() ;
        cout<<state.FrameID<<endl;
        //cerr<<"aaa"<<endl;
        if(count == 0)first_action();
        else robot_action();
        //cerr<<"bbb"<<endl;
        payloads.clear();
        for(int i=0;i<4;++i){
            payloads.push_back(calPayload(i,robots[i].target_id));
        }
        //cerr<<"kkk"<<endl;
        pl_g=payloads;
        control(payloads);
        count++;
    }
    return 0;
   
}
