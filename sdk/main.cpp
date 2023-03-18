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

int main()
{
    std::ios::sync_with_stdio(false);   
    std::cin.tie(0);    // IO
    readMapUntilOK();
    calcuStudioDis();
    cout<<"OK\n";
    cout.flush();
    int count = 0;
    initRobortInfo();
    init_studio_parameter();
    while (cin>>state.FrameID){
        //cerr<<" time "<<state.FrameID<<endl;
        readStatusUntilOK() ;
        cout<<state.FrameID<<endl;
        if(count == 0)first_action();
        else robot_action();
        payloads.clear();
        for(int i=0;i<4;++i){
            payloads.push_back(calPayload(i,robots[i].target_id));
        }
        pl_g=payloads;
        control(payloads);
        // int lineSpeed = 3;
        // double angleSpeed = 1.5;
        // for(int robotId = 0; robotId < 4; robotId++){
        //     cout<<"forward "<<robotId<<" "<<lineSpeed<<"\n"
        //     <<"rotate "<<robotId<<" "<<angleSpeed<<"\n";
        // }
        // cout<<"OK\n";
        // cout.flush();
        count++;
    }
    return 0;
   
}
