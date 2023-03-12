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


int main()
{
    std::ios::sync_with_stdio(false);   
    std::cin.tie(0);    // IO
    readMapUntilOK();
    calcuStudioDis();
    cout<<"OK\n";
    cout.flush();
    int count = 0;
    vector<PayLoad> payloads;
    while (cin>>state.FrameID){
        readStatusUntilOK() ;
        cout<<state.FrameID<<endl;
        if(count == 0)first_pick_point();
        else robot_action();
        payloads.clear();
        for(int i=0;i<4;++i){
            payloads.push_back(calPayload(i));
        }
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
