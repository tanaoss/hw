#include <iostream>
#include <vector>
#include<string>
#include <cmath>
#include "class.h"
using namespace std;
extern vector<vector<double>> dis;
extern vector<Studio> studios;
extern vector<Robot> robots;
extern State state;//当前帧数，全局可见

bool readUntilOK() {
    char line[1024];
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
    }
    return false;
}
int main()
{
    std::ios::sync_with_stdio(false);   
    std::cin.tie(0);    // IO
    readMapUntilOK();
    cout<<"OK\n";
    cout.flush();
    while (cin>>state.FrameID){
        readStatusUntilOK() ;
        cout<<state.FrameID<<endl;
        int lineSpeed = 3;
        double angleSpeed = 1.5;
        for(int robotId = 0; robotId < 4; robotId++){
            cout<<"forward "<<robotId<<" "<<lineSpeed<<"\n"
            <<"rotate "<<robotId<<" "<<angleSpeed<<"\n";
        }
        cout<<"OK\n";
        cout.flush();
    }
    return 0;
   
}
