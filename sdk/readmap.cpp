#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include "class.h"
using namespace std;
int graph[100][100];
double dis[10000][10000];
int target_sequence[10000][10000];
int wail[101][101];
double EPS=1e-7;
vector<pane> panes;

bool eq(double a, double b) { return abs(a - b) < EPS; } // ==
bool gt(double a, double b) { return a - b > EPS; }      // >
bool lt(double a, double b) { return a - b < -EPS; }     // <
bool ge(double a, double b) { return a - b > -EPS; }     // >=
bool le(double a, double b) { return a - b < EPS; }      // <=
double calcuDis(pair<double, double> a, pair<double, double> b)
{
    return sqrt((a.first - b.first) * (a.first - b.first) + (a.second - b.second) * (a.second - b.second));
}

int main()
{
    std::ios::sync_with_stdio(false);   
    std::cin.tie(0);    // IO
    readMapUntilOK();
    floyd();
    print_queue();
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
    ifstream inFile("../../maps/2.txt", ios::in | ios::binary);
    if (!inFile) {
        cout << "error" << endl;
        return 0;
    }
    while (inFile.getline(line,sizeof(line))) {
        for(i=0;i<100;i++){
            pane train;
            train.id = num;
            train.pos.first = i * 0.5 + 0.25;
            train.pos.second = (100 - count) * 0.5 - 0.25;
            train.type = -1;
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
bool check_fall_into_scope(double k1,double b1,double b2,double k2,double b3,double b4,pair<double,double> pos){
    if(gt((k1*pos.first+b1),pos.second)){
        if (lt((k1 * pos.first + b2), pos.second)){
            if (gt((k2 * pos.first + b3), pos.second)){
                if (gt((k2 * pos.first + b4), pos.second)){
                    return true;
                }
            }
        }
    }
    return false;
}
bool check_barrier(int start,int end,int carry){
    double r, k, b, offset,k1,b1,b2,k2,b3,b4,a,b5,c,d;
    if(carry==1)r = 0.45;
    else r = 0.53;
    if (!eq((panes[end].pos.first - panes[start].pos.first),0 )){
        k =(double) (panes[end].pos.second - panes[start].pos.second) / (panes[end].pos.first - panes[start].pos.first);
    }
    else if (gt((panes[end].pos.second - panes[start].pos.second),0)){
        k = 100;
    }
    else if (lt((panes[end].pos.second - panes[start].pos.second), 0)){
        k = -100;
    }
    else k = 0;
    b = (panes[start].pos.second - k * panes[start].pos.first);
    k1 = k;
    offset =(double)r/(cos(Pi-atan(k)));
    if(gt(offset,0)){
        b1 = b + offset;
        b2 = b - offset;
    }
    else{
        b1 = b - offset;
        b2 = b + offset;
    }
    k2 = (double)(-1/k1);
    if (gt(panes[end].pos.second, panes[start].pos.second)){
        b1 = panes[end].pos.second - k2 * (panes[end].pos.first);
        b2 = panes[start].pos.second - k2 * (panes[start].pos.first);
        c = panes[end].pos.second+r;
        d = panes[start].pos.second-r;
    }
    else{
        b2 = panes[end].pos.second - k2 * (panes[end].pos.first);
        b1 = panes[start].pos.second - k2 * (panes[start].pos.first);
        d = panes[end].pos.second-r;
        c = panes[start].pos.second+r;
    }
    if(gt(panes[end].pos.first, panes[start].pos.first)){
        a = panes[end].pos.first+r;
        b5 = panes[start].pos.first-r;
    }
    else{
        b5 = panes[end].pos.first-r;
        a = panes[start].pos.first+r;
    }
    cerr<<b5<<' '<<a<<' '<<c<<' '<<d<<endl;
    for(int i = (int)b5; i<(int)(a); i++){
        for(int j = (int)d; j<(int)(c); j++){
            if(wail[i][j]==-2){
                if (check_fall_into_scope(k1,b1,b2,k2,b3,b4,pair<double,double>(i*0.5,j*0.5))){
                    // if(wail[i][j]==-2){
                    return false;
                    // }
                }
            }
        }
    }
    return true;

}
void init_dis(){
    int i,j;
    for(i=0;i<10000;i++){
        for(j=i;j<10000;j++){
            if(panes[i].type == -2 ||panes[j].type == -2){
                dis[i][j] = 1000;
                dis[j][i] = 1000;
                target_sequence[i][j] = -1;
                target_sequence[j][i] = -1;
                continue;
            }
            if (i == j)
                dis[j][i] = 0;
            else{
                if (check_barrier(i,j,0)){
            //         // cerr<<"cc"<<endl;
                    dis[i][j] = calcuDis(panes[i].pos, panes[j].pos);
                    dis[j][i] = dis[i][j];
                    // if(j==9999){
                    //     cerr<<i<<' '<<j<<' '<<dis[i][j]<<endl;
                    //     dis[j][i] = dis[i][j];
                    // }
                    target_sequence[i][j] = j;

                    target_sequence[j][i] = i;
                }
                else{
                    dis[i][j] = 1000;
                    dis[j][i] = dis[i][j];
                    target_sequence[i][j] = -1;
                    target_sequence[j][i] = -1;
                }
            //     // cerr<<"bb"<<endl;
            }
            // if(i>=3728)
            // cerr<<"j = "<<j<<endl;
        }
        cerr<<i<<endl;
    }
}

void floyd(){
    init_dis();
    cerr<<"AA"<<endl;
    for(int i = 0;i<10000;i++){
        for(int j = 0;j<10000;j++){
            for(int k =j+1 ;k<10000;k++){
                if (gt(dis[j][k],dis[j][i]+dis[i][k])){
                    dis[j][k] = dis[j][i]+dis[i][k];
                    dis[k][j] = dis[j][k];
                    target_sequence[j][k] = target_sequence[j][i];
                    target_sequence[k][j] = target_sequence[k][i];
                }
            }
        }
    }

}
void print_queue(){
    for(int i=0;i<10000;i++){
        for(int j = 0; j<10000;j++){
            cerr<<i<<' '<<j<<' '<<dis[i][j]<<endl;
            cerr<<target_sequence[i][j]<<endl;
        }
        cerr<<endl;
    }
}
