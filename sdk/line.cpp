#include "line.h"

bool checkLineCross(LLine l1, LLine l2) {
    if(sgn(l1.s.y-l1.e.y)==0 || sgn(l2.s.y-l2.e.y) == 0)
        return false;
        if(sgn(l1.s.y-l1.e.y) < 0)swap(l1.s,l1.e);
        if(sgn(l2.s.y-l2.e.y) < 0)swap(l2.s,l2.e);
        if(inter(l1,l2) == false)
        return false;
        //口被封掉的情况
        if(inter(LLine(l1.s,Point(l1.s.x,100000)),l2) )
        return false;
        //口被封掉
        if(inter(LLine(l2.s,Point(l2.s.x,100000)),l1) )
        return false;
}