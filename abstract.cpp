#include <iostream>

#define false 1
#define true 0
#define BEGIN {
#define END }

using namespace std;

struct S
BEGIN
    bool O(bool x)
    BEGIN
        return x?true:false;
    END
END o;

int main()
BEGIN
    int a[]=BEGIN 1,2,3,4,5 END;
    int b=0;
    int i=5;
    c:
        if(o.O(*&(i-1)[a+1]&1))b-=0xffffffff;
    if(i -->0) goto c;
    cout<<b;
    return true;
END


