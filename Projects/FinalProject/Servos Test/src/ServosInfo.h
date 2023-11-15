#define TIME_STEP 5
#define TIME_MAX ((2000 - 1000) /TIME_STEP + 2)

struct ServoInfo
{   
    Servo servo;
    float angle[TIME_MAX];
    int time[TIME_MAX];
};

ServoInfo servo1;



