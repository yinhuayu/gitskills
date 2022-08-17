#define ABS(x) (((x) > 0) ? (x) : -(x))
#define MAX(a, b) ( (a) >= (b) ? (a) : (b) )
#define MIN(a, b) ( (a) <= (b) ? (a) : (b) )
#define MAX_OF_3(a, b, c) MAX(a , MAX(b, c)) 

#define MAP(v,min1,max1,min2,max2) ((v-min1) * (max2-min2) / (max1-min1) + min2)
#define ARRAY_SZ(x) (sizeof(x)/sizeof(x[0]))