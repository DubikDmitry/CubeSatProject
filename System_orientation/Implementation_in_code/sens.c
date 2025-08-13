#include "stdio.h"
//#include "mml1.c"
#include "vv2q.c"



Vec3 get_vec_sun(){
	Vec3 s1;
	s1.x = 1;
	s1.y = 1;
	s1.z = 0;
	return s1;
}


Vec3 get_vec_ground(){
	Vec3 g1;
	g1.x = 0;
	g1.y = 0;
	g1.z = -1;
	return g1;
}
