#include "stdio.h"
//#include "vv2q.c"
#include "sens.c"
//#include "structs.h"
//#include "mml1.c"

//typedef struct {
//    double x, y, z;
//} Vec3;


Vec3 crossProduct(const Vec3 *A, const Vec3 *B) {
    Vec3 C;
    C.x = A->y * B->z - A->z * B->y;
    C.y = A->z * B->x - A->x * B->z;
    C.z = A->x * B->y - A->y * B->x;
    return C;
}



int main(){
	
	int mode = 0;
	
	Vec3 vec_sun1;
	Vec3 vec_ground1;
	
	Vec3 vec_sun2;
	Vec3 vec_ground2;
	
	struct timespec ts0;
    clock_gettime(CLOCK_REALTIME, &ts0);
    double t0 = ts0.tv_sec + ts0.tv_nsec * 1e-9;
    
    struct timespec tsx;
    clock_gettime(CLOCK_REALTIME, &tsx);
    double tx = tsx.tv_sec + tsx.tv_nsec * 1e-9;
	
	struct timespec ts1;
    clock_gettime(CLOCK_REALTIME, &ts1);
    double t1 = ts1.tv_sec + ts1.tv_nsec * 1e-9;
    
    struct timespec sleep_ts = {0, 33 * 1000 * 1000 * 30};
    //nanosleep(&sleep_ts, NULL);
    
    
    Keyframe keyframes[5];
    Quaternion q_out, q1;
    Vec3 omega, alpha;
    Vec3 B1[3];
    Vec3 B2[3];
    
    const double velocity_dt = 0.001;  // шаг для ω
    const double accel_dt    = 0.01;   // шаг для α
    
    while(mode != 179){
    	//printf("tyt");
		printf("mode: ");
		scanf("%d", &mode);
		//printf("md: %d", mode);
		//printf("tyt2");
		//mode = 1;
		if (mode == 1){
			printf("1\n");
			vec_sun1 = get_vec_sun();
			vec_ground1 = get_vec_ground();
		   	clock_gettime(CLOCK_REALTIME, &ts0);
		   	t0 = ts0.tv_sec + ts0.tv_nsec * 1e-9;
		    B1[0] = vec_sun1;
		    B1[1] = vec_ground1;
		    B1[2] = crossProduct(&vec_sun1, &vec_ground1);
		    
			for (int i = 0; i < 5; i++){
				
				nanosleep(&sleep_ts, NULL);
				
				vec_sun2 = get_vec_sun();
				vec_ground2 = get_vec_ground();
				clock_gettime(CLOCK_REALTIME, &ts1);
		 	    t1 = ts1.tv_sec + ts1.tv_nsec * 1e-9;	
				
				//q1 = vv2qu({vec_sun1, vec_ground1, crossProduct(&vec_sun1, &vec_ground1)}, {vec_sun2, vec_ground2, crossProduct(&vec_sun2, &vec_ground2}]);
				
			    B2[0] = vec_sun2;
			    B2[1] = vec_ground2;
			    B2[2] = crossProduct(&vec_sun2, &vec_ground2);
				q1 = v2qu(B1, B2);
				//keyframes[i] = {t1, q1};
				keyframes[i].time = t1;
				keyframes[i].quat = q1;
				printf("t1: %f\n", t1);
			}
		}
		
		else if (mode == 2){
			printf("2\n");
		    int key_count = sizeof keyframes / sizeof *keyframes;
		    Quaternion* s_points = compute_s_points(keyframes, key_count);
			
			clock_gettime(CLOCK_REALTIME, &tsx);
		   	tx = tsx.tv_sec + tsx.tv_nsec * 1e-9;
		
			//get_kinematics(tx, &q_out, &omega, &alpha, keyframes);
			Kinematics kin = compute_kinematics(
            keyframes, s_points, key_count,
            tx,
            velocity_dt,
            accel_dt
        	);
        	printf(
            "%f	 | (% .3f, % .3f, % .3f, % .3f) | "
            "(% .3f, % .3f, % .3f) | "
            "(% .3f, % .3f, % .3f)\n",
            tx,
            kin.orientation.w,
            kin.orientation.x,
            kin.orientation.y,
            kin.orientation.z,
            kin.angular_velocity.x,
            kin.angular_velocity.y,
            kin.angular_velocity.z,
            kin.angular_accel.x,
            kin.angular_accel.y,
            kin.angular_accel.z
        	);

        
        
			/*printf("tx=%.3f: ori=(% .3f,% .3f,% .3f,% .3f)  ω=(% .3f,% .3f,% .3f)  α=(% .3f,% .3f,% .3f)\n",
       		 tx,
       		 q_out.w, q_out.x, q_out.y, q_out.z,
        	 omega.x, omega.y, omega.z,
           	 alpha.x, alpha.y, alpha.z
    		);
    		*/
		}
		
	}
	//free()
	
	return 0;
}
