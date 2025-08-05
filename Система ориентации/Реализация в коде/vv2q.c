#include <stdio.h>
#include <math.h>
//#include "structs.h"
#include "mml1.c"

/* Структуры для вектора и кватерниона */
//typedef struct {
//    double x, y, z;
//} Vec3;

//typedef struct {
//    double w, x, y, z;
//} Quaternion;

/* Вспомогательные функции */
static Vec3 vec3_cross(const Vec3 *a, const Vec3 *b) {
    return (Vec3){
        a->y * b->z - a->z * b->y,
        a->z * b->x - a->x * b->z,
        a->x * b->y - a->y * b->x
    };
}

static double vec3_dot(const Vec3 *a, const Vec3 *b) {
    return a->x*b->x + a->y*b->y + a->z*b->z;
}

static Vec3 vec3_normalize(const Vec3 *v) {
    double norm = sqrt(vec3_dot(v, v));
    return (Vec3){v->x / norm, v->y / norm, v->z / norm};
}

/* Перевод матрицы 3x3 в кватернион */
static Quaternion mat3_to_quat(double R[3][3]) {
    Quaternion q;
    double trace = R[0][0] + R[1][1] + R[2][2];

    if (trace > 0.0) {
        double s = 0.5 / sqrt(trace + 1.0);
        q.w = 0.25 / s;
        q.x = (R[2][1] - R[1][2]) * s;
        q.y = (R[0][2] - R[2][0]) * s;
        q.z = (R[1][0] - R[0][1]) * s;
    } else {
        if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
            double s = 2.0 * sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
            q.w = (R[2][1] - R[1][2]) / s;
            q.x = 0.25 * s;
            q.y = (R[0][1] + R[1][0]) / s;
            q.z = (R[0][2] + R[2][0]) / s;
        } else if (R[1][1] > R[2][2]) {
            double s = 2.0 * sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
            q.w = (R[0][2] - R[2][0]) / s;
            q.x = (R[0][1] + R[1][0]) / s;
            q.y = 0.25 * s;
            q.z = (R[1][2] + R[2][1]) / s;
        } else {
            double s = 2.0 * sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
            q.w = (R[1][0] - R[0][1]) / s;
            q.x = (R[0][2] + R[2][0]) / s;
            q.y = (R[1][2] + R[2][1]) / s;
            q.z = 0.25 * s;
        }
    }

    return q;
}

/* Главная функция: строит кватернион перехода */
Quaternion build_transition_quat(const Vec3 u[3], const Vec3 v[3]) {
    double R[3][3];

    /* Матрица перехода R = V * U^T */
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] = vec3_dot(&v[i], &u[j]);
        }
    }

    /* Преобразуем матрицу в кватернион */
    return mat3_to_quat(R);
}

Quaternion v2qu(Vec3 va[3], Vec3 vb[3]){
	for (int i = 0; i < 3; ++i) {
        va[i] = vec3_normalize(&va[i]);
    }
    
    for (int i = 0; i < 3; ++i) {
        vb[i] = vec3_normalize(&vb[i]);
    }

    return build_transition_quat(va, vb);
}
/*
int main(void) {
    // Пример: пусть исходная система — стандартная базисная 
    Vec3 u[3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    // Целевая система выполнила поворот на 90° вокруг оси Z
    Vec3 v[3] = {
        { 0, 1, 0},
        {-1, 0, 0},
        { 0, 0, 1}
    };

    for (int i = 0; i < 3; ++i) {
        v[i] = vec3_normalize(&v[i]);
    }

    Quaternion q = build_transition_quat(u, v);

    printf("q = [w=% .6f, x=% .6f, y=% .6f, z=% .6f]\n",
           q.w, q.x, q.y, q.z);

    return 0;
}
*/

