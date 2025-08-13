#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "structs.h"


// Вспомогательные функции ----------------------------------------------------

// Нормализация кватерниона
Quaternion normalize(Quaternion q) {
    double norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (norm < 1e-12) return (Quaternion){1, 0, 0, 0};
    return (Quaternion){q.w/norm, q.x/norm, q.y/norm, q.z/norm};
}

// Скалярное произведение кватернионов
double dot(Quaternion a, Quaternion b) {
    return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

// Умножение кватернионов
Quaternion multiply(Quaternion a, Quaternion b) {
    return (Quaternion){
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

// Сопряжение кватерниона
Quaternion conjugate(Quaternion q) {
    return (Quaternion){q.w, -q.x, -q.y, -q.z};
}

// Логарифм кватерниона (возвращает «чистый» кватернион с w=0)
Quaternion quat_log(Quaternion q) {
    q = normalize(q);
    double v_norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
    double theta  = atan2(v_norm, q.w);

    if (v_norm < 1e-12) {
        // нет векторной части → логарифм нулевой
        return (Quaternion){0, 0, 0, 0};
    }

    double factor = theta / v_norm;
    return (Quaternion){0, factor*q.x, factor*q.y, factor*q.z};
}

// Экспонента «чистого» кватерниона (w=0 → вектор)
Quaternion quat_exp(Quaternion v) {
    double theta = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);

    if (theta < 1e-12) {
        // мало угла → близко к единичному кватерниону
        return (Quaternion){1, 0, 0, 0};
    }

    double sin_t   = sin(theta);
    double cos_t   = cos(theta);
    double factor  = sin_t / theta;

    return (Quaternion){cos_t, factor*v.x, factor*v.y, factor*v.z};
}

// SLERP интерполяция
Quaternion slerp(Quaternion q0, Quaternion q1, double t) {
    q0 = normalize(q0);
    q1 = normalize(q1);

    double cos_ht = dot(q0, q1);

    // выбираем кратчайший путь
    if (cos_ht < 0.0) {
        cos_ht = -cos_ht;
        q1.w = -q1.w;
        q1.x = -q1.x;
        q1.y = -q1.y;
        q1.z = -q1.z;
    }

    // если углы малы, просто линейная интерполяция
    if (fabs(cos_ht) > 0.9995) {
        Quaternion qr = {
            q0.w + t*(q1.w - q0.w),
            q0.x + t*(q1.x - q0.x),
            q0.y + t*(q1.y - q0.y),
            q0.z + t*(q1.z - q0.z),
        };
        return normalize(qr);
    }

    double half_theta   = acos(cos_ht);
    double sin_ht       = sqrt(1.0 - cos_ht*cos_ht);
    double ratio0       = sin((1 - t)*half_theta) / sin_ht;
    double ratio1       = sin(t*half_theta)       / sin_ht;

    return (Quaternion){
        ratio0*q0.w + ratio1*q1.w,
        ratio0*q0.x + ratio1*q1.x,
        ratio0*q0.y + ratio1*q1.y,
        ratio0*q0.z + ratio1*q1.z
    };
}

// Вычисление промежуточных S-точек для SQUAD
Quaternion* compute_s_points(Keyframe* keyframes, int count) {
    Quaternion* s = malloc(sizeof(Quaternion) * count);

    // Крайние точки совпадают с ключевыми
    s[0]        = keyframes[0].quat;
    s[count-1]  = keyframes[count-1].quat;

    for (int i = 1; i < count-1; i++) {
        Quaternion q_i_inv = conjugate(keyframes[i].quat);

        // относительные кватернионы
        Quaternion prev_rel = multiply(q_i_inv, keyframes[i-1].quat);
        Quaternion next_rel = multiply(q_i_inv, keyframes[i+1].quat);

        // logarithm of the relatives
        Quaternion log_prev = quat_log(prev_rel);
        Quaternion log_next = quat_log(next_rel);

        // combined „half‐tangent“
        Quaternion log_comb = {
            0,
            -0.25*(log_prev.x + log_next.x),
            -0.25*(log_prev.y + log_next.y),
            -0.25*(log_prev.z + log_next.z)
        };

        Quaternion exp_comb = quat_exp(log_comb);

        // s[i] = q[i] * exp_comb
        s[i] = multiply(keyframes[i].quat, exp_comb);
    }

    return s;
}

// SQUAD на отрезке [q0, q1] с S‐точками s0, s1
Quaternion squad_segment(
    Quaternion q0, Quaternion q1,
    Quaternion s0, Quaternion s1,
    double     t
) {
    Quaternion c1 = slerp(q0, q1, t);
    Quaternion c2 = slerp(s0, s1, t);
    double     w  = 2.0 * t * (1.0 - t);
    return slerp(c1, c2, w);
}

// Общая SQUAD‐интерполяция на всем наборе ключей
Quaternion squad_interpolation(
    Keyframe*   keyframes,
    Quaternion* s_points,
    int         count,
    double      t
) {
    // выход за диапазон
    if (t <= keyframes[0].time)        return keyframes[0].quat;
    if (t >= keyframes[count-1].time)  return keyframes[count-1].quat;

    // находим сегмент
    int i = 0;
    while (t > keyframes[i+1].time) i++;

    // локальное время [0..1]
    double dt = keyframes[i+1].time - keyframes[i].time;
    double u  = (t - keyframes[i].time) / dt;

    return squad_segment(
        keyframes[i].quat,
        keyframes[i+1].quat,
        s_points[i],
        s_points[i+1],
        u
    );
}

// Численная скорость угловой ориентации
Vec3 compute_angular_velocity(
    Quaternion q_cur,
    Quaternion q_next,
    double     dt
) {
    // dq/dt
    Quaternion dq = {
        (q_next.w - q_cur.w)/dt,
        (q_next.x - q_cur.x)/dt,
        (q_next.y - q_cur.y)/dt,
        (q_next.z - q_cur.z)/dt
    };

    // ω_quat = 2 * q⁻¹ * dq, берём векторную часть
    Quaternion omega_q = multiply(conjugate(q_cur), dq);
    return (Vec3){
        2.0*omega_q.x,
        2.0*omega_q.y,
        2.0*omega_q.z
    };
}

// Численная угловая ускорение
Vec3 compute_angular_accel(
    Vec3 omega_cur,
    Vec3 omega_next,
    double dt
) {
    return (Vec3){
        (omega_next.x - omega_cur.x)/dt,
        (omega_next.y - omega_cur.y)/dt,
        (omega_next.z - omega_cur.z)/dt
    };
}

// Собираем полную кинематику (ориентация + ω + α)
Kinematics compute_kinematics(
    Keyframe*   keyframes,
    Quaternion* s_points,
    int         count,
    double      t,
    double      velocity_dt,
    double      accel_dt
) {
    Kinematics K;

    // ориентация в t
    K.orientation = squad_interpolation(keyframes, s_points, count, t);

    // ориентация чуть позже для скорости и ускорения
    Quaternion q_vel_next = squad_interpolation(keyframes, s_points, count, t + velocity_dt);
    Quaternion q_acc_next = squad_interpolation(keyframes, s_points, count, t + accel_dt);

    // угловая скорость
    K.angular_velocity = compute_angular_velocity(K.orientation, q_vel_next, velocity_dt);

    // вычисляем новую ω для accel
    Vec3 omega2 = compute_angular_velocity(q_vel_next, q_acc_next, velocity_dt);
    K.angular_accel = compute_angular_accel(K.angular_velocity, omega2, accel_dt);

    return K;
}
// Глобальный массив ключевых кадров (с «относительным» временем)
/*Keyframe keyframes[] = {
    {0.0, {1.0,    0.0,    0.0,    0.0}},    // t=0
    {1.0, {0.7071, 0.0,    0.7071, 0.0}},    // t=1
    {2.0, {0.0,    0.7071, 0.0,    0.7071}}, // t=2
    {3.0, {0.5,    0.5,    0.5,    0.5}},    // t=3
    {4.0, {0.0,    0.0,    0.0,    1.0}}     // t=4
};
*/
/*int main(void) {
    // 1) Берём текущее время – это «начало анимации»
    struct timespec ts0;
    clock_gettime(CLOCK_REALTIME, &ts0);
    double t0 = ts0.tv_sec + ts0.tv_nsec * 1e-9;

    // 2) Объявляем ключевые кадры с абсолютными временами
    Keyframe keyframes[] = {
        { t0 + 0.0,    {1.0,    0.0,    0.0,    0.0} },   // старт: t = t0 + 0
        { t0 + 1.0,    {0.7071, 0.0,    0.7071, 0.0} },   // через 1 сек
        { t0 + 2.0,    {0.0,    0.7071, 0.0,    0.7071}}, // через 2 сек
        { t0 + 3.0,    {0.5,    0.5,    0.5,    0.5}},    // через 3 сек
        { t0 + 4.0,    {0.0,    0.0,    0.0,    1.0}}     // через 4 сек
    };
    int key_count = sizeof keyframes / sizeof *keyframes;

    // 3) Предвычисляем S-точки для SQUAD
    Quaternion* s_points = compute_s_points(keyframes, key_count);

    // 4) Параметры численного дифференцирования
    const double velocity_dt = 0.001;  // шаг для ω
    const double accel_dt    = 0.01;   // шаг для α

    // 5) Заголовок вывода
    printf("AbsTime              | Ori(w,x,y,z)               | "
           "  ω(x,y,z)                |  α(x,y,z)\n");
    printf("----------------------------------------------------------------------------------------\n");

    // 6) Цикл до конца анимации
    while (1) {
        // текущее абсолютное время
        struct timespec ts_now;
        clock_gettime(CLOCK_REALTIME, &ts_now);
        double t_now = ts_now.tv_sec + ts_now.tv_nsec * 1e-9;

        // если мы прошли все ключи — выходим
        if (t_now > keyframes[key_count-1].time) break;

        // считаем кинематику на момент t_now
        Kinematics kin = compute_kinematics(
            keyframes, s_points, key_count,
            t_now,
            velocity_dt,
            accel_dt
        );

        // форматируем абсолютное время для печати
        time_t sec = ts_now.tv_sec;
        struct tm tm;
        localtime_r(&sec, &tm);
        char timebuf[32];
        strftime(timebuf, sizeof(timebuf), "%F %T", &tm);

        // выводим строку
        printf(
            "%s | (% .3f, % .3f, % .3f, % .3f) | "
            "(% .3f, % .3f, % .3f) | "
            "(% .3f, % .3f, % .3f)\n",
            timebuf,
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

        // небольшая пауза, чтобы ~30 FPS
        struct timespec sleep_ts = {0, 33 * 1000 * 1000};
        nanosleep(&sleep_ts, NULL);
    }

    free(s_points);
    return 0;
}

*/

