// Структура 3D-вектора
typedef struct Vec3{
    double x, y, z;
} Vec3;

// Структура кватерниона
typedef struct Quaternion{
    double w, x, y, z;
} Quaternion;

// Структура ключевого кадра
typedef struct Keyframe{
    double time;     // Время в секундах
    Quaternion quat; // Ориентация в этот момент
} Keyframe;

// Структура для полной кинематики
typedef struct Kinematics{
    Quaternion orientation; // Ориентация
    Vec3 angular_velocity;  // Угловая скорость (рад/с)
    Vec3 angular_accel;     // Угловое ускорение (рад/с²)
} Kinematics;
