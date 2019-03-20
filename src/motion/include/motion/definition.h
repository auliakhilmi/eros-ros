#ifndef DEFINITION_H
#define DEFINITION_H

//
#define ON true
#define OFF false

//STATE ROBOT
#define CONTROL_ENABLE true
#define READY 0
#define WALK  1
#define KICK  2
#define STANDUP 3
#define FALL 4

//STATE FALL
#define BackFall	1
#define FrontFall	2

#define RAD2DEG     (float) 57.295779513082320876798154814105
#define DEG2RAD     (float) 0.01745329251994329576923690768489
#define Phi         (float) 3.1415926535897932384626433832795
#define DualPhi     (float) 6.283185307179586476925286766559

#define ROBOT_HEIGHT 28


#endif // DEFINITION_H
