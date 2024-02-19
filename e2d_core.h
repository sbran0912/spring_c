#ifndef CORE_H
#define CORE_H
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include "raylib.h"

typedef struct {
	float index[3][3];
} Matrix3x3;

typedef struct {
	float index[3];
} Matrix3x1;

typedef struct {
	float distance;
	Vector2 point;
} Intersection;

int e2_random(int range_start, int range_end);
float e2_limitNum(float number, float limit);
void e2_drawArrow(Vector2 v_base, Vector2 v_target, Color c);

Matrix3x1 e2_transVecToMatrix(Vector2 point);
Vector2 e2_transMatrixToVec(Matrix3x1 matrix);
Matrix3x1 e2_multMatrix(Matrix3x3 a, Matrix3x1 b);
Matrix3x1 e2_rotateMatrix(Vector2 point, Vector2 center, float angel);

Vector2 e2_Vector(float x, float y);
void e2_vecSet(Vector2* v, float x, float y);
void e2_vecScale(Vector2* v, float n);
Vector2 e2_VecScale(Vector2 v, float n);
void e2_vecDiv(Vector2* v, float n);
Vector2 e2_VecAdd(Vector2 v, Vector2 v_add);
Vector2 e2_VecSub(Vector2 v, Vector2 v_sub);
Vector2 e2_VecDiv(Vector2 v, float n);
void e2_vecNorm(Vector2* v);
void e2_vecLimit(Vector2* v, float max);
void e2_vecSetMag(Vector2* v, float magnitude);
float e2_dot(Vector2 v1, Vector2 v2);
float e2_cross(Vector2 v1, Vector2 v2);
Vector2 e2_VecPerp(Vector2 v);
float e2_mag(Vector2 v);
float e2_magsq(Vector2 v);
float e2_dist(Vector2 v1, Vector2 v2);
Vector2 e2_VecRotate(Vector2 v, Vector2 base, float n);

Intersection e2_intersect(Vector2 start_a, Vector2 end_a, Vector2 start_b, Vector2 end_b);
float e2_minDist(Vector2 p, Vector2 start_a, Vector2 end_a);


#endif