#include "e2d_core.h"

int e2_random(int range_start, int range_end) {
    struct timespec tp;
    clockid_t clk_id;
    clk_id = CLOCK_MONOTONIC;
    clock_gettime(clk_id, &tp);
    srand(tp.tv_nsec);
    return (rand() % (range_end - range_start + 1)) + range_start;
}

float e2_limitNum(float number, float limit) {
	int vorzeichen = (number < 0) ? -1 : 1;
	float numberMag = number * vorzeichen;
	if (numberMag > limit) numberMag = limit;

	return numberMag * vorzeichen;
}

void e2_drawArrow(Vector2 v_base, Vector2 v_target, Color c) {
	DrawLineEx(v_base, v_target, 4, c);
}

Matrix3x1 e2_transVecToMatrix(Vector2 point) {

	return (Matrix3x1){{point.x, point.y, 1}};
}

Vector2 e2_transMatrixToVec(Matrix3x1 matrix) {

	return (Vector2){matrix.index[0], matrix.index[1]};
}

Matrix3x1 e2_multMatrix(Matrix3x3 a, Matrix3x1 b) {
	Matrix3x1 result = {{0, 0, 0}};
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			result.index[i] += a.index[i][j] * b.index[j];
		}
	}
	return result;
}

Matrix3x1 e2_rotateMatrix(Vector2 point, Vector2 center, float angel) {

	Matrix3x1 matrix_point = e2_transVecToMatrix(point);
	Matrix3x1 matrix_center = e2_transVecToMatrix(center);

	Matrix3x3 m_rotate = { {
		{cosf(angel), -sinf(angel), 0},
		{sinf(angel), cosf(angel), 0},
		{0, 0, 1}
		} };

	Matrix3x3 m_transform_center = { {
		{1, 0, -matrix_center.index[0]},
		{0, 1, -matrix_center.index[1]},
		{0, 0, 1}
	} };

	Matrix3x3 m_transform_center_back = { {
		{1, 0, matrix_center.index[0]},
		{0, 1, matrix_center.index[1]},
		{0, 0, 1}
	} };

	Matrix3x1 matrix_p_transformed = e2_multMatrix(m_transform_center, matrix_point);
	Matrix3x1 matrix_p_transformed_rotated = e2_multMatrix(m_rotate, matrix_p_transformed);

	return e2_multMatrix(m_transform_center_back, matrix_p_transformed_rotated);
}



Vector2 e2_Vector(float x, float y) {
    return (Vector2){x, y};
}

void e2_vecSet(Vector2* v, float x, float y) {
	v->x = x;
	v->y = y;
}

void e2_vecScale(Vector2* v, float n) {
	v->x = v->x * n;
	v->y = v->y * n;
}

Vector2 e2_VecScale(Vector2 v, float n) {
	return (Vector2) {v.x * n, v.y * n};
}

void e2_vecDiv(Vector2* v, float n) {
	v->x = v->x/n;
	v->y = v->y/n;	
}

Vector2 e2_VecAdd(Vector2 v1, Vector2 v2) {
	Vector2 result = {v1.x + v2.x, v1.y + v2.y};
	return result;
}

Vector2 e2_VecSub(Vector2 v1, Vector2 v2) {
	Vector2 result = {v1.x - v2.x, v1.y - v2.y};
	return result;
}

Vector2 e2_VecDiv(Vector2 v, float n) {
	Vector2 result = {v.x/n, v.y/n};
	return result;

}

void e2_vecNorm(Vector2* v) {
	float len = sqrtf(v->x * v->x + v->y * v->y);
	if (len > 0.0001) {
		v->x = v->x / len;
		v->y = v->y / len;
	}
}

void e2_vecLimit(Vector2* v, float max) {
	float len = sqrtf(v->x * v->x + v->y * v->y);
	if (len > max) {
		v->x = v->x / len * max;
		v->y = v->y / len * max;
	}
}

void e2_vecSetMag(Vector2* v, float magnitude) {
	float len = sqrtf(v->x * v->x + v->y * v->y);
	if (len > 0) {
		v->x = v->x / len * magnitude;
		v->y = v->y / len * magnitude;
	}
}

float e2_dot(Vector2 v1, Vector2 v2) {
	return v1.x * v2.x + v1.y * v2.y;
}

float e2_cross(Vector2 v1, Vector2 v2) {
	return v1.x * v2.y - v1.y * v2.x;
}

Vector2 e2_VecPerp(Vector2 v) {
	return (Vector2){-v.y, v.x};
}

float e2_mag(Vector2 v) {
	return sqrtf(v.x * v.x + v.y * v.y);
}

float e2_magsq(Vector2 v) {
	return v.x * v.x + v.y * v.y;
}

float e2_dist(Vector2 v1, Vector2 v2) {
	Vector2 vdist = e2_VecSub(v1, v2);
	return e2_mag(vdist);
}

Vector2 e2_VecRotate(Vector2 v, Vector2 base, float n) {
	Vector2 direction = e2_VecSub(v, base);
	float x = direction.x * cosf(n) - direction.y * sinf(n);
	float y = direction.x * sinf(n) + direction.y * cosf(n);
	return (Vector2){x = x + base.x, y = y + base.y};
}

Intersection e2_intersect(Vector2 start_a, Vector2 end_a, Vector2 start_b, Vector2 end_b) {
	Vector2 a = e2_VecSub(end_a, start_a);
	Vector2 b = e2_VecSub(end_b, start_b);
	float cross1 = e2_cross(a, b);
	float cross2 = e2_cross(b, a);
	if (fabs(cross1 - 0.0f) > 0.01) { //Float kann man nicht direkt auf 0 testen!!!
		float s = e2_cross(e2_VecSub(start_b, start_a), b) / cross1;
		float u = e2_cross(e2_VecSub(start_a, start_b), a) / cross2;
		if (s > 0.0001 && s < 1 && u > 0.0001 && u < 1) {
			return (Intersection){s, e2_VecAdd(start_a, e2_VecScale(a, s))};
		}
	}
	return (Intersection){0.0f, (Vector2){0.0f, 0.0f}};
}

float e2_minDist(Vector2 p, Vector2 start_a, Vector2 end_a) {
	float dist = -1.0f;

	//Vektor start_a to end_a (line_a)
	Vector2 line_a = e2_VecSub(end_a, start_a);
	//Vektor imaginary line start_a to p
	Vector2 start_a_to_p = e2_VecSub(p, start_a);
	//Magnitude of line_a
	float magnitude = e2_mag(line_a);

	//Scalarprojecton from line (start_a to p) on line_a
	e2_vecNorm(&line_a);
	float sp = e2_dot(line_a, start_a_to_p);

	//Scalarprojection in magnitude of line_a?
	if (sp > 0.0001 && sp <= magnitude) {
		e2_vecScale(&line_a, sp);
		dist = e2_mag(e2_VecSub(start_a_to_p, line_a));
	}
	return dist;
}