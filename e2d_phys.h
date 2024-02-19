#ifndef PHYS_H
#define PHYS_H
#include "e2d_core.h"

enum figure {BOX, BALL};

typedef struct {
    float minX;
    float maxX;
    float minY;
    float maxY;
} Shadow;

typedef struct {
    bool isCollision;
    Vector2 cp;
    Vector2 normal;
} CollisionPoint;

typedef struct shape_t {
    enum figure typ;
    Vector2 location;
    Vector2 velocity;
    float angVelocity;
    Vector2 accel;
    float angAccel;
    float mass;
    float inertia;
    bool marked;
    float radius;
    Vector2 orientation;
    Vector2 vertices[5];
    void (*funcDraw)(struct shape_t*, float, Color);
    void (*funcUpdate)(struct shape_t*);
    void (*funcResetPos)(struct shape_t*, Vector2);
} Shape;

Shape e2_Box(float x, float y, float w, float h);
Shape e2_Ball(float x, float y, float r);
Shadow e2_Shadow(Shape* shape);

void e2_shapeDraw(Shape* shape, float thick, Color c);
void e2_shapeUpdate(Shape* shape);
void e2_applyForce(Shape* shape, Vector2 force, float angForce);
void e2_shapeResetPos(Shape* shape, Vector2 v);
Vector2 e2_checkKick(Shape* shape);

CollisionPoint e2_detectCollBox(Shape* boxA, Shape* boxB);
CollisionPoint e2_detectCollBall(Shape* ballA, Shape* ballB);
CollisionPoint e2_detectCollBallBox(Shape* ball, Shape* box);
void e2_resolveCollBox(Shape* boxA, Shape* boxB, Vector2 cp, Vector2 normal);
void e2_resolveCollBall(Shape* ballA, Shape* ballB, Vector2 normal);
void e2_resolveCollBallBox(Shape* ball, Shape* box, Vector2 cp, Vector2 normal);
void e2_checkColl(Shape* shapeA, Shape* shapeB);


#endif
