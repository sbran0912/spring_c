#include "e2d_phys.h"

void _drawBox(Shape *box, float thick, Color c) {
    for (int i = 0; i < 4; i++)
    {
        DrawLineEx(box->vertices[i], box->vertices[i + 1], thick, c);
    }
    DrawCircleV(box->location, 5, c);
}

void _rotateBox(Shape *box, float angle) {
    for (int i = 0; i < 5; i++)
    {
        box->vertices[i] = e2_VecRotate(box->vertices[i], box->location, angle);
    }
}

void _updateBox(Shape *box) {
    box->velocity = e2_VecAdd(box->velocity, box->accel);
    e2_vecLimit(&box->velocity, 10.0f);
    e2_vecSet(&box->accel, 0.0f, 0.0f);

    box->angVelocity += box->angAccel;
    e2_limitNum(box->angVelocity, 0.05f);
    box->angAccel = 0.0f;

    box->location = e2_VecAdd(box->location, box->velocity);
    box->vertices[0] = e2_VecAdd(box->vertices[0], box->velocity);
    box->vertices[1] = e2_VecAdd(box->vertices[1], box->velocity);
    box->vertices[2] = e2_VecAdd(box->vertices[2], box->velocity);
    box->vertices[3] = e2_VecAdd(box->vertices[3], box->velocity);
    box->vertices[4] = e2_VecAdd(box->vertices[4], box->velocity);

    _rotateBox(box, box->angVelocity);
}

void _resetPosBox(Shape *box, Vector2 v) {
    if (box->mass != INFINITY)
    {
        box->location = e2_VecAdd(box->location, v);
        box->vertices[0] = e2_VecAdd(box->vertices[0], v);
        box->vertices[1] = e2_VecAdd(box->vertices[1], v);
        box->vertices[2] = e2_VecAdd(box->vertices[2], v);
        box->vertices[3] = e2_VecAdd(box->vertices[3], v);
        box->vertices[4] = e2_VecAdd(box->vertices[4], v);
    }
}

void _drawBall(Shape *ball, float thick, Color c) {
    DrawRing(ball->location, ball->radius - 3, ball->radius, 0, 360, 1, c);
    // DrawCircleV(location.pos,radius, c);
    DrawLineEx(ball->location, ball->orientation, thick, c);
}

void _rotateBall(Shape* ball, float angle) {
    ball->orientation = e2_VecRotate(ball->orientation, ball->location, angle);
}

void _updateBall(Shape* ball) {
    ball->velocity = e2_VecAdd(ball->velocity, ball->accel);
    e2_vecLimit(&ball->velocity, 10.0f);
    e2_vecSet(&ball->accel, 0.0f, 0.0f);

    ball->angVelocity += ball->angAccel;
    e2_limitNum(ball->angVelocity, 0.05f);
    ball->angAccel = 0.0f;

    ball->location = e2_VecAdd(ball->location, ball->velocity);
    ball->orientation = e2_VecAdd(ball->orientation, ball->velocity);

    _rotateBall(ball, ball->angVelocity);
}

void _resetPosBall(Shape *ball, Vector2 v) {
    if (ball->mass != INFINITY) {
        ball->location = e2_VecAdd(ball->location, v);
        ball->orientation = e2_VecAdd(ball->orientation, v);
    }
}

Shadow e2_Shadow(Shape* shape) {
    Shadow shadow;
    if (shape->typ == BALL) {
        shadow = (Shadow){
            .minX = shape->location.x - shape->radius, 
            .maxX = shape->location.x + shape->radius, 
            .minY = shape->location.y - shape->radius, 
            .maxY = shape->location.y + shape->radius
            };
    } else {
        shadow = (Shadow){
            .minX = INFINITY, 
            .maxX =-INFINITY, 
            .minY = INFINITY, 
            .maxY =-INFINITY
            };
        for (int i = 0; i < 4; i++) {
            if (shape->vertices[i].x < shadow.minX) {
                shadow.minX = shape->vertices[i].x;
            } 
            if (shape->vertices[i].y < shadow.minY) {
                shadow.minY = shape->vertices[i].y;
            } 
            if (shape->vertices[i].x > shadow.maxX) {
                shadow.maxX = shape->vertices[i].x;
            } 
            if (shape->vertices[i].y > shadow.maxY) {
                shadow.maxY = shape->vertices[i].y;
            } 
        }    
    }
    
    return shadow;
}

Shape e2_Box(float x, float y, float w, float h) {
    Shape result = {
        .typ = BOX,
        .marked = false,
        .location = {x + w / 2, y + h / 2},
        .mass = (w + h) * 2,
        .inertia = w * h * w,
        .velocity = {0, 0},
        .angVelocity = 0,
        .accel = {0, 0},
        .angAccel = 0,
        .vertices[0] = {x, y},
        .vertices[1] = {x + w, y},
        .vertices[2] = {x + w, y + h},
        .vertices[3] = {x, y + h},
        .vertices[4] = {x, y},
        .funcDraw = &_drawBox,
        .funcUpdate = &_updateBox,
        .funcResetPos = &_resetPosBox};
    return result;
}

Shape e2_Ball(float x, float y, float r) {
    Shape result = {
        .typ = BALL,
        .marked = false,
        .location = {x, y},
        .radius = r,
        .mass = r * 2,
        .inertia = r * r * r / 2,
        .velocity = {0, 0},
        .angVelocity = 0,
        .accel = {0, 0},
        .angAccel = 0,
        .orientation = {r + x, y},
        .funcDraw = &_drawBall,
        .funcUpdate = &_updateBall,
        .funcResetPos = &_resetPosBall};
    return result;
}

void e2_shapeDraw(Shape *shape, float thick, Color c) {
    shape->funcDraw(shape, thick, c);
}

void e2_shapeUpdate(Shape *shape) {
    shape->funcUpdate(shape);
}

void e2_applyForce(Shape *shape, Vector2 force, float angForce) {
    shape->accel = e2_VecAdd(shape->accel, e2_VecDiv(force, shape->mass));
    shape->angAccel += angForce / shape->mass;
}

void e2_shapeResetPos(Shape *shape, Vector2 v) {
    shape->funcResetPos(shape, v);
}

Vector2 e2_checkKick(Shape* shape) {

    Vector2 mousePos = { (float)GetMouseX(), (float)GetMouseY() };

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {

        if (e2_dist(shape->location, mousePos) < 10) {
            shape->marked = true;
        }

        if (shape->marked) {
            e2_drawArrow(shape->location, mousePos, RED);
        }
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && shape->marked) {
        shape->marked = false;
        Vector2 force = e2_VecSub(mousePos, shape->location);
        e2_vecScale(&force, 5);
        return force;
    }

    return (Vector2){0, 0};
}

CollisionPoint e2_detectCollBox(Shape* boxA, Shape* boxB) {
    // Geprüft wird, ob eine Ecke von boxA in die Kante von boxB schneidet
    // Zusätzlich muss die Linie von Mittelpunkt boxA und Mittelpunkt boxB durch Kante von boxB gehen
    // i ist Index von Ecke und j ist Index von Kante
    // d = Diagonale von A.Mittelpunkt zu A.vertices(i)
    // e = Kante von B(j) zu B(j+1)
    // z = Linie von A.Mittelpunkt zu B.Mittelpunkt
    // _perp = Perpendicularvektor
    // scalar_d Faktor von d für den Schnittpunkt d/e
    // scalar_z Faktor von z für den Schnittpunkt z/e
    // mtv = minimal translation vector (überlappender Teil von d zur Kante e)

    for (int i = 0; i < 4; i++) {            
        for (int j = 0; j < 4; j++) {
            // Prüfung auf intersection von Diagonale d zu Kante e
            Intersection isd = e2_intersect(boxA->location, boxA->vertices[i], boxB->vertices[j], boxB->vertices[j + 1]);   
            if (isd.distance > 0.0f) {
                // Prüfung auf intersection Linie z zu Kante e
                Intersection isz = e2_intersect(boxA->location, boxB->location, boxB->vertices[j], boxB->vertices[j + 1]);
                if (isz.distance > 0.0f) {
                    // Collision findet statt
                    // Objekte zurücksetzen und normal_e berechnen. Kollisionspunkt ist Ecke i von BoxA
                    Vector2 e = e2_VecSub(boxB->vertices[j + 1], boxB->vertices[j]);
                    Vector2 e_perp = {-(e.y), e.x};
                    Vector2 d = e2_VecSub(boxA->vertices[i], boxA->location);  
                    e2_vecScale(&d, 1-isd.distance);
                    e2_vecNorm(&e_perp);
                    float distance = e2_dot(e_perp, d);
                    e2_vecScale(&e_perp, -distance); //mtv
                    e2_shapeResetPos(boxA, e2_VecScale(e_perp, 0.5f));
                    e2_shapeResetPos(boxB, e2_VecScale(e_perp, -0.5f));
                    e2_vecNorm(&e_perp); // normal_e
                    return (CollisionPoint){true, boxA->vertices[i], e_perp};                }
            }
        }
    }
    return (CollisionPoint) {false, {0.0f, 0.0f}, {0.0f, 0.0f}};
};

CollisionPoint e2_detectCollBall(Shape* ballA, Shape* ballB) {
    //Distanz ermitteln
    float radiusTotal = ballA->radius + ballB->radius;
    float dist = e2_dist(ballA->location, ballB->location);
    if (dist < radiusTotal) {
        //Treffer
        float space = (radiusTotal - dist);
        Vector2 collisionLine = e2_VecSub(ballA->location, ballB->location);
        e2_vecSetMag(&collisionLine, space);
        e2_shapeResetPos(ballA, e2_VecScale( collisionLine, 0.5));
        e2_shapeResetPos(ballB, e2_VecScale( collisionLine, -0.5));
        e2_vecNorm(&collisionLine);
        return (CollisionPoint) {true, {0.0f, 0.0f}, collisionLine};
    }
    return (CollisionPoint) {false, {0.0f, 0.0f}, {0.0f, 0.0f}};
}

CollisionPoint e2_detectCollBallBox(Shape* ball, Shape* box) {
    for (int i = 0; i < 4; i++) {
        //Kante der Box
        Vector2 e = e2_VecSub(box->vertices[i+1], box->vertices[i]);
        //Vektor von Ecke der Box zum Ball
        Vector2 VerticeToBall = e2_VecSub(ball->location, box->vertices[i]);
        //Kollision mit Ecken abfangen
        if (e2_mag(VerticeToBall) < ball->radius) {
            return (CollisionPoint){true, box->vertices[i], VerticeToBall};
        }
        float mag_e = e2_mag(e);
        e2_vecNorm(&e);
        //Scalarprojektion von Vektor VerticeToBall auf Kante e
        float scalar_e = e2_dot(VerticeToBall, e);
        if (scalar_e > 0 && scalar_e <= mag_e) {
            //Senkrechte von Ball trifft auf Kante e der Box
            //e2 = Kante e mit der Länge von scalar_e
            Vector2 e2 = e2_VecScale(e, scalar_e);
            //Senkrechte von e zum Ball = VerticeToBall - e2
            Vector2 e_perp = e2_VecSub(VerticeToBall, e2);

            if (e2_mag(e_perp) < ball->radius) {
                //Ball berührt Box
                //Abstand wieder herstellen mit mtv (minimal translation vector)
                Vector2 mtv = e_perp;
                Vector2 p = e2_VecAdd(box->vertices[i], e2);
                e2_vecSetMag(&mtv, ball->radius - e2_mag(e_perp));
                //e_perp und damit mtv zeigt von Kante zu Ball
                e2_shapeResetPos(ball, mtv);
                //vor Berechnung muss e_perp normalisiert werden
                e2_vecNorm(&e_perp);
                //resolveCollisionBallBox(ball, box, p, e_perp)
                return (CollisionPoint){true, p, e_perp};
            }
        }
    }
    return (CollisionPoint) {false, {0.0f, 0.0f}, {0.0f, 0.0f}};
}

void e2_resolveCollBox(Shape* boxA, Shape* boxB, Vector2 cp, Vector2 normal) {
    // rAP = Linie von A.location zu Kollisionspunkt (Ecke i von BoxA)
    Vector2 rAP = e2_VecSub(cp, boxA->location);
    // rBP = Linie von B.location zu Kollisionspunkt (ebenfalls Ecke i von BoxA)
    Vector2 rBP = e2_VecSub(cp, boxB->location);
    Vector2 rAP_perp = {-rAP.y, rAP.x};
    Vector2 rBP_perp = {-rBP.y, rBP.x};
    Vector2 VtanA = e2_VecScale(rAP_perp, boxA->angVelocity);
    Vector2 VtanB = e2_VecScale(rBP_perp, boxB->angVelocity);
    Vector2 VgesamtA = e2_VecAdd(boxA->velocity, VtanA);
    Vector2 VgesamtB = e2_VecAdd(boxB->velocity, VtanB);
    Vector2 velocity_AB = e2_VecSub(VgesamtA, VgesamtB);
    if (e2_dot(velocity_AB, normal) < 0) { // wenn negativ, dann auf Kollisionskurs
        float e = 1.0f; //inelastischer Stoß
        float j_denominator = e2_dot(e2_VecScale(velocity_AB, -(1+e)), normal);
        float j_divLinear = e2_dot(normal, e2_VecScale(normal, (1/boxA->mass + 1/boxB->mass)));
        float j_divAngular = (float)pow(e2_dot(rAP_perp, normal), 2) / boxA->inertia + (float)pow(e2_dot(rBP_perp, normal), 2) / boxB->inertia;
        float j = j_denominator / (j_divLinear + j_divAngular);
        // Grundlage für Friction berechnen (t)
        Vector2 t = {-(normal.y), normal.x};
        float t_scalarprodukt = e2_dot(velocity_AB, t);
        e2_vecScale(&t, (t_scalarprodukt));
        e2_vecNorm(&t);

        //apply Force        
        Vector2 force = e2_VecAdd(e2_VecScale(normal, (j/boxA->mass)), e2_VecScale(t, (0.2*-j/boxA->mass)));
        float force_ang = e2_dot(rAP_perp, e2_VecAdd(e2_VecScale(normal, j/boxA->inertia), e2_VecScale(t, 0.2*-j/boxA->inertia)));
        boxA->accel = e2_VecAdd(boxA->accel, force);
        boxA->angAccel += force_ang;

        force = e2_VecAdd(e2_VecScale(normal, (-j/boxB->mass)), e2_VecScale(t, (0.2*j/boxB->mass)));
        force_ang = e2_dot(rAP_perp, e2_VecAdd(e2_VecScale(normal, -j/boxB->inertia), e2_VecScale(t, 0.2*j/boxB->inertia)));
        boxB->accel = e2_VecAdd(boxB->accel, force);
        boxB->angAccel += force_ang;

    }
}

void e2_resolveCollBall(Shape* ballA, Shape* ballB, Vector2 normal) {
    Vector2 rA = e2_VecScale(normal, -ballA->radius);
    Vector2 rA_perp = {-rA.y, rA.x};
    Vector2 rB = e2_VecScale(normal, ballB->radius);
    Vector2 rB_perp = {-rB.y, rB.x};
    Vector2 VtanA = e2_VecScale(rA_perp, ballA->angVelocity);
    Vector2 VtanB = e2_VecScale(rB_perp, ballB->angVelocity);
    Vector2 VgesamtA = e2_VecAdd(ballA->velocity, VtanA);
    Vector2 VgesamtB = e2_VecAdd(ballB->velocity, VtanB);
    Vector2 velocity_AB = e2_VecSub(VgesamtA, VgesamtB);   
    
    if (e2_dot(velocity_AB, normal) < 0) { // wenn negativ, dann auf Kollisionskurs
        float e = 1; //inelastischer Stoß
        float j_denominator = e2_dot(e2_VecScale(velocity_AB, -(1+e)), normal);
        float j_divLinear = e2_dot(normal, e2_VecScale(normal, (1/ballA->mass + 1/ballB->mass)));
        float j = j_denominator / j_divLinear;
        // Grundlage für Friction berechnen
        Vector2 t = {-normal.y, normal.x};
        float t_scalarprodukt = e2_dot(velocity_AB, t);
        e2_vecScale(&t, t_scalarprodukt);
        e2_vecNorm(&t);

        //apply Force
        Vector2 force = e2_VecAdd(e2_VecScale(normal, (0.8*j/ballA->mass)), e2_VecScale(t, (0.2*-j/ballA->mass)));
        float force_ang = e2_dot(rA_perp, e2_VecScale(t, 0.1*-j/ballA->inertia));
        ballA->accel = e2_VecAdd(ballA->accel, force);
        ballA->angAccel += force_ang;

        force = e2_VecAdd(e2_VecScale(normal, (0.8*-j/ballB->mass)), e2_VecScale(t, (0.2*j/ballB->mass)));
        force_ang = e2_dot(rB_perp, e2_VecScale(t, 0.1*j/ballB->inertia));
        ballB->accel = e2_VecAdd(ballB->accel, force);
        ballB->angAccel += force_ang;
    }
}

void e2_resolveCollBallBox(Shape* ball, Shape* box, Vector2 cp, Vector2 normal) {
    Vector2 rA = e2_VecScale(normal, -ball->radius);
    Vector2 rA_perp = {-rA.y, rA.x};
    Vector2 rBP = e2_VecSub(cp, box->location);
    Vector2 rBP_perp = {-rBP.y, rBP.x};
    Vector2 VtanA = e2_VecScale(rA_perp, ball->angVelocity);
    Vector2 VgesamtA = e2_VecAdd(ball->velocity, VtanA);
    Vector2 VtanB = e2_VecScale(rBP_perp, box->angVelocity);
    Vector2 VgesamtB = e2_VecAdd(box->velocity, VtanB);
    Vector2 velocity_AB = e2_VecSub(VgesamtA, VgesamtB);

    if (e2_dot(velocity_AB, normal) < 0) { // wenn negativ, dann auf Kollisionskurs

        float e = 1; //inelastischer Stoß
        float j_denominator = e2_dot(e2_VecScale(velocity_AB, -(1+e)), normal);
        float j_divLinear = e2_dot(normal, e2_VecScale(normal, (1/ball->mass + 1/box->mass)));
        float j_divAngular = (float)pow(e2_dot(rBP_perp, normal), 2) / box->inertia; //nur für Box zu rechnen
        float j = j_denominator / (j_divLinear + j_divAngular);
        // Grundlage für Friction berechnen
        Vector2 t = {-normal.y, normal.x};
        float t_scalarprodukt = e2_dot(velocity_AB, t);
        e2_vecScale(&t, t_scalarprodukt);
        e2_vecNorm(&t);
        // Apply Force
        Vector2 force = e2_VecAdd(e2_VecScale(normal, (0.8*j/ball->mass)), e2_VecScale(t, (0.05*-j/ball->mass)));
        float force_ang = e2_dot(rA_perp, e2_VecScale(t, 0.05*-j/ball->inertia));
        ball->accel = e2_VecAdd(ball->accel, force);
        ball->angAccel += force_ang;
           
        force = e2_VecAdd(e2_VecScale(normal, (-j/box->mass)), e2_VecScale(t, (0.05*j/box->mass)));
        force_ang = e2_dot(rBP_perp, e2_VecAdd(e2_VecScale(normal, -j/box->inertia), e2_VecScale(t, 0.05*j/box->inertia)));
        box->accel = e2_VecAdd(box->accel, force);
        box->angAccel += force_ang;
    }
}

void e2_checkColl(Shape* shapeA, Shape* shapeB) {
    //Shadow berechnen von Element i und Element j 
    Shadow shadowA = e2_Shadow(shapeA);
    Shadow shadowB = e2_Shadow(shapeB);
    //Überschneidung prüfen
    if (shadowA.maxX >= shadowB.minX && shadowA.minX <= shadowB.maxX && shadowA.maxY >= shadowB.minY && shadowA.minY <= shadowB.maxY) {  
        //dann Überschneidung
        // Testcode
        DrawLineV(shapeA->location, shapeB->location, GREEN);
        // Ende Testcodew

        if (shapeA->typ == BALL) {
            if (shapeB->typ == BALL) {
                CollisionPoint cp = e2_detectCollBall(shapeA, shapeB);
                if (cp.isCollision) {
                    e2_resolveCollBall(shapeA, shapeB, cp.normal);
                }
            } else {
                CollisionPoint cp = e2_detectCollBallBox(shapeA, shapeB);
                if (cp.isCollision) {
                    e2_resolveCollBallBox(shapeA, shapeB, cp.cp, cp.normal);
                }
            }
        }

        if (shapeA->typ == BOX) {
            if (shapeB->typ == BOX) {
                // beide Boxen müssen geprüft werden, ob sie auf
                // die jeweils andere trefen könnte
                CollisionPoint cp = e2_detectCollBox(shapeA, shapeB);
                if (cp.isCollision) {
                    e2_resolveCollBox(shapeA, shapeB, cp.cp, cp.normal);  
                } else {
                    CollisionPoint cp = e2_detectCollBox(shapeA, shapeB);    
                    if (cp.isCollision) {
                        e2_resolveCollBox(shapeA, shapeB, cp.cp, cp.normal);
                    }
                }
            } else {
                CollisionPoint cp = e2_detectCollBallBox(shapeB, shapeA);
                if (cp.isCollision) {
                    e2_resolveCollBallBox(shapeB, shapeA, cp.cp, cp.normal);
                }
            }            
        }
    }
}