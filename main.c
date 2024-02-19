#include "e2d_phys.h"
#define MAX_OBJ 6
#define MAX_SPR 6

typedef struct spring {
    float stiff;            // stiffnes of spring
    float damp;            // damping of spring
    Shape* ball_a;            // ball a 
    Shape* ball_b;            // ball b
    float restlength;
} Spring;

Spring createSpring(float stiffnes, float damping, Shape* ballA, Shape* ballB) {
    Spring result = {
        .stiff = stiffnes,
        .damp = damping,
        .ball_a = ballA,
        .ball_b = ballB,
        .restlength = e2_mag(e2_VecSub(ballB->location, ballA->location))
    };
    return result;
}

void updateSpring(Spring* spr) {
    //SpringForce 
    Vector2 connect = e2_VecSub(spr->ball_b->location, spr->ball_a->location);
    float expand = e2_mag(connect) - spr->restlength;
    e2_vecNorm(&connect);
    Vector2 fs = e2_VecScale(connect, -(spr->stiff * expand));

    //DampingForce
    Vector2 vel_total = e2_VecSub(spr->ball_b->velocity, spr->ball_a->velocity);
    float vel_connect_mag = e2_dot(connect, vel_total);
    Vector2 fd = e2_VecScale(connect, vel_connect_mag * -(spr->damp));

    //applyForce
    Vector2 ft = e2_VecAdd(fd, fs);
    e2_applyForce(spr->ball_a, e2_VecScale(ft, -1), 0);
    e2_applyForce(spr->ball_b, ft, 0);
  }

int main() {

    // Initialization
    //--------------------------------------------------------------------------------------
    
    Shape obj[MAX_OBJ];
    Spring spr[MAX_SPR];
    obj[0] = e2_Ball(50,50, 15);
    obj[1] = e2_Ball(150, 50, 15);
    obj[2] = e2_Ball(50, 100, 15);
    obj[3] = e2_Ball(150, 100, 15);
    obj[4] = e2_Box(5, 470, 600, 30);
    obj[4].mass = INFINITY;
    obj[4].inertia = INFINITY;
    obj[5] = e2_Box(760, 5, 30, 400);
    obj[5].mass = INFINITY;
    
      

    spr[0] = createSpring(0.9, 0.8, &obj[0], &obj[1]);
    spr[1] = createSpring(0.9, 0.8, &obj[1], &obj[3]);
    spr[2] = createSpring(0.9, 0.8, &obj[0], &obj[2]);
    spr[3] = createSpring(0.9, 0.8, &obj[2], &obj[3]);
    spr[4] = createSpring(0.9, 0.8, &obj[0], &obj[3]);
    spr[5] = createSpring(0.9, 0.8, &obj[1], &obj[2]);
    
    const int screenWidth = 1200;
    const int screenHeight = 800; 

    InitWindow(screenWidth, screenHeight, "raylib");
    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second


    // Main game loop
    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        for (size_t i = 0; i < MAX_OBJ; i++) {
            for (size_t j = i + 1; j < MAX_OBJ; j++) {
                e2_checkColl(&obj[i], &obj[j]);
            }
            Vector2 kick = e2_checkKick(&obj[i]);
            e2_applyForce(&obj[i], kick, 0);
            e2_shapeUpdate(&obj[i]);
            e2_shapeDraw(&obj[i], 1, RED);
        }
        for (size_t i = 0; i < MAX_SPR; i++) {
            updateSpring(&spr[i]);    
            DrawLineV(spr[i].ball_a->location, spr[i].ball_b->location, BLACK);
        }        
    	EndDrawing();
    }

    CloseWindow();   
	return 0;


    return EXIT_SUCCESS;
}
