/**
 * Chipmunk2D + SDL2 物理演示
 *
 * 一个简单的 2D 物理沙盒：
 * - 圆形和方形物体在重力作用下下落
 * - 物体之间以及物体与墙壁之间发生碰撞
 * - 使用 SDL2 进行窗口管理和渲染
 * - 使用 Chipmunk2D 进行物理模拟
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "chipmunk/chipmunk.h"
#include "chipmunk/chipmunk_structs.h"
#include <SDL2/SDL.h>

#define WINDOW_WIDTH 900
#define WINDOW_HEIGHT 640

/* ── 坐标系转换 ────────────────────────────────────────── */
/* Chipmunk 坐标 → 屏幕像素坐标 */
#define SCALE 30.0f

static cpVect pixelToWorld(int px, int py)
{
    return cpv((px - WINDOW_WIDTH / 2.0) / SCALE,
               -(py - WINDOW_HEIGHT / 2.0) / SCALE);
}

static void worldToPixel(cpVect w, int *px, int *py)
{
    *px = (int)(w.x * SCALE + WINDOW_WIDTH / 2);
    *py = (int)(-w.y * SCALE + WINDOW_HEIGHT / 2);
}

/* ── 绘制函数 ──────────────────────────────────────────── */

static void drawFilledCircle(SDL_Renderer *ren, int cx, int cy,
                             int r, Uint8 cr, Uint8 cg, Uint8 cb)
{
    SDL_SetRenderDrawColor(ren, cr, cg, cb, 220);
    for (int dy = -r; dy <= r; dy++)
    {
        int dx = (int)sqrt((double)(r * r - dy * dy));
        SDL_RenderDrawLine(ren, cx - dx, cy + dy, cx + dx, cy + dy);
    }
    /* 轮廓 */
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 90);
    for (int dy = -r; dy <= r; dy++)
    {
        int dx = (int)sqrt((double)(r * r - dy * dy));
        SDL_RenderDrawPoint(ren, cx - dx, cy + dy);
        SDL_RenderDrawPoint(ren, cx + dx, cy + dy);
    }
}

static void drawFilledPoly(SDL_Renderer *ren, const SDL_Point *pts,
                           int n, Uint8 cr, Uint8 cg, Uint8 cb)
{
    /* 扫描线填充 */
    int minY = pts[0].y, maxY = pts[0].y;
    for (int i = 1; i < n; i++)
    {
        if (pts[i].y < minY)
            minY = pts[i].y;
        if (pts[i].y > maxY)
            maxY = pts[i].y;
    }

    SDL_SetRenderDrawColor(ren, cr, cg, cb, 200);
    for (int py = minY; py <= maxY; py++)
    {
        int xs[16], cnt = 0;
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            int y1 = pts[i].y, y2 = pts[j].y;
            if ((y1 <= py && y2 > py) || (y2 <= py && y1 > py))
            {
                int ix = pts[i].x +
                         (py - y1) * (pts[j].x - pts[i].x) / (y2 - y1);
                if (cnt < 16)
                    xs[cnt++] = ix;
            }
        }
        /* 冒泡排序 */
        for (int a = 0; a < cnt; a++)
            for (int b = a + 1; b < cnt; b++)
                if (xs[a] > xs[b])
                {
                    int t = xs[a];
                    xs[a] = xs[b];
                    xs[b] = t;
                }
        for (int a = 0; a + 1 < cnt; a += 2)
            SDL_RenderDrawLine(ren, xs[a], py, xs[a + 1], py);
    }

    /* 轮廓 */
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 140);
    SDL_Point *loop = malloc((n + 1) * sizeof(SDL_Point));
    for (int i = 0; i < n; i++)
        loop[i] = pts[i];
    loop[n] = pts[0];
    SDL_RenderDrawLines(ren, loop, n + 1);
    free(loop);
}

/* ── 形状绘制（遍历空间中的每个形状） ─────────────────── */

typedef struct
{
    SDL_Renderer *ren;
    int count;
} DrawContext;

static void drawShape(cpShape *shape, void *data)
{
    DrawContext *ctx = (DrawContext *)data;
    cpBody *body = cpShapeGetBody(shape);
    if (!body)
        return;

    cpVect pos = cpBodyGetPosition(body);
    cpFloat ang = cpBodyGetAngle(body);

    /* 静态物体（墙壁）不绘制，稍后单独画 */
    if (cpBodyGetType(body) == CP_BODY_TYPE_STATIC)
        return;

    /* 根据索引分配颜色 */
    int ci = ctx->count % 7;
    static Uint8 colors[7][3] = {
        {255, 100, 100}, {100, 200, 255}, {255, 220, 70}, {80, 255, 140}, {255, 160, 60}, {200, 140, 255}, {100, 255, 255}};

    /* 根据形状类型绘制 */
    if (shape->klass->type == CP_CIRCLE_SHAPE)
    {
        cpFloat radius = cpCircleShapeGetRadius(shape);
        int cx, cy;
        worldToPixel(pos, &cx, &cy);
        int r = (int)(radius * SCALE);
        if (r < 3)
            r = 3;
        drawFilledCircle(ctx->ren, cx, cy, r,
                         colors[ci][0], colors[ci][1], colors[ci][2]);
        /* 方向指示线 */
        int lx = cx + (int)(r * cos(ang));
        int ly = cy + (int)(r * sin(ang));
        SDL_SetRenderDrawColor(ctx->ren, 255, 255, 255, 180);
        SDL_RenderDrawLine(ctx->ren, cx, cy, lx, ly);
        ctx->count++;
    }
    else if (shape->klass->type == CP_POLY_SHAPE)
    {
        int vc = cpPolyShapeGetCount(shape);
        if (vc >= 3)
        {
            SDL_Point *pts = malloc(vc * sizeof(SDL_Point));
            for (int i = 0; i < vc; i++)
            {
                cpVect wv = cpBodyLocalToWorld(body, cpPolyShapeGetVert(shape, i));
                worldToPixel(wv, &pts[i].x, &pts[i].y);
            }
            drawFilledPoly(ctx->ren, pts, vc,
                           colors[ci][0], colors[ci][1], colors[ci][2]);
            free(pts);
            ctx->count++;
        }
    }
}

/* ── 主函数 ────────────────────────────────────────────── */

#ifdef main
#undef main
#endif

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    srand((unsigned)time(NULL));

    /* ---- SDL 初始化 ---- */
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        fprintf(stderr, "SDL_Init 失败: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *win = SDL_CreateWindow(
        "Chipmunk2D + SDL2 物理演示  |  ESC退出  R重置",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
    if (!win)
    {
        fprintf(stderr, "创建窗口失败: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer *ren = SDL_CreateRenderer(
        win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!ren)
    {
        fprintf(stderr, "创建渲染器失败: %s\n", SDL_GetError());
        SDL_DestroyWindow(win);
        SDL_Quit();
        return 1;
    }

    /* ---- Chipmunk2D 初始化 ---- */
    cpSpace *space = cpSpaceNew();
    cpSpaceSetGravity(space, cpv(0, -9.8));
    cpSpaceSetDamping(space, 0.85);
    cpSpaceSetIterations(space, 15);

    cpBody *staticBody = cpSpaceGetStaticBody(space);

    /* 创建边界墙 */
    cpShape *ground = cpSegmentShapeNew(staticBody, cpv(-16, -9.5), cpv(16, -9.5), 0.6);
    cpShapeSetFriction(ground, 0.9);
    cpShapeSetElasticity(ground, 0.2);
    cpSpaceAddShape(space, ground);

    cpShape *lwall = cpSegmentShapeNew(staticBody, cpv(-16, -9.5), cpv(-16, 12), 0.6);
    cpShapeSetFriction(lwall, 0.5);
    cpShapeSetElasticity(lwall, 0.3);
    cpSpaceAddShape(space, lwall);

    cpShape *rwall = cpSegmentShapeNew(staticBody, cpv(16, -9.5), cpv(16, 12), 0.6);
    cpShapeSetFriction(rwall, 0.5);
    cpShapeSetElasticity(rwall, 0.3);
    cpSpaceAddShape(space, rwall);

    /* 在空间中放一个斜面，增加视觉效果 */
    cpShape *ramp = cpSegmentShapeNew(staticBody, cpv(-10, -5), cpv(-2, -2), 0.3);
    cpShapeSetFriction(ramp, 0.8);
    cpShapeSetElasticity(ramp, 0.1);
    cpSpaceAddShape(space, ramp);

    /* ---- 创建动态物体 ---- */
#define NUM_OBJECTS 20
    cpBody *bodies[NUM_OBJECTS];
    cpShape *shapes[NUM_OBJECTS];

    for (int i = 0; i < NUM_OBJECTS; i++)
    {
        cpFloat mass = 1.0;
        cpBody *body;
        cpShape *shape;

        cpFloat x = -12 + (rand() % 240) / 10.0;
        cpFloat y = 2 + i * 0.7;

        if (i < 8)
        {
            /* 圆形 */
            cpFloat r = 0.35 + (rand() % 4) * 0.12;
            cpFloat moment = cpMomentForCircle(mass, 0, r, cpvzero);
            body = cpBodyNew(mass, moment);
            cpBodySetPosition(body, cpv(x, y));
            shape = cpCircleShapeNew(body, r, cpvzero);
        }
        else
        {
            /* 矩形 */
            cpFloat w = 0.5 + (rand() % 4) * 0.12;
            cpFloat h = 0.5 + (rand() % 4) * 0.12;
            cpFloat moment = cpMomentForBox(mass, w, h);
            body = cpBodyNew(mass, moment);
            cpBodySetPosition(body, cpv(x, y));
            shape = cpBoxShapeNew(body, w, h, 0.05);
        }

        cpShapeSetFriction(shape, 0.5 + (rand() % 5) * 0.08);
        cpShapeSetElasticity(shape, 0.1 + (rand() % 5) * 0.06);

        cpSpaceAddBody(space, body);
        cpSpaceAddShape(space, shape);

        bodies[i] = body;
        shapes[i] = shape;
    }

    /* 创建一个悬挂摆球（固定在空中的圆） */
    {
        cpFloat mass = 2.0;
        cpFloat r = 0.7;
        cpFloat moment = cpMomentForCircle(mass, 0, r, cpvzero);
        cpBody *pendBody = cpBodyNew(mass, moment);
        cpBodySetPosition(pendBody, cpv(0, 4));
        cpSpaceAddBody(space, pendBody);

        cpShape *pendShape = cpCircleShapeNew(pendBody, r, cpvzero);
        cpShapeSetFriction(pendShape, 0.4);
        cpShapeSetElasticity(pendShape, 0.6);
        cpSpaceAddShape(space, pendShape);

        /* 用枢轴关节将摆球固定在天花板上 */
        cpConstraint *joint = cpPivotJointNew(pendBody, staticBody, cpv(0, 9));
        cpSpaceAddConstraint(space, joint);
    }

    /* ---- 主循环 ---- */
    int running = 1;
    SDL_Event ev;
    Uint32 lastTime = SDL_GetTicks();
    const cpFloat dt = 1.0f / 60.0f;
    float accumulator = 0;

    while (running)
    {
        while (SDL_PollEvent(&ev))
        {
            if (ev.type == SDL_QUIT)
                running = 0;
            if (ev.type == SDL_KEYDOWN)
            {
                if (ev.key.keysym.sym == SDLK_ESCAPE)
                    running = 0;
                /* 按 R 键重置所有物体位置 */
                if (ev.key.keysym.sym == SDLK_r)
                {
                    for (int i = 0; i < NUM_OBJECTS; i++)
                    {
                        cpFloat x = -12 + (rand() % 240) / 10.0;
                        cpFloat y = 2 + i * 0.7;
                        cpBodySetPosition(bodies[i], cpv(x, y));
                        cpBodySetVelocity(bodies[i], cpvzero);
                        cpBodySetAngularVelocity(bodies[i], 0);
                        cpBodySetAngle(bodies[i], 0);
                        cpBodyActivate(bodies[i]);
                    }
                }
            }
        }

        /* 固定时间步长物理更新 */
        Uint32 now = SDL_GetTicks();
        float frameTime = (now - lastTime) / 1000.0f;
        lastTime = now;
        if (frameTime > 0.05f)
            frameTime = 0.05f; /* 防止卡顿时跳跃过大 */

        accumulator += frameTime;
        while (accumulator >= dt)
        {
            cpSpaceStep(space, dt);
            accumulator -= dt;
        }

        /* ---- 渲染 ---- */
        SDL_SetRenderDrawColor(ren, 22, 22, 42, 255);
        SDL_RenderClear(ren);

        /* 绘制地面和墙壁 */
        SDL_SetRenderDrawColor(ren, 70, 70, 90, 255);
        int gx1, gy1, gx2, gy2;

        /* 地面 */
        worldToPixel(cpv(-16, -9.5), &gx1, &gy1);
        worldToPixel(cpv(16, -9.5), &gx2, &gy2);
        for (int t = -3; t <= 3; t++)
            SDL_RenderDrawLine(ren, gx1, gy1 + t, gx2, gy2 + t);

        /* 左墙 */
        worldToPixel(cpv(-16, -9.5), &gx1, &gy1);
        worldToPixel(cpv(-16, 12), &gx2, &gy2);
        for (int t = -3; t <= 3; t++)
            SDL_RenderDrawLine(ren, gx1 + t, gy1, gx2 + t, gy2);

        /* 右墙 */
        worldToPixel(cpv(16, -9.5), &gx1, &gy1);
        worldToPixel(cpv(16, 12), &gx2, &gy2);
        for (int t = -3; t <= 3; t++)
            SDL_RenderDrawLine(ren, gx1 + t, gy1, gx2 + t, gy2);

        /* 斜面 */
        worldToPixel(cpv(-10, -5), &gx1, &gy1);
        worldToPixel(cpv(-2, -2), &gx2, &gy2);
        for (int t = -1; t <= 1; t++)
            SDL_RenderDrawLine(ren, gx1, gy1 + t, gx2, gy2 + t);

        /* 绘制所有动态物理物体 */
        DrawContext dctx = {.ren = ren, .count = 0};
        cpSpaceEachShape(space, drawShape, &dctx);

        /* 顶部提示条 */
        SDL_SetRenderDrawColor(ren, 0, 0, 0, 160);
        SDL_Rect topBar = {0, 0, WINDOW_WIDTH, 28};
        SDL_RenderFillRect(ren, &topBar);

        SDL_RenderPresent(ren);
    }

    /* ---- 清理 ---- */
    for (int i = 0; i < NUM_OBJECTS; i++)
    {
        cpShapeFree(shapes[i]);
        cpBodyFree(bodies[i]);
    }
    cpShapeFree(ramp);
    cpShapeFree(ground);
    cpShapeFree(lwall);
    cpShapeFree(rwall);
    cpSpaceFree(space);

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
