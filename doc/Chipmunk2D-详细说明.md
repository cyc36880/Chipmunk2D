以下是为 slembcke/Chipmunk2D 物理引擎仓库编写的详细使用文档，包含文件说明、调用关系与使用示例。

---

# Chipmunk2D 物理引擎详细使用文档

## 1. 仓库概述

Chipmunk2D 是一个轻量级、快速、可移植的 2D 刚体物理库，使用 C99 编写，基于 MIT 许可证。它专门为 2D 游戏设计，支持多种碰撞形状、丰富的关节类型、碰撞回调等功能。

仓库地址: [https://github.com/slembcke/Chipmunk2D](https://github.com/slembcke/Chipmunk2D)

**特性：**

- 圆形、凸多边形和带斜面线段碰撞几何体
- 多个碰撞几何体可附加到同一个刚体
- 使用包围盒树进行快速宽相碰撞检测
- 基于 Erin Catto 接触持久化算法的快速脉冲求解器
- 支持休眠对象以减少 CPU 负载
- 灵活的碰撞过滤和回调系统
- 支持点查询、射线检测、形状查询
- 丰富的关节类型（销轴、滑动、旋转、齿轮等）

**注意：** 仓库中头文件位于 `include/chipmunk/` 目录，源文件位于 `src/` 目录。本文档主要参考 7.0.3 版本。

## 2. 头文件 (.h) 说明

所有公共头文件位于 `include/chipmunk/` 目录下，按功能模块组织。

| 头文件                   | 描述                                                   |
| ------------------------ | ------------------------------------------------------ |
| `chipmunk.h`             | 主汇总头文件，包含 CP_EXPORT 宏定义、初始化函数等      |
| `chipmunk_types.h`       | 基本类型定义（cpFloat、cpBool 等），可配置浮点精度     |
| `chipmunk_ffi.h`         | FFI 接口类型定义（用于外部语言绑定）                   |
| `chipmunk_private.h`     | 私有 API 头文件，仅内部使用，不推荐用户直接引用        |
| `chipmunk_structs.h`     | 结构体定义（私有），用于预分配内存时的尺寸参考         |
| `chipmunk_unsafe.h`      | 不安全形状操作函数声明                                 |
| `cpVect.h`               | 二维向量 (cpVect) 及相关操作函数                       |
| `cpBB.h`                 | 包围盒 (cpBB) 定义及操作                               |
| `cpTransform.h`          | 2D 变换矩阵定义及操作                                  |
| `cpBody.h`               | 刚体 (cpBody) 创建、属性设置、速度/位置更新等          |
| `cpShape.h`              | 碰撞形状基类 (cpShape)，含点查询、线段查询、过滤类型   |
| `cpPolyShape.h`          | 凸多边形形状 (cpPolyShape)                             |
| `cpArbiter.h`            | 碰撞仲裁 (cpArbiter)，碰撞对信息                       |
| `cpConstraint.h`         | 约束基类 (cpConstraint)                                |
| `cpPinJoint.h`           | 销轴关节：固定两点间距离                               |
| `cpSlideJoint.h`         | 滑动关节：限制两点间最小/最大距离                      |
| `cpPivotJoint.h`         | 枢轴关节：允许两物体相对旋转，保持锚点重合             |
| `cpGrooveJoint.h`        | 槽关节：一点沿线段滑动                                 |
| `cpDampedSpring.h`       | 阻尼弹簧                                               |
| `cpDampedRotarySpring.h` | 旋转阻尼弹簧                                           |
| `cpRotaryLimitJoint.h`   | 旋转限制关节                                           |
| `cpRatchetJoint.h`       | 棘轮关节                                               |
| `cpGearJoint.h`          | 齿轮关节                                               |
| `cpSimpleMotor.h`        | 简单马达关节                                           |
| `cpSpace.h`              | 物理空间 (cpSpace)：模拟容器，管理所有物体、碰撞、回调 |
| `cpSpatialIndex.h`       | 空间索引抽象接口                                       |
| `cpHastySpace.h`         | 多线程空间实现（使用多线程求解器）                     |
| `cpMarch.h`              | 行进正方形算法（用于自动几何生成）                     |
| `cpPolyline.h`           | 多段线简化工具                                         |
| `cpRobust.h`             | 鲁棒性几何计算工具                                     |

## 3. 源文件 (.c) 说明

所有源文件位于 `src/` 目录下。

| 源文件                   | 实现内容                                                     |
| ------------------------ | ------------------------------------------------------------ |
| `chipmunk.c`             | 全局初始化、版本信息、惯量/面积计算等工具函数                |
| `cpSpace.c`              | 物理空间：步进、碰撞检测调度、约束求解、碰撞回调             |
| `cpBody.c`               | 刚体：创建/销毁、质量/转动惯量设置、力/冲量施加、速度/位置更新 |
| `cpShape.c`              | 形状基类：过滤、弹性/摩擦、传感器、质量属性                  |
| `cpPolyShape.c`          | 凸多边形形状实现                                             |
| `cpCollision.c`          | 窄相碰撞检测：GJK/EPA 算法                                   |
| `cpArbiter.c`            | 碰撞仲裁：接触点管理、冲量计算                               |
| `cpConstraint.c`         | 约束基类：最大力、误差偏移、碰撞体设置                       |
| `cpPinJoint.c`           | 销轴关节求解器                                               |
| `cpSlideJoint.c`         | 滑动关节求解器                                               |
| `cpPivotJoint.c`         | 枢轴关节求解器                                               |
| `cpGrooveJoint.c`        | 槽关节求解器                                                 |
| `cpDampedSpring.c`       | 阻尼弹簧求解器                                               |
| `cpDampedRotarySpring.c` | 旋转阻尼弹簧求解器                                           |
| `cpRotaryLimitJoint.c`   | 旋转限制关节求解器                                           |
| `cpRatchetJoint.c`       | 棘轮关节求解器                                               |
| `cpGearJoint.c`          | 齿轮关节求解器                                               |
| `cpSimpleMotor.c`        | 简单马达求解器                                               |
| `cpBBTree.c`             | 动态包围盒树空间索引                                         |
| `cpSpatialIndex.c`       | 空间索引抽象接口                                             |
| `cpSpaceHash.c`          | 空间哈希索引实现                                             |
| `cpSpaceStep.c`          | 空间步进核心逻辑                                             |
| `cpSpaceQuery.c`         | 空间查询（点、线段、形状查询）                               |
| `cpSpaceComponent.c`     | 空间组件管理                                                 |
| `cpSpaceDebug.c`         | 调试绘制支持                                                 |
| `cpHastySpace.c`         | 多线程空间实现                                               |
| `cpArray.c`              | 动态数组工具                                                 |
| `cpHashSet.c`            | 哈希集合工具                                                 |
| `cpSweep1D.c`            | 1D 扫描和修剪                                                |
| `cpMarch.c`              | 行进正方形算法                                               |
| `cpPolyline.c`           | 多段线简化                                                   |
| `cpRobust.c`             | 鲁棒性几何计算                                               |

## 4. 文件间调用关系与编译

### 4.1 依赖层次

```
chipmunk.h (顶层汇总)
├── chipmunk_types.h (基础类型)
├── cpVect.h (向量)
│   └── chipmunk_types.h
├── cpBB.h (包围盒)
│   └── cpVect.h
├── cpTransform.h (变换)
│   └── cpVect.h
├── cpBody.h (刚体)
│   ├── cpVect.h
│   └── cpTransform.h
├── cpShape.h (形状)
│   ├── cpBB.h
│   ├── cpBody.h
│   └── cpTransform.h
├── cpPolyShape.h (多边形形状)
│   └── cpShape.h
├── cpArbiter.h (仲裁器)
│   └── cpBody.h
├── cpConstraint.h (约束)
│   └── cpBody.h
├── cpSpace.h (空间)
│   ├── cpBody.h
│   ├── cpShape.h
│   ├── cpArbiter.h
│   └── cpConstraint.h
├── cpSpatialIndex.h (空间索引)
├── cpMarch.h (行进正方形)
├── cpPolyline.h (多段线)
└── cpRobust.h (鲁棒性几何)
```

### 4.2 编译集成

**CMake 构建：**

```bash
cmake -D CMAKE_BUILD_TYPE=Release .
make
```

**直接包含源文件：**

最简单的集成方式是将 `src/` 中的所有 `.c` 文件添加到项目中，并配置头文件搜索路径包含 `include/` 目录。

> **注意：** 仓库 README 提到 Chipmunk2D 是一个轻量级 C99 实现，除了标准 C 库外没有外部依赖。因此可以直接将源文件包含到项目中编译。

## 5. 核心 API 函数说明

### 5.1 向量与数学函数 (cpVect.h)

```c
// 创建向量
cpVect cpv(cpFloat x, cpFloat y);

// 向量运算
cpVect cpvadd(cpVect v1, cpVect v2);     // 加法
cpVect cpvsub(cpVect v1, cpVect v2);     // 减法
cpVect cpvmult(cpVect v, cpFloat s);     // 标量乘法
cpFloat cpvdot(cpVect v1, cpVect v2);    // 点积
cpFloat cpvcross(cpVect v1, cpVect v2);  // 2D 叉积
cpFloat cpvlength(cpVect v);             // 长度
cpVect cpvnormalize(cpVect v);           // 归一化
cpVect cpvlerp(cpVect v1, cpVect v2, cpFloat t); // 线性插值
```

### 5.2 刚体 (cpBody.h)

**创建/销毁：**

```c
// 创建动态刚体（指定质量和转动惯量）
cpBody* cpBodyNew(cpFloat mass, cpFloat moment);

// 创建静态刚体（无限质量）
cpBody* cpBodyNewStatic();

// 创建运动学刚体（无限质量，不受力影响，可手动设置速度）
cpBody* cpBodyNewKinematic();

// 释放刚体
void cpBodyFree(cpBody *body);
```

**属性设置/获取：**

```c
// 质量和转动惯量
void cpBodySetMass(cpBody *body, cpFloat mass);
cpFloat cpBodyGetMass(const cpBody *body);
void cpBodySetMoment(cpBody *body, cpFloat moment);
cpFloat cpBodyGetMoment(const cpBody *body);

// 位置、角度
cpVect cpBodyGetPosition(const cpBody *body);
void cpBodySetPosition(cpBody *body, cpVect pos);
cpFloat cpBodyGetAngle(const cpBody *body);
void cpBodySetAngle(cpBody *body, cpFloat angle);

// 速度、角速度
cpVect cpBodyGetVelocity(const cpBody *body);
void cpBodySetVelocity(cpBody *body, cpVect velocity);
cpFloat cpBodyGetAngularVelocity(const cpBody *body);
void cpBodySetAngularVelocity(cpBody *body, cpFloat angularVelocity);

// 类型
cpBodyType cpBodyGetType(const cpBody *body);
void cpBodySetType(cpBody *body, cpBodyType type);
// 类型可选：CP_BODY_TYPE_DYNAMIC, CP_BODY_TYPE_KINEMATIC, CP_BODY_TYPE_STATIC
```

**力与冲量：**

```c
// 在世界坐标系下施加力
void cpBodyApplyForceAtWorldPoint(cpBody *body, cpVect force, cpVect point);
// 在局部坐标系下施加力
void cpBodyApplyForceAtLocalPoint(cpBody *body, cpVect force, cpVect point);
// 在世界坐标系下施加冲量
void cpBodyApplyImpulseAtWorldPoint(cpBody *body, cpVect impulse, cpVect point);
// 在局部坐标系下施加冲量
void cpBodyApplyImpulseAtLocalPoint(cpBody *body, cpVect impulse, cpVect point);
// 重置所有力
void cpBodySetForce(cpBody *body, cpVect force);
// 设置扭矩
void cpBodySetTorque(cpBody *body, cpFloat torque);
```

**坐标转换：**

```c
// 局部坐标转世界坐标
cpVect cpBodyLocalToWorld(const cpBody *body, cpVect point);
// 世界坐标转局部坐标
cpVect cpBodyWorldToLocal(const cpBody *body, cpVect point);
```

**惯量计算辅助函数（定义在 chipmunk.c 中）**：

```c
// 圆的转动惯量
cpFloat cpMomentForCircle(cpFloat m, cpFloat r1, cpFloat r2, cpVect offset);
// 线段的转动惯量
cpFloat cpMomentForSegment(cpFloat m, cpVect a, cpVect b, cpFloat r);
// 多边形的转动惯量
cpFloat cpMomentForPoly(cpFloat m, int count, const cpVect *verts, cpVect offset, cpFloat r);
// 盒子（矩形）的转动惯量
cpFloat cpMomentForBox(cpFloat m, cpFloat width, cpFloat height);
```

### 5.3 形状 (cpShape.h / cpPolyShape.h)

**形状创建：**

```c
// 创建圆形形状
cpShape* cpCircleShapeNew(cpBody *body, cpFloat radius, cpVect offset);
// 创建线段形状
cpShape* cpSegmentShapeNew(cpBody *body, cpVect a, cpVect b, cpFloat radius);
// 创建多边形（盒子）形状
cpShape* cpBoxShapeNew(cpBody *body, cpFloat width, cpFloat height, cpFloat radius);
// 创建凸多边形形状
cpShape* cpPolyShapeNew(cpBody *body, int count, const cpVect *verts, cpTransform transform, cpFloat radius);
// 从原始多边形创建（不需要变换和半径）
cpShape* cpPolyShapeNewRaw(cpBody *body, int count, const cpVect *verts, cpFloat radius);
```

**形状属性：**

```c
// 弹性系数 (e) 和摩擦系数 (u)
void cpShapeSetFriction(cpShape *shape, cpFloat friction);
cpFloat cpShapeGetFriction(const cpShape *shape);
void cpShapeSetElasticity(cpShape *shape, cpFloat elasticity);
cpFloat cpShapeGetElasticity(const cpShape *shape);

// 碰撞过滤
void cpShapeSetFilter(cpShape *shape, cpShapeFilter filter);
cpShapeFilter cpShapeGetFilter(const cpShape *shape);

// 传感器（不产生物理碰撞，仅触发回调）
void cpShapeSetSensor(cpShape *shape, cpBool sensor);
cpBool cpShapeIsSensor(const cpShape *shape);

// 碰撞类型（用于碰撞回调匹配）
void cpShapeSetCollisionType(cpShape *shape, cpCollisionType type);
cpCollisionType cpShapeGetCollisionType(const cpShape *shape);

// 所属刚体
cpBody* cpShapeGetBody(const cpShape *shape);

// 用户数据指针
void cpShapeSetUserData(cpShape *shape, cpDataPointer data);
cpDataPointer cpShapeGetUserData(const cpShape *shape);

// 表面积（用于浮力等计算）
cpFloat cpShapeGetArea(const cpShape *shape);
```

**形状过滤器 (cpShapeFilter)：**

```c
typedef struct cpShapeFilter {
    cpGroup group;       // 碰撞组（同组非零值不碰撞）
    cpBitmask categories; // 所属类别位掩码
    cpBitmask mask;       // 碰撞掩码
} cpShapeFilter;

// 预定义过滤器
CP_SHAPE_FILTER_ALL   // 与所有物体碰撞
CP_SHAPE_FILTER_NONE  // 不与任何物体碰撞

// 创建自定义过滤器
cpShapeFilter cpShapeFilterNew(cpGroup group, cpBitmask categories, cpBitmask mask);
```

### 5.4 约束 (cpConstraint.h)

**通用约束属性：**

```c
// 最大力限制
void cpConstraintSetMaxForce(cpConstraint *constraint, cpFloat maxForce);
cpFloat cpConstraintGetMaxForce(const cpConstraint *constraint);

// 误差修正率（默认 pow(1.0 - 0.1, 60.0) 即每1/60秒修正10%误差）
void cpConstraintSetErrorBias(cpConstraint *constraint, cpFloat errorBias);
cpFloat cpConstraintGetErrorBias(const cpConstraint *constraint);

// 最大修正速度
void cpConstraintSetMaxBias(cpConstraint *constraint, cpFloat maxBias);
cpFloat cpConstraintGetMaxBias(const cpConstraint *constraint);

// 是否允许连接的两物体碰撞
void cpConstraintSetCollideBodies(cpConstraint *constraint, cpBool collideBodies);
cpBool cpConstraintGetCollideBodies(const cpConstraint *constraint);

// 获取连接的两刚体
cpBody* cpConstraintGetBodyA(const cpConstraint *constraint);
cpBody* cpConstraintGetBodyB(const cpConstraint *constraint);
```

**各类关节创建函数：**

```c
// 销轴关节：固定两点距离
cpConstraint* cpPinJointNew(cpBody *a, cpBody *b, cpVect anchorA, cpVect anchorB);

// 滑动关节：限制两点距离范围
cpConstraint* cpSlideJointNew(cpBody *a, cpBody *b, cpVect anchorA, cpVect anchorB, cpFloat min, cpFloat max);

// 枢轴关节：两点重合，允许旋转
cpConstraint* cpPivotJointNew(cpBody *a, cpBody *b, cpVect anchorA, cpVect anchorB);
cpConstraint* cpPivotJointNew2(cpBody *a, cpBody *b, cpVect pivot); // 世界空间锚点

// 槽关节：一个点沿线段滑动
cpConstraint* cpGrooveJointNew(cpBody *a, cpBody *b, cpVect grooveA, cpVect grooveB, cpVect anchorB);

// 阻尼弹簧
cpConstraint* cpDampedSpringNew(cpBody *a, cpBody *b, cpVect anchorA, cpVect anchorB, cpFloat restLength, cpFloat stiffness, cpFloat damping);

// 旋转阻尼弹簧
cpConstraint* cpDampedRotarySpringNew(cpBody *a, cpBody *b, cpFloat restAngle, cpFloat stiffness, cpFloat damping);

// 旋转限制关节
cpConstraint* cpRotaryLimitJointNew(cpBody *a, cpBody *b, cpFloat min, cpFloat max);

// 棘轮关节
cpConstraint* cpRatchetJointNew(cpBody *a, cpBody *b, cpFloat phase, cpFloat ratchet);

// 齿轮关节
cpConstraint* cpGearJointNew(cpBody *a, cpBody *b, cpFloat phase, cpFloat ratio);

// 简单马达（恒定相对角速度）
cpConstraint* cpSimpleMotorNew(cpBody *a, cpBody *b, cpFloat rate);
```

### 5.5 物理空间 (cpSpace.h)

**创建/销毁/步进：**

```c
// 创建空间
cpSpace* cpSpaceNew();

// 释放空间及所有内容
void cpSpaceFree(cpSpace *space);

// 按固定时间步进模拟
void cpSpaceStep(cpSpace *space, cpFloat dt);
```

**添加/移除对象：**

```c
// 刚体
cpSpaceAddBody(cpSpace *space, cpBody *body);
cpSpaceRemoveBody(cpSpace *space, cpBody *body);

// 形状
cpSpaceAddShape(cpSpace *space, cpShape *shape);
cpSpaceRemoveShape(cpSpace *space, cpShape *shape);

// 约束
cpSpaceAddConstraint(cpSpace *space, cpConstraint *constraint);
cpSpaceRemoveConstraint(cpSpace *space, cpConstraint *constraint);
```

**空间属性：**

```c
// 重力
void cpSpaceSetGravity(cpSpace *space, cpVect gravity);
cpVect cpSpaceGetGravity(const cpSpace *space);

// 阻尼
void cpSpaceSetDamping(cpSpace *space, cpFloat damping);
cpFloat cpSpaceGetDamping(const cpSpace *space);

// 迭代次数（默认10）
void cpSpaceSetIterations(cpSpace *space, int iterations);

// 静态体（空间的内置静态刚体引用）
cpBody* cpSpaceGetStaticBody(const cpSpace *space);

// 碰撞持久性（步数，默认3）
void cpSpaceSetCollisionPersistence(cpSpace *space, cpTimestamp value);
cpTimestamp cpSpaceGetCollisionPersistence(const cpSpace *space);
```

**碰撞回调设置：**

```c
// 添加碰撞处理器
cpCollisionHandler* cpSpaceAddCollisionHandler(cpSpace *space, cpCollisionType typeA, cpCollisionType typeB);

// 使用通配符匹配所有碰撞类型
cpCollisionHandler* cpSpaceAddWildcardHandler(cpSpace *space, cpCollisionType type);

// 设置默认碰撞处理器
cpCollisionHandler* cpSpaceAddDefaultCollisionHandler(cpSpace *space);
```

**碰撞处理器结构体：**

```c
struct cpCollisionHandler {
    const cpCollisionType typeA;
    const cpCollisionType typeB;
    cpCollisionBeginFunc beginFunc;           // 开始碰撞回调
    cpCollisionPreSolveFunc preSolveFunc;     // 求解前回调
    cpCollisionPostSolveFunc postSolveFunc;   // 求解后回调
    cpCollisionSeparateFunc separateFunc;     // 分离回调
    cpDataPointer userData;                   // 用户数据指针
};

// 回调函数签名：
typedef cpBool (*cpCollisionBeginFunc)(cpArbiter *arb, cpSpace *space, cpDataPointer userData);
typedef cpBool (*cpCollisionPreSolveFunc)(cpArbiter *arb, cpSpace *space, cpDataPointer userData);
typedef void (*cpCollisionPostSolveFunc)(cpArbiter *arb, cpSpace *space, cpDataPointer userData);
typedef void (*cpCollisionSeparateFunc)(cpArbiter *arb, cpSpace *space, cpDataPointer userData);
```

**空间查询：**

```c
// 点查询：查找包围某点的形状
cpShape* cpSpacePointQueryNearest(cpSpace *space, cpVect point, cpFloat maxDistance, cpShapeFilter filter, cpPointQueryInfo *out);
// 线段查询（射线检测）
cpShape* cpSpaceSegmentQueryFirst(cpSpace *space, cpVect start, cpVect end, cpFloat radius, cpShapeFilter filter, cpSegmentQueryInfo *out);
// 包围盒查询
void cpSpaceBBQuery(cpSpace *space, cpBB bb, cpShapeFilter filter, cpSpaceBBQueryFunc func, void *data);
// 形状查询
cpBool cpSpaceShapeQuery(cpSpace *space, cpShape *shape, cpSpaceShapeQueryFunc func, void *data);
```

### 5.6 碰撞仲裁 (cpArbiter.h)

```c
// 获取碰撞中的两形状
cpShape* cpArbiterGetShapeA(const cpArbiter *arb);
cpShape* cpArbiterGetShapeB(const cpArbiter *arb);

// 获取碰撞中的两刚体
cpBody* cpArbiterGetBodyA(const cpArbiter *arb);
cpBody* cpArbiterGetBodyB(const cpArbiter *arb);

// 获取接触点数量
int cpArbiterGetCount(const cpArbiter *arb);

// 获取法向量
cpVect cpArbiterGetNormal(const cpArbiter *arb);

// 是否为第一次碰撞（可用于触发音效等）
cpBool cpArbiterIsFirstContact(const cpArbiter *arb);

// 判断碰撞是否已清除
cpBool cpArbiterIsRemoval(const cpArbiter *arb);

// 忽略此碰撞（在 begin 或 preSolve 回调中调用）
void cpArbiterIgnore(cpArbiter *arb);
```

## 6. 示例程序

### 6.1 最简示例：球落在线段上

```c
#include "chipmunk/chipmunk.h"
#include <stdio.h>

int main(void) {
    // 1. 创建物理空间
    cpSpace *space = cpSpaceNew();
    cpSpaceSetGravity(space, cpv(0, -100));  // 重力向下

    // 2. 创建地面（静态线段）
    cpBody *groundBody = cpSpaceGetStaticBody(space);
    cpShape *ground = cpSegmentShapeNew(groundBody, cpv(-20, 5), cpv(20, -5), 0);
    cpShapeSetFriction(ground, 1.0);
    cpSpaceAddShape(space, ground);

    // 3. 创建球体（动态圆）
    cpFloat radius = 5.0;
    cpFloat mass = 1.0;
    cpFloat moment = cpMomentForCircle(mass, 0, radius, cpvzero);

    cpBody *ballBody = cpBodyNew(mass, moment);
    cpBodySetPosition(ballBody, cpv(0, 15));
    cpSpaceAddBody(space, ballBody);

    cpShape *ballShape = cpCircleShapeNew(ballBody, radius, cpvzero);
    cpShapeSetFriction(ballShape, 0.7);
    cpShapeSetElasticity(ballShape, 0.5);
    cpSpaceAddShape(space, ballShape);

    // 4. 模拟 60 步，打印位置
    cpFloat dt = 1.0 / 60.0;
    for (int i = 0; i < 60; i++) {
        cpSpaceStep(space, dt);
        cpVect pos = cpBodyGetPosition(ballBody);
        printf("Step %d: (%.2f, %.2f)\n", i, pos.x, pos.y);
    }

    // 5. 清理
    cpShapeFree(ballShape);
    cpBodyFree(ballBody);
    cpShapeFree(ground);
    cpSpaceFree(space);

    return 0;
}
```

### 6.2 碰撞回调示例：检测碰撞并播放音效

```c
#include "chipmunk/chipmunk.h"
#include <stdio.h>

// 定义碰撞类型
#define COLLISION_TYPE_BALL 1
#define COLLISION_TYPE_WALL 2

// 碰撞开始回调
static cpBool onCollisionBegin(cpArbiter *arb, cpSpace *space, void *userData) {
    cpShape *a = cpArbiterGetShapeA(arb);
    cpShape *b = cpArbiterGetShapeB(arb);

    printf("碰撞开始！形状类型: %d 与 %d\n",
           cpShapeGetCollisionType(a),
           cpShapeGetCollisionType(b));

    // 如果是第一次接触，触发音效等
    if (cpArbiterIsFirstContact(arb)) {
        printf("  -> 首次接触\n");
    }

    return cpTrue; // 返回 cpTrue 允许碰撞，返回 cpFalse 忽略
}

// 分离回调
static void onCollisionSeparate(cpArbiter *arb, cpSpace *space, void *userData) {
    printf("碰撞分离！\n");
}

// 求解后回调（获取碰撞冲量）
static void onCollisionPostSolve(cpArbiter *arb, cpSpace *space, void *userData) {
    cpVect impulse = cpArbiterTotalImpulse(arb);
    printf("碰撞冲量: (%.2f, %.2f)\n", impulse.x, impulse.y);
}

int main(void) {
    cpSpace *space = cpSpaceNew();
    cpSpaceSetGravity(space, cpv(0, -100));

    // 设置碰撞回调
    cpCollisionHandler *handler = cpSpaceAddCollisionHandler(space,
        COLLISION_TYPE_BALL, COLLISION_TYPE_WALL);
    handler->beginFunc = onCollisionBegin;
    handler->separateFunc = onCollisionSeparate;
    handler->postSolveFunc = onCollisionPostSolve;

    // ... 创建物体和形状，设置碰撞类型 ...

    // 设置形状碰撞类型
    cpShapeSetCollisionType(wallShape, COLLISION_TYPE_WALL);
    cpShapeSetCollisionType(ballShape, COLLISION_TYPE_BALL);

    // 模拟循环
    for (int i = 0; i < 100; i++) {
        cpSpaceStep(space, 1.0 / 60.0);
    }

    cpSpaceFree(space);
    return 0;
}
```

### 6.3 关节示例：钟摆

```c
#include "chipmunk/chipmunk.h"

int main(void) {
    cpSpace *space = cpSpaceNew();
    cpSpaceSetGravity(space, cpv(0, -100));

    // 创建固定锚点（静态体）
    cpBody *staticBody = cpSpaceGetStaticBody(space);

    // 创建摆锤
    cpFloat mass = 1.0;
    cpFloat radius = 10.0;
    cpFloat moment = cpMomentForCircle(mass, 0, radius, cpvzero);
    cpBody *bobBody = cpBodyNew(mass, moment);
    cpBodySetPosition(bobBody, cpv(50, 50));
    cpSpaceAddBody(space, bobBody);

    cpShape *bobShape = cpCircleShapeNew(bobBody, radius, cpvzero);
    cpShapeSetFriction(bobShape, 0.7);
    cpSpaceAddShape(space, bobShape);

    // 创建枢轴关节（钟摆的支点）
    cpConstraint *pivotJoint = cpPivotJointNew2(
        staticBody, bobBody, cpv(50, 100));
    cpSpaceAddConstraint(space, pivotJoint);

    // 模拟
    for (int i = 0; i < 200; i++) {
        cpSpaceStep(space, 1.0 / 60.0);
        cpVect pos = cpBodyGetPosition(bobBody);
        printf("Step %d: (%.2f, %.2f)\n", i, pos.x, pos.y);
    }

    // 清理
    cpConstraintFree(pivotJoint);
    cpShapeFree(bobShape);
    cpBodyFree(bobBody);
    cpSpaceFree(space);

    return 0;
}
```

### 6.4 汽车模拟示例

```c
#include "chipmunk/chipmunk.h"

int main(void) {
    cpSpace *space = cpSpaceNew();
    cpSpaceSetGravity(space, cpv(0, -500));

    // 创建地面
    cpShape *ground = cpSegmentShapeNew(
        cpSpaceGetStaticBody(space), cpv(-200, 0), cpv(200, 0), 0);
    cpShapeSetFriction(ground, 1.0);
    cpSpaceAddShape(space, ground);

    // 创建车身
    cpFloat mass = 10.0;
    cpFloat width = 60, height = 30;
    cpFloat moment = cpMomentForBox(mass, width, height);
    cpBody *chassisBody = cpBodyNew(mass, moment);
    cpBodySetPosition(chassisBody, cpv(0, 50));
    cpSpaceAddBody(space, chassisBody);

    cpShape *chassisShape = cpBoxShapeNew(chassisBody, width, height, 0);
    cpShapeSetFriction(chassisShape, 0.7);
    cpSpaceAddShape(space, chassisShape);

    // 创建车轮
    cpFloat wheelMass = 2.0;
    cpFloat wheelRadius = 15.0;
    cpFloat wheelMoment = cpMomentForCircle(wheelMass, 0, wheelRadius, cpvzero);

    cpBody *rearWheel = cpBodyNew(wheelMass, wheelMoment);
    cpBodySetPosition(rearWheel, cpv(-25, 25));
    cpSpaceAddBody(space, rearWheel);
    cpShape *rearWheelShape = cpCircleShapeNew(rearWheel, wheelRadius, cpvzero);
    cpShapeSetFriction(rearWheelShape, 1.0);
    cpSpaceAddShape(space, rearWheelShape);

    cpBody *frontWheel = cpBodyNew(wheelMass, wheelMoment);
    cpBodySetPosition(frontWheel, cpv(25, 25));
    cpSpaceAddBody(space, frontWheel);
    cpShape *frontWheelShape = cpCircleShapeNew(frontWheel, wheelRadius, cpvzero);
    cpShapeSetFriction(frontWheelShape, 1.0);
    cpSpaceAddShape(space, frontWheelShape);

    // 用销轴连接车轮到底盘
    cpConstraint *rearPin = cpPinJointNew(chassisBody, rearWheel,
        cpv(-25, -15), cpvzero);
    cpSpaceAddConstraint(space, rearPin);
    cpConstraint *frontPin = cpPinJointNew(chassisBody, frontWheel,
        cpv(25, -15), cpvzero);
    cpSpaceAddConstraint(space, frontPin);

    // 添加槽关节限制底盘悬挂行程
    cpConstraint *rearSuspension = cpGrooveJointNew(
        chassisBody, rearWheel, cpv(-25, -15), cpv(-25, -30), cpvzero);
    cpSpaceAddConstraint(space, rearSuspension);

    // 添加简单马达驱动后轮
    cpConstraint *motor = cpSimpleMotorNew(chassisBody, rearWheel, 10.0);
    cpSpaceAddConstraint(space, motor);

    // 模拟
    for (int i = 0; i < 500; i++) {
        cpSpaceStep(space, 1.0 / 60.0);
    }

    cpSpaceFree(space);
    return 0;
}
```

### 6.5 Demo 程序说明

仓库的 `demo/` 目录包含了多个可直接运行的示例程序：

| 文件                       | 说明                                                  |
| -------------------------- | ----------------------------------------------------- |
| `ChipmunkDemo.c / .h`      | Demo 框架，提供空间创建、调试绘制、鼠标拖拽等基础设施 |
| `ChipmunkDebugDraw.c / .h` | 调试绘制支持                                          |
| `Bench.c`                  | 性能基准测试                                          |
| `Buoyancy.c`               | 浮力模拟示例                                          |
| `Chains.c`                 | 链条示例                                              |

**编译 Demo：**

```bash
cd demo
cmake .
make
./chipmunk_demos
```

## 7. 文件间调用关系总结

1. **初始化流程**：`chipmunk.c` 提供 `cpInitChipmunk()` 等全局初始化函数，以及质量/惯量计算工具函数。

2. **空间管理**：`cpSpace.c` 包含 `cpSpaceNew()`、`cpSpaceStep()` 等核心函数，在步进时调用 `cpSpaceStep.c` 进行碰撞检测、约束求解和位置更新。

3. **碰撞检测流程**：
   - 宽相：`cpSpace.c` 调用空间索引（`cpBBTree.c` 或 `cpSpaceHash.c`）查找潜在碰撞对
   - 窄相：`cpCollision.c` 实现 GJK/EPA 算法进行精确碰撞检测
   - 接触管理：`cpArbiter.c` 管理碰撞对信息，触发碰撞回调

4. **约束求解**：各关节 `.c` 文件实现具体求解逻辑，如 `cpPinJoint.c` 实现距离约束的求解。

5. **刚体更新**：`cpBody.c` 的 `cpBodyUpdateVelocity()` 和 `cpBodyUpdatePosition()` 在步进时被调用进行积分。

## 总结

Chipmunk2D 提供了完整的 2D 物理模拟功能，API 设计清晰，层次结构合理。通过理解空间(cpSpace)→刚体(cpBody)+形状(cpShape)+约束(cpConstraint)+碰撞系统(cpSpace回调+cpArbiter)的架构，可以灵活构建各种物理场景。建议开发时参考仓库自带的 demo 示例获取更多使用灵感。