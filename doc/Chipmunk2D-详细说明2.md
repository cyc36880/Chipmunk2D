# Chipmunk2D 物理引擎使用文档

> 版本: 7.0.3
> 仓库: https://github.com/slembcke/Chipmunk2D (已迁移至 Codeberg)

## 目录

1. [仓库概述](#1-仓库概述)
2. [目录结构](#2-目录结构)
3. [头文件说明](#3-头文件说明)
4. [源文件说明](#4-源文件说明)
5. [文件调用关系](#5-文件调用关系)
6. [核心数据结构](#6-核心数据结构)
7. [API详细说明](#7-api详细说明)
8. [示例程序](#8-示例程序)
9. [碰撞检测流程](#9-碰撞检测流程)

---

## 1. 仓库概述

Chipmunk2D 是一个轻量级、高效的 2D 刚体物理引擎，使用 C 语言编写，采用 MIT 许可证。

### 核心特性

- **碰撞几何体**: 圆形、凸多边形、线段（带圆角）
- **多形状附着**: 一个刚体可附着多个碰撞形状
- **快速碰撞检测**: AABB 树、空间哈希等多种Broad-phase算法
- **持久接触**: 使用Arbiter缓存接触点，减少重复计算
- **对象休眠**: 不活动对象进入休眠状态，节省CPU资源
- **灵活的碰撞过滤**: 支持层级、组、回调机制
- **空间查询**: 射线检测、点查询、边界框查询
- **丰富的关节类型**: 车辆、关节约束、弹簧等
- **无外部依赖**: 纯C99实现

---

## 2. 目录结构

```
Chipmunk2D/
├── include/chipmunk/    # 公共头文件
│   ├── chipmunk.h              # 主头文件，包含版本和工具函数
│   ├── chipmunk_types.h        # 基本类型定义
│   ├── chipmunk_structs.h      # 数据结构定义
│   ├── chipmunk_private.h      # 私有API（内部使用）
│   ├── chipmunk_ffi.h          # FFI导出接口
│   ├── chipmunk_unsafe.h       # 不安全形状修改API
│   ├── cpVect.h                # 2D向量数学库
│   ├── cpBB.h                  # 轴对齐边界框
│   ├── cpTransform.h           # 2D仿射变换
│   ├── cpSpatialIndex.h        # 空间索引接口
│   ├── cpArbiter.h             # 碰撞仲裁器
│   ├── cpBody.h                # 刚体
│   ├── cpShape.h               # 基础形状（圆形、线段）
│   ├── cpPolyShape.h           # 多边形形状
│   ├── cpConstraint.h          # 约束基类和所有关节头文件
│   ├── cpPinJoint.h            # 销关节
│   ├── cpSlideJoint.h          # 滑动关节
│   ├── cpPivotJoint.h          # 枢轴关节
│   ├── cpGrooveJoint.h          # 槽关节
│   ├── cpDampedSpring.h         # 阻尼弹簧
│   ├── cpDampedRotarySpring.h   # 阻尼旋转弹簧
│   ├── cpRotaryLimitJoint.h     # 旋转限制关节
│   ├── cpRatchetJoint.h         # 棘轮关节
│   ├── cpGearJoint.h            # 齿轮关节
│   ├── cpSimpleMotor.h          # 简单电机
│   ├── cpPolyline.h             # 折线
│   ├── cpMarch.h                #  marching squares算法
│   ├── cpHastySpace.h           # 多线程优化空间
│   └── cpRobust.h               # 严格浮点运算
│
├── src/                    # 源代码实现
│   ├── chipmunk.c              # 主入口，版本，质量/面积计算
│   ├── cpSpace.c                # 空间创建、销毁、对象管理
│   ├── cpSpaceStep.c            # 时间步进、碰撞检测循环
│   ├── cpSpaceComponent.c       # 休眠体组件检测
│   ├── cpSpaceQuery.c           # 空间查询
│   ├── cpSpaceDebug.c           # 调试绘制
│   ├── cpBody.c                 # 刚体创建、质量属性
│   ├── cpShape.c                # 基础形状、圆形、线段
│   ├── cpPolyShape.c            # 多边形
│   ├── cpArbiter.c              # 仲裁器管理
│   ├── cpCollision.c            # GJK/EPA 碰撞检测
│   ├── cpConstraint.c           # 约束基础功能
│   ├── cpPinJoint.c             # 销关节实现
│   ├── cpSlideJoint.c           # 滑动关节实现
│   ├── cpPivotJoint.c           # 枢轴关节实现
│   ├── cpGrooveJoint.c          # 槽关节实现
│   ├── cpDampedSpring.c         # 阻尼弹簧实现
│   ├── cpDampedRotarySpring.c   # 阻尼旋转弹簧实现
│   ├── cpRotaryLimitJoint.c     # 旋转限制关节实现
│   ├── cpRatchetJoint.c         # 棘轮关节实现
│   ├── cpGearJoint.c            # 齿轮关节实现
│   ├── cpSimpleMotor.c          # 简单电机实现
│   ├── cpSpatialIndex.c         # 空间索引基础
│   ├── cpBBTree.c               # AABB树实现
│   ├── cpSpaceHash.c            # 空间哈希实现
│   ├── cpSweep1D.c              # 单轴扫描
│   ├── cpArray.c                # 动态数组
│   ├── cpHashSet.c              # 哈希集合
│   ├── cpPolyline.c             # 折线处理
│   ├── cpMarch.c                # Marching squares实现
│   ├── cpHastySpace.c           # NEON优化
│   └── cpRobust.c               # 严格浮点函数
│
├── demo/                   # 示例程序
│   ├── ChipmunkDemo.c           # 主demo框架
│   ├── ChipmunkDemo.h            # Demo接口定义
│   ├── Joints.c                  # 关节示例
│   ├── PyramidStack.c            # 金字塔堆叠
│   ├── Query.c                   # 空间查询示例
│   ├── LogoSmash.c               # Logo碰撞
│   └── ...                       # 更多示例
│
└── doc/                    # 文档
```

---

## 3. 头文件说明

### 3.1 chipmunk_types.h - 基本类型定义

```c
typedef double cpFloat;              // 浮点数类型（默认double）
typedef uintptr_t cpHashValue;       // 哈希值
typedef unsigned char cpBool;        // 布尔类型
typedef void * cpDataPointer;        // 用户数据指针
typedef uintptr_t cpCollisionType;   // 碰撞类型标识
typedef uintptr_t cpGroup;           // 碰撞组
typedef unsigned int cpBitmask;       // 分类位掩码
typedef unsigned int cpTimestamp;     // 时间戳

// 2D向量
typedef struct cpVect { cpFloat x, y; } cpVect;

// 2D仿射变换矩阵
typedef struct cpTransform { cpFloat a, b, c, d, tx, ty; } cpTransform;

// 2x2矩阵
typedef struct cpMat2x2 { cpFloat a, b, c, d; } cpMat2x2;
```

### 3.2 cpVect.h - 2D向量数学库

所有函数均为 `static inline` 以获得最佳性能。

| 函数 | 说明 |
|------|------|
| `cpv(x, y)` | 构造向量 |
| `cpvadd(v1, v2)` | 向量加法 |
| `cpvsub(v1, v2)` | 向量减法 |
| `cpvneg(v)` | 向量取反 |
| `cpvmult(v, s)` | 标量乘法 |
| `cpvdot(v1, v2)` | 点积 |
| `cpvcross(v1, v2)` | 2D叉积（返回标量） |
| `cpvperp(v)` | 法向量（逆时针90度） |
| `cpvrperp(v)` | 反向法向量（顺时针90度） |
| `cpvproject(v1, v2)` | 向量投影 |
| `cpvforangle(a)` | 角度转单位向量 |
| `cpvtoangle(v)` | 向量转角度 |
| `cpvrotate(v1, v2)` | 复数旋转 |
| `cpvunrotate(v1, v2)` | 逆旋转 |
| `cpvlengthsq(v)` | 长度平方 |
| `cpvlength(v)` | 长度 |
| `cpvlerp(v1, v2, t)` | 线性插值 |
| `cpvnormalize(v)` | 单位向量 |
| `cpvslerp(v1, v2, t)` | 球面线性插值 |
| `cpvclamp(v, len)` | 限制长度 |
| `cpvdist(v1, v2)` | 距离 |
| `cpvdistsq(v1, v2)` | 距离平方 |
| `cpvnear(v1, v2, dist)` | 判断是否接近 |

### 3.3 cpBB.h - 轴对齐边界框

```c
typedef struct cpBB {
    cpFloat l, b, r, t;  // left, bottom, right, top
} cpBB;
```

### 3.4 cpTransform.h - 2D仿射变换

```c
static const cpTransform cpTransformIdentity;  // 单位矩阵

// 变换操作
cpTransformNew(a, b, c, d, tx, ty)      // 创建变换
cpTransformInverse(t)                    // 求逆
cpTransformMult(t1, t2)                 // 组合变换
cpTransformPoint(t, p)                  // 变换点（含平移）
cpTransformVect(t, v)                    // 变换向量（无平移）
cpTransformRigid(translate, radians)     // 刚体变换
```

### 3.5 cpBody.h - 刚体

**刚体类型:**
- `CP_BODY_TYPE_DYNAMIC` - 动态体，受力影响
- `CP_BODY_TYPE_KINEMATIC` - 运动学体，无质量，用户控制速度
- `CP_BODY_TYPE_STATIC` - 静态体，无质量，永不动

**主要函数:**

创建/销毁:
```c
cpBodyAlloc()                           // 分配内存
cpBodyInit(body, mass, moment)          // 初始化
cpBodyNew(mass, moment)                 // 创建动态体
cpBodyNewKinematic()                    // 创建运动学体
cpBodyNewStatic()                       // 创建静态体
cpBodyDestroy(body)                     // 销毁
cpBodyFree(body)                        // 释放内存
```

状态控制:
```c
cpBodyActivate(body)                     // 唤醒休眠体
cpBodyActivateStatic(body, filter)       // 唤醒接触静态体的动态体
cpBodySleep(body)                       // 使体入睡
cpBodySleepWithGroup(body, group)        // 与组一起入睡
cpBodyIsSleeping(body) -> cpBool        // 检查是否休眠
```

属性访问:
```c
cpBodyGetMass(cpBody *body) -> cpFloat
cpBodySetMass(cpBody *body, cpFloat mass)
cpBodyGetMoment(cpBody *body) -> cpFloat
cpBodySetMoment(cpBody *body, cpFloat moment)
cpBodyGetPosition(cpBody *body) -> cpVect
cpBodySetPosition(cpBody *body, cpVect pos)
cpBodyGetAngle(cpBody *body) -> cpFloat
cpBodySetAngle(cpBody *body, cpFloat angle)
cpBodyGetVelocity(cpBody *body) -> cpVect
cpBodySetVelocity(cpBody *body, cpVect vel)
cpBodyGetAngularVelocity(cpBody *body) -> cpFloat
cpBodySetAngularVelocity(cpBody *body, cpFloat angVel)
cpBodyGetForce(cpBody *body) -> cpVect
cpBodySetForce(cpBody *body, cpVect force)
cpBodyGetTorque(cpBody *body) -> cpFloat
cpBodySetTorque(cpBody *body, cpFloat torque)
cpBodyGetRotation(body) -> cpVect        // 获取旋转矩阵的x基向量
```

坐标转换:
```c
cpBodyLocalToWorld(body, point) -> cpVect    // 局部坐标转世界坐标
cpBodyWorldToLocal(body, point) -> cpVect    // 世界坐标转局部坐标
```

力和冲量:
```c
cpBodyApplyForceAtWorldPoint(body, force, point)     // 在世界点施加力
cpBodyApplyForceAtLocalPoint(body, force, point)      // 在局部点施加力
cpBodyApplyImpulseAtWorldPoint(body, impulse, point)  // 在世界点施加冲量
cpBodyApplyImpulseAtLocalPoint(body, impulse, point)  // 在局部点施加冲量
```

速度查询:
```c
cpBodyGetVelocityAtWorldPoint(body, point) -> cpVect
cpBodyGetVelocityAtLocalPoint(body, point) -> cpVect
cpBodyKineticEnergy(body) -> cpFloat
```

迭代器:
```c
cpBodyEachShape(body, func, data)           // 遍历所有形状
cpBodyEachConstraint(body, func, data)       // 遍历所有约束
cpBodyEachArbiter(body, func, data)          // 遍历所有仲裁器
```

### 3.6 cpShape.h - 基础形状

**形状类型:**
- `CP_CIRCLE_SHAPE` - 圆形
- `CP_SEGMENT_SHAPE` - 线段
- `CP_POLY_SHAPE` - 多边形

**通用函数:**
```c
cpShapeDestroy(shape)                    // 销毁形状
cpShapeFree(shape)                       // 释放形状
cpShapeCacheBB(shape) -> cpBB            // 缓存边界框
cpShapeUpdate(shape, transform) -> cpBB  // 更新边界框
cpShapePointQuery(shape, p, info) -> cpFloat  // 点查询
cpShapeSegmentQuery(shape, a, b, radius, info) -> cpBool  // 线段查询
cpShapesCollide(a, b) -> cpContactPointSet  // 碰撞检测
```

**属性访问:**
```c
cpShapeGetSpace(shape) -> cpSpace*
cpShapeGetBody(shape) -> cpBody*
cpShapeSetBody(shape, body)
cpShapeGetMass(shape) -> cpFloat
cpShapeSetMass(shape, mass)
cpShapeGetDensity(shape) -> cpFloat
cpShapeSetDensity(shape, density)
cpShapeGetMoment(shape) -> cpFloat
cpShapeGetArea(shape) -> cpFloat
cpShapeGetCenterOfGravity(shape) -> cpVect
cpShapeGetBB(shape) -> cpBB
cpShapeGetSensor(shape) -> cpBool
cpShapeSetSensor(shape, sensor)
cpShapeGetElasticity(shape) -> cpFloat
cpShapeSetElasticity(shape, e)
cpShapeGetFriction(shape) -> cpFloat
cpShapeSetFriction(shape, u)
cpShapeGetSurfaceVelocity(shape) -> cpVect
cpShapeSetSurfaceVelocity(shape, v)
cpShapeGetUserData(shape) -> void*
cpShapeSetUserData(shape, data)
cpShapeGetCollisionType(shape) -> cpCollisionType
cpShapeSetCollisionType(shape, type)
cpShapeGetFilter(shape) -> cpShapeFilter
cpShapeSetFilter(shape, filter)
```

**圆形形状:**
```c
cpCircleShapeAlloc()                              // 分配
cpCircleShapeInit(circle, body, radius, offset)   // 初始化
cpCircleShapeNew(body, radius, offset)             // 创建
cpCircleShapeGetOffset(shape) -> cpVect           // 获取圆心偏移
cpCircleShapeGetRadius(shape) -> cpFloat           // 获取半径
```

**线段形状:**
```c
cpSegmentShapeAlloc()
cpSegmentShapeInit(seg, body, a, b, radius)
cpSegmentShapeNew(body, a, b, radius)
cpSegmentShapeSetNeighbors(shape, prev, next)  // 设置邻居（避免端盖碰撞）
cpSegmentShapeGetA(shape) -> cpVect
cpSegmentShapeGetB(shape) -> cpVect
cpSegmentShapeGetNormal(shape) -> cpVect
cpSegmentShapeGetRadius(shape) -> cpFloat
```

### 3.7 cpPolyShape.h - 多边形形状

```c
cpPolyShapeAlloc()
cpPolyShapeInit(poly, body, count, verts, transform, radius)
cpPolyShapeNew(body, count, verts, transform, radius)
cpPolyShapeInitRaw(poly, body, count, verts, radius)
cpPolyShapeNewRaw(body, count, verts, radius)

// 矩形便捷函数
cpBoxShapeInit(poly, body, width, height, radius)
cpBoxShapeNew(body, width, height, radius)
cpBoxShapeInit2(poly, body, box, radius)
cpBoxShapeNew2(body, box, radius)

cpPolyShapeGetCount(shape) -> int
cpPolyShapeGetVert(shape, index) -> cpVect
cpPolyShapeGetRadius(shape) -> cpFloat
```

### 3.8 cpConstraint.h - 约束（关节）

**所有约束类型:**
- PinJoint - 销关节，保持两点间固定距离
- SlideJoint - 滑动关节，距离在范围内
- PivotJoint - 枢轴关节，公共锚点
- GrooveJoint - 槽关节，一端沿线段滑动
- DampedSpring - 阻尼弹簧
- DampedRotarySpring - 阻尼旋转弹簧
- RotaryLimitJoint - 旋转限制关节
- RatchetJoint - 棘轮关节
- GearJoint - 齿轮关节
- SimpleMotor - 简单电机

**基础约束函数:**
```c
cpConstraintDestroy(constraint)
cpConstraintFree(constraint)
cpConstraintGetSpace(constraint) -> cpSpace*
cpConstraintGetBodyA(constraint) -> cpBody*
cpConstraintGetBodyB(constraint) -> cpBody*
cpConstraintGetMaxForce(constraint) -> cpFloat
cpConstraintSetMaxForce(constraint, maxForce)
cpConstraintGetErrorBias(constraint) -> cpFloat
cpConstraintSetErrorBias(constraint, bias)
cpConstraintGetMaxBias(constraint) -> cpFloat
cpConstraintSetMaxBias(constraint, maxBias)
cpConstraintGetCollideBodies(constraint) -> cpBool
cpConstraintSetCollideBodies(constraint, collide)
cpConstraintGetPreSolveFunc(constraint) -> cpConstraintPreSolveFunc
cpConstraintSetPreSolveFunc(constraint, func)
cpConstraintGetPostSolveFunc(constraint) -> cpConstraintPostSolveFunc
cpConstraintSetPostSolveFunc(constraint, func)
cpConstraintGetUserData(constraint) -> void*
cpConstraintSetUserData(constraint, data)
cpConstraintGetImpulse(constraint) -> cpFloat  // 获取当前冲量

// 类型检查
cpConstraintIsPinJoint(c) -> cpBool
cpConstraintIsSlideJoint(c) -> cpBool
cpConstraintIsPivotJoint(c) -> cpBool
cpConstraintIsGrooveJoint(c) -> cpBool
cpConstraintIsDampedSpring(c) -> cpBool
cpConstraintIsDampedRotarySpring(c) -> cpBool
cpConstraintIsRotaryLimitJoint(c) -> cpBool
cpConstraintIsRatchetJoint(c) -> cpBool
cpConstraintIsGearJoint(c) -> cpBool
cpConstraintIsSimpleMotor(c) -> cpBool
```

**PinJoint（销关节）:**
```c
cpPinJointNew(a, b, anchorA, anchorB) -> cpConstraint*
cpPinJointGetAnchorA(joint) -> cpVect
cpPinJointSetAnchorA(joint, anchorA)
cpPinJointGetAnchorB(joint) -> cpVect
cpPinJointSetAnchorB(joint, anchorB)
cpPinJointGetDist(joint) -> cpFloat
cpPinJointSetDist(joint, dist)
```

**SlideJoint（滑动关节）:**
```c
cpSlideJointNew(a, b, anchorA, anchorB, min, max) -> cpConstraint*
cpSlideJointGetAnchorA(joint) -> cpVect
cpSlideJointSetAnchorA(joint, anchorA)
cpSlideJointGetAnchorB(joint) -> cpVect
cpSlideJointSetAnchorB(joint, anchorB)
cpSlideJointGetMin(joint) -> cpFloat
cpSlideJointSetMin(joint, min)
cpSlideJointGetMax(joint) -> cpFloat
cpSlideJointSetMax(joint, max)
```

**PivotJoint（枢轴关节）:**
```c
cpPivotJointNew(a, b, pivot) -> cpConstraint*     // 世界坐标锚点
cpPivotJointNew2(a, b, anchorA, anchorB) -> cpConstraint*  // 各自局部坐标
cpPivotJointGetAnchorA(joint) -> cpVect
cpPivotJointSetAnchorA(joint, anchorA)
cpPivotJointGetAnchorB(joint) -> cpVect
cpPivotJointSetAnchorB(joint, anchorB)
```

**GrooveJoint（槽关节）:**
```c
cpGrooveJointNew(a, b, groove_a, groove_b, anchorB) -> cpConstraint*
cpGrooveJointGetGrooveA(joint) -> cpVect
cpGrooveJointSetGrooveA(joint, grooveA)
cpGrooveJointGetGrooveB(joint) -> cpVect
cpGrooveJointSetGrooveB(joint, grooveB)
cpGrooveJointGetAnchorB(joint) -> cpVect
cpGrooveJointSetAnchorB(joint, anchorB)
```

**DampedSpring（阻尼弹簧）:**
```c
cpDampedSpringNew(a, b, anchorA, anchorB, restLength, stiffness, damping) -> cpConstraint*
cpDampedSpringGetAnchorA(joint) -> cpVect
cpDampedSpringSetAnchorA(joint, anchorA)
cpDampedSpringGetAnchorB(joint) -> cpVect
cpDampedSpringSetAnchorB(joint, anchorB)
cpDampedSpringGetRestLength(joint) -> cpFloat
cpDampedSpringSetRestLength(joint, restLength)
cpDampedSpringGetStiffness(joint) -> cpFloat
cpDampedSpringSetStiffness(joint, stiffness)
cpDampedSpringGetDamping(joint) -> cpFloat
cpDampedSpringSetDamping(joint, damping)
```

**DampedRotarySpring（阻尼旋转弹簧）:**
```c
cpDampedRotarySpringNew(a, b, restAngle, stiffness, damping) -> cpConstraint*
cpDampedRotarySpringGetRestAngle(joint) -> cpFloat
cpDampedRotarySpringSetRestAngle(joint, angle)
cpDampedRotarySpringGetStiffness(joint) -> cpFloat
cpDampedRotarySpringSetStiffness(joint, stiffness)
cpDampedRotarySpringGetDamping(joint) -> cpFloat
cpDampedRotarySpringSetDamping(joint, damping)
```

**RotaryLimitJoint（旋转限制关节）:**
```c
cpRotaryLimitJointNew(a, b, min, max) -> cpConstraint*
cpRotaryLimitJointGetMin(joint) -> cpFloat
cpRotaryLimitJointSetMin(joint, min)
cpRotaryLimitJointGetMax(joint) -> cpFloat
cpRotaryLimitJointSetMax(joint, max)
```

**RatchetJoint（棘轮关节）:**
```c
cpRatchetJointNew(a, b, phase, ratchet) -> cpConstraint*
cpRatchetJointGetAngle(joint) -> cpFloat
cpRatchetJointSetAngle(joint, angle)
cpRatchetJointGetPhase(joint) -> cpFloat
cpRatchetJointSetPhase(joint, phase)
cpRatchetJointGetRatchet(joint) -> cpFloat
cpRatchetJointSetRatchet(joint, ratchet)
```

**GearJoint（齿轮关节）:**
```c
cpGearJointNew(a, b, phase, ratio) -> cpConstraint*
cpGearJointGetPhase(joint) -> cpFloat
cpGearJointSetPhase(joint, phase)
cpGearJointGetRatio(joint) -> cpFloat
cpGearJointSetRatio(joint, ratio)
```

**SimpleMotor（简单电机）:**
```c
cpSimpleMotorNew(a, b, rate) -> cpConstraint*
cpSimpleMotorGetRate(joint) -> cpFloat
cpSimpleMotorSetRate(joint, rate)
```

### 3.9 cpArbiter.h - 碰撞仲裁器

```c
// 接触点信息
cpArbiterTotalImpulse(arb) -> cpVect         // 总冲量
cpArbiterTotalKE(arb) -> cpFloat             // 总动能
cpArbiterIgnore(arb) -> cpBool               // 忽略本次碰撞

// 获取碰撞形状和刚体
cpArbiterGetShapes(arb, &a, &b)
cpArbiterGetBodies(arb, &a, &b)

// 获取接触点集
cpArbiterGetContactPointSet(arb) -> cpContactPointSet
cpArbiterSetContactPointSet(arb, set)

// 状态查询
cpArbiterIsFirstContact(arb) -> cpBool       // 是否首次接触
cpArbiterIsRemoval(arb) -> cpBool           // 是否正在移除

// 基本信息
cpArbiterGetCount(arb) -> int                 // 接触点数量
cpArbiterGetNormal(arb) -> cpVect            // 碰撞法向量
cpArbiterGetPointA(arb, i) -> cpVect          // 形状A上的点
cpArbiterGetPointB(arb, i) -> cpVect          // 形状B上的点
cpArbiterGetDepth(arb, i) -> cpFloat          // 穿透深度

// 属性
cpArbiterGetRestitution(arb) -> cpFloat
cpArbiterSetRestitution(arb, restitution)
cpArbiterGetFriction(arb) -> cpFloat
cpArbiterSetFriction(arb, friction)
cpArbiterGetSurfaceVelocity(arb) -> cpVect
cpArbiterSetSurfaceVelocity(arb, vel)
cpArbiterGetUserData(arb) -> void*
cpArbiterSetUserData(arb, data)
```

### 3.10 cpSpace.h - 仿真空间

**内存管理:**
```c
cpSpaceAlloc() -> cpSpace*
cpSpaceInit(space) -> cpSpace*
cpSpaceNew() -> cpSpace*
cpSpaceDestroy(space)
cpSpaceFree(space)
```

**空间属性:**
```c
cpSpaceGetIterations(space) -> int
cpSpaceSetIterations(space, iterations)        // 求解器迭代次数，默认10

cpSpaceGetGravity(space) -> cpVect
cpSpaceSetGravity(space, gravity)             // 默认(0, 0)

cpSpaceGetDamping(space) -> cpFloat
cpSpaceSetDamping(space, damping)             // 默认1.0

cpSpaceGetIdleSpeedThreshold(space) -> cpFloat
cpSpaceSetIdleSpeedThreshold(space, threshold) // 默认INFINITY

cpSpaceGetSleepTimeThreshold(space) -> cpFloat
cpSpaceSetSleepTimeThreshold(space, threshold) // 默认INFINITY

cpSpaceGetCollisionSlop(space) -> cpFloat
cpSpaceSetCollisionSlop(space, slop)           // 默认0.1

cpSpaceGetCollisionBias(space) -> cpFloat
cpSpaceSetCollisionBias(space, bias)           // 默认0.001

cpSpaceGetCollisionPersistence(space) -> cpTimestamp
cpSpaceSetCollisionPersistence(space, persistence) // 默认3

cpSpaceGetUserData(space) -> void*
cpSpaceSetUserData(space, data)

cpSpaceGetStaticBody(space) -> cpBody*
cpSpaceGetCurrentTimeStep(space) -> cpFloat
cpSpaceIsLocked(space) -> cpBool              // 是否在回调中
```

**添加/移除对象:**
```c
cpSpaceAddShape(space, shape) -> cpShape*
cpSpaceAddBody(space, body) -> cpBody*
cpSpaceAddConstraint(space, constraint) -> cpConstraint*

cpSpaceRemoveShape(space, shape)
cpSpaceRemoveBody(space, body)
cpSpaceRemoveConstraint(space, constraint)

cpSpaceContainsShape(space, shape) -> cpBool
cpSpaceContainsBody(space, body) -> cpBool
cpSpaceContainsConstraint(space, constraint) -> cpBool
```

**碰撞处理器:**
```c
cpSpaceAddDefaultCollisionHandler(space) -> cpCollisionHandler*
cpSpaceAddCollisionHandler(space, a, b) -> cpCollisionHandler*  // 特定类型对
cpSpaceAddWildcardHandler(space, type) -> cpCollisionHandler* // 通配符类型
```

**碰撞回调签名:**
```c
typedef cpBool (*cpCollisionBeginFunc)(cpArbiter *arb, cpSpace *space, cpDataPointer data);
typedef cpBool (*cpCollisionPreSolveFunc)(cpArbiter *arb, cpSpace *space, cpDataPointer data);
typedef void (*cpCollisionPostSolveFunc)(cpArbiter *arb, cpSpace *space, cpDataPointer data);
typedef void (*cpCollisionSeparateFunc)(cpArbiter *arb, cpSpace *space, cpDataPointer data);
```

**空间查询:**
```c
// 点查询
cpSpacePointQuery(space, point, maxDistance, filter, func, data)
cpSpacePointQueryNearest(space, point, maxDistance, filter, out) -> cpShape*

// 线段查询
cpSpaceSegmentQuery(space, start, end, radius, filter, func, data)
cpSpaceSegmentQueryFirst(space, start, end, radius, filter, out) -> cpShape*

// 边界框查询
cpSpaceBBQuery(space, bb, filter, func, data)

// 形状查询
cpSpaceShapeQuery(space, shape, func, data) -> cpBool
```

**迭代:**
```c
cpSpaceEachBody(space, func, data)
cpSpaceEachShape(space, func, data)
cpSpaceEachConstraint(space, func, data)
```

**重索引:**
```c
cpSpaceReindexStatic(space)                   // 重新索引所有静态形状
cpSpaceReindexShape(space, shape)             // 重新索引单个形状
cpSpaceReindexShapesForBody(space, body)       // 为体重新索引所有形状
cpSpaceUseSpatialHash(space, dim, count)      // 使用空间哈希
```

**时间步进:**
```c
cpSpaceStep(space, dt)                        // 执行单个时间步
```

**调试绘制:**
```c
cpSpaceDebugDraw(space, options)
```

### 3.11 cpSpatialIndex.h - 空间索引

**Broad-phase 实现:**
```c
// AABB树（默认）
cpBBTreeAlloc()
cpBBTreeInit(tree, bbfunc, staticIndex)
cpBBTreeNew(bbfunc, staticIndex)
cpBBTreeOptimize(index)
cpBBTreeSetVelocityFunc(index, func)

// 空间哈希
cpSpaceHashAlloc()
cpSpaceHashInit(hash, celldim, numcells, bbfunc, staticIndex)
cpSpaceHashNew(celldim, cells, bbfunc, staticIndex)
cpSpaceHashResize(hash, celldim, numcells)

// 单轴扫描
cpSweep1DAlloc()
cpSweep1DInit(sweep, bbfunc, staticIndex)
cpSweep1DNew(bbfunc, staticIndex)
```

### 3.12 chipmunk.h - 主头文件

**质量/面积计算:**
```c
cpMomentForCircle(m, r1, r2, offset) -> cpFloat
cpAreaForCircle(r1, r2) -> cpFloat
cpMomentForSegment(m, a, b, radius) -> cpFloat
cpAreaForSegment(a, b, radius) -> cpFloat
cpMomentForPoly(m, count, verts, offset, radius) -> cpFloat
cpAreaForPoly(count, verts, radius) -> cpFloat
cpCentroidForPoly(count, verts) -> cpVect
cpMomentForBox(m, width, height) -> cpFloat
cpMomentForBox2(m, box) -> cpFloat
cpConvexHull(count, verts, result, first, tol) -> int
```

---

## 4. 源文件说明

### 核心实现文件

| 文件 | 功能 |
|------|------|
| `chipmunk.c` | 版本定义、凸包计算、辅助函数 |
| `cpSpace.c` | 空间管理、对象添加/移除、碰撞处理器 |
| `cpSpaceStep.c` | 时间步进、碰撞检测循环、求解器 |
| `cpSpaceComponent.c` | 休眠体组件管理 |
| `cpSpaceQuery.c` | 空间查询实现 |
| `cpBody.c` | 刚体力学、速度/位置更新 |
| `cpShape.c` | 圆形、线段形状实现 |
| `cpPolyShape.c` | 多边形形状实现 |
| `cpArbiter.c` | 接触点管理、回调调用 |
| `cpCollision.c` | GJK/EPA 窄相位碰撞检测 |

### 约束实现文件

| 文件 | 功能 |
|------|------|
| `cpConstraint.c` | 约束基类功能 |
| `cpPinJoint.c` | 销关节约束 |
| `cpSlideJoint.c` | 滑动关节约束 |
| `cpPivotJoint.c` | 枢轴关节约束 |
| `cpGrooveJoint.c` | 槽关节约束 |
| `cpDampedSpring.c` | 阻尼弹簧约束 |
| `cpDampedRotarySpring.c` | 阻尼旋转弹簧约束 |
| `cpRotaryLimitJoint.c` | 旋转限制约束 |
| `cpRatchetJoint.c` | 棘轮约束 |
| `cpGearJoint.c` | 齿轮约束 |
| `cpSimpleMotor.c` | 简单电机约束 |

### 空间索引实现

| 文件 | 功能 |
|------|------|
| `cpSpatialIndex.c` | 空间索引基类 |
| `cpBBTree.c` | AABB bounding树 |
| `cpSpaceHash.c` | 空间哈希网格 |
| `cpSweep1D.c` | 单轴扫描 |

### 工具文件

| 文件 | 功能 |
|------|------|
| `cpArray.c` | 动态数组实现 |
| `cpHashSet.c` | 哈希集合实现 |
| `cpPolyline.c` | 折线处理、凸分解 |
| `cpMarch.c` | Marching squares算法 |
| `cpHastySpace.c` | NEON优化多线程空间 |
| `cpRobust.c` | 严格浮点运算 |
| `cpSpaceDebug.c` | 调试绘制实现 |

---

## 5. 文件调用关系

### 头文件包含层次

```
chipmunk.h (主公共头文件)
├── chipmunk_types.h
├── cpVect.h
├── cpBB.h
├── cpTransform.h
├── cpSpatialIndex.h
├── cpArbiter.h
├── cpBody.h
├── cpShape.h
├── cpPolyShape.h
├── cpConstraint.h (包含所有关节头文件)
│   ├── cpPinJoint.h
│   ├── cpSlideJoint.h
│   ├── cpPivotJoint.h
│   ├── cpGrooveJoint.h
│   ├── cpDampedSpring.h
│   ├── cpDampedRotarySpring.h
│   ├── cpRotaryLimitJoint.h
│   ├── cpRatchetJoint.h
│   ├── cpGearJoint.h
│   └── cpSimpleMotor.h
└── cpSpace.h

chipmunk_structs.h
└── chipmunk.h

chipmunk_private.h (内部API)
├── chipmunk.h
└── chipmunk_structs.h

chipmunk_ffi.h
└── (无依赖)
```

### 源代码依赖关系

```
chipmunk.c
└── chipmunk_private.h

cpSpace.c
└── chipmunk_private.h

cpSpaceStep.c
└── chipmunk_private.h

cpBody.c
└── chipmunk_private.h

cpShape.c
├── chipmunk_private.h
└── chipmunk_unsafe.h

cpPolyShape.c
├── chipmunk_private.h
└── chipmunk_unsafe.h

cpCollision.c
├── chipmunk_private.h
└── cpRobust.h

cpArbiter.c
└── chipmunk_private.h

cpConstraint.c
├── chipmunk_private.h
└── chipmunk_unsafe.h

关节实现文件 (cpPinJoint.c 等)
└── chipmunk_private.h

cpSpatialIndex.c
└── chipmunk_private.h

cpBBTree.c, cpSpaceHash.c, cpSweep1D.c
└── chipmunk_private.h

cpPolyline.c
└── chipmunk_private.h

cpMarch.c
└── chipmunk_private.h

cpHastySpace.c
└── chipmunk_private.h
```

---

## 6. 核心数据结构

### 6.1 cpBody - 刚体

```c
struct cpBody {
    cpBodyVelocityFunc velocity_func;  // 自定义速度函数
    cpBodyPositionFunc position_func;   // 自定义位置函数

    // 质量属性
    cpFloat m, m_inv;                   // 质量和逆质量
    cpFloat i, i_inv;                   // 转动惯量和逆惯量
    cpVect cog;                          // 重心偏移

    // 状态
    cpVect p, v, f;                      // 位置、速度、受力
    cpFloat a, w, t;                     // 角度、角速度、力矩

    // 缓存变换矩阵
    cpTransform transform;

    // 用户数据
    cpDataPointer userData;

    // 内务用（穿透校正）
    cpVect v_bias, w_bias;

    // 所属空间
    cpSpace *space;

    // 附加的对象
    cpShape *shapeList;                  // 形状链表
    cpArbiter *arbiterList;              // 活动仲裁器链表
    cpConstraint *constraintList;        // 约束链表

    // 休眠组件
    struct {
        cpBody *root, *next;             // 组件根节点和下一个
        cpFloat idleTime;                 // 空闲时间
    } sleeping;
};
```

### 6.2 cpShape - 形状基类

```c
struct cpShape {
    const cpShapeClass *klass;           // 类型虚表
    cpSpace *space;                      // 所属空间
    cpBody *body;                        // 附加刚体

    // 质量信息
    struct cpShapeMassInfo {
        cpFloat m, i;                    // 质量和转动惯量
        cpVect cog;                       // 重心
        cpFloat area;                      // 面积
    } massInfo;

    cpBB bb;                             // 边界框

    // 材质
    cpBool sensor;                       // 传感器（无物理响应）
    cpFloat e;                           // 弹性（恢复系数）
    cpFloat u;                           // 摩擦系数
    cpVect surfaceV;                     // 表面速度

    // 用户数据
    cpDataPointer userData;

    // 碰撞过滤
    cpCollisionType type;                // 碰撞类型
    cpShapeFilter filter;               // 过滤条件

    // 链表
    cpShape *next, *prev;

    // 空间索引用
    cpHashValue hashid;                  // 唯一ID
};
```

### 6.3 cpConstraint - 约束基类

```c
struct cpConstraint {
    const cpConstraintClass *klass;     // 类型虚表
    cpSpace *space;                      // 所属空间
    cpBody *a, *b;                       // 连接的两个刚体

    // 链表
    cpConstraint *next_a, *next_b;

    // 约束参数
    cpFloat maxForce;                    // 最大力
    cpFloat errorBias;                    // 误差校正率
    cpFloat maxBias;                      // 最大校正量

    // 碰撞
    cpBool collideBodies;                 // 两体是否可碰撞

    // 回调
    cpConstraintPreSolveFunc preSolve;   // 解算前回调
    cpConstraintPostSolveFunc postSolve; // 解算后回调

    // 用户数据
    cpDataPointer userData;
};
```

### 6.4 cpArbiter - 碰撞仲裁器

```c
struct cpArbiter {
    // 组合材质
    cpFloat e, u;                        // 弹性和摩擦
    cpVect surface_vr;                   // 相对表面速度

    // 用户数据
    cpDataPointer data;

    // 碰撞对
    const cpShape *a, *b;
    cpBody *body_a, *body_b;

    // 链表线程
    struct cpArbiterThread {
        cpArbiter *next, *prev;
    } thread_a, thread_b;

    // 接触点
    int count;
    struct cpContact *contacts;
    cpVect n;                            // 碰撞法向量

    // 处理器
    cpCollisionHandler *handler, *handlerA, *handlerB;

    // 状态
    cpBool swapped;                      // 是否交换了a/b
    cpTimestamp stamp;                   // 上次使用时间戳

    enum cpArbiterState {
        CP_ARBITER_STATE_FIRST_COLLISION,  // 首次碰撞
        CP_ARBITER_STATE_NORMAL,           // 正常碰撞
        CP_ARBITER_STATE_IGNORE,           // 忽略
        CP_ARBITER_STATE_CACHED,            // 缓存
        CP_ARBITER_STATE_INVALIDATED        // 无效
    } state;
};
```

### 6.5 cpSpace - 仿真空间

```c
struct cpSpace {
    // 求解器参数
    int iterations;                      // 迭代次数
    cpVect gravity;                       // 重力
    cpFloat damping;                      // 阻尼
    cpFloat idleSpeedThreshold;           // 空闲速度阈值
    cpFloat sleepTimeThreshold;           // 休眠时间阈值
    cpFloat collisionSlop;               // 碰撞允许量
    cpFloat collisionBias;               // 碰撞校正率
    cpTimestamp collisionPersistence;     // 接触缓存帧数

    // 用户数据
    cpDataPointer userData;

    // 时间
    cpTimestamp stamp;                   // 当前时间戳
    cpFloat curr_dt;                     // 当前时间步

    // 对象管理
    cpArray *dynamicBodies;              // 动态体数组
    cpArray *staticBodies;               // 静态体数组
    cpArray *rousedBodies;               // 待唤醒体数组
    cpArray *sleepingComponents;         // 休眠组件数组

    // 空间索引
    cpHashValue shapeIDCounter;          // 形状ID计数器
    cpSpatialIndex *staticShapes;        // 静态形状索引
    cpSpatialIndex *dynamicShapes;       // 动态形状索引

    // 约束
    cpArray *constraints;                 // 约束数组

    // 碰撞
    cpArray *arbiters;                    // 仲裁器数组
    cpContactBufferHeader *contactBuffersHead;  // 接触缓冲环
    cpHashSet *cachedArbiters;            // 缓存仲裁器哈希
    cpArray *pooledArbiters;              // 对象池
    cpArray *allocatedBuffers;            // 已分配缓冲

    // 回调锁定
    int locked;

    // 碰撞处理器
    cpBool usesWildcards;
    cpHashSet *collisionHandlers;
    cpCollisionHandler defaultHandler;

    // 后步回调
    cpBool skipPostStep;
    cpArray *postStepCallbacks;

    // 静态体
    cpBody *staticBody;
    cpBody _staticBody;
};
```

---

## 7. API详细说明

### 7.1 完整使用流程

```c
// 1. 创建空间
cpSpace *space = cpSpaceNew();

// 2. 配置空间
cpSpaceSetGravity(space, cpv(0, -100));  // 设置重力
cpSpaceSetIterations(space, 10);          // 迭代次数

// 3. 创建静态边界（地面、墙壁）
cpBody *staticBody = cpSpaceGetStaticBody(space);
cpShape *ground = cpSegmentShapeNew(staticBody, cpv(-320, -240), cpv(320, -240), 0);
cpShapeSetElasticity(ground, 1.0f);
cpShapeSetFriction(ground, 1.0f);
cpSpaceAddShape(space, ground);

// 4. 创建动态体
cpFloat mass = 1.0f;
cpBody *body = cpBodyNew(mass, cpMomentForCircle(mass, 0, 20, cpvzero));
cpBodySetPosition(body, cpv(0, 100));
cpSpaceAddBody(space, body);

// 5. 创建形状
cpShape *shape = cpCircleShapeNew(body, 20, cpvzero);
cpShapeSetElasticity(shape, 0.5f);
cpShapeSetFriction(shape, 0.5f);
cpSpaceAddShape(space, shape);

// 6. 添加约束
cpConstraint *joint = cpPinJointNew(body, staticBody, cpvzero, cpvzero);
cpSpaceAddConstraint(space, joint);

// 7. 碰撞回调（可选）
cpCollisionHandler *handler = cpSpaceAddCollisionHandler(space, 1, 2);
handler->beginFunc = myBeginFunc;
handler->preSolveFunc = myPreSolveFunc;
handler->postSolveFunc = myPostSolveFunc;
handler->separateFunc = mySeparateFunc;

// 8. 时间步进
cpSpaceStep(space, 1.0/60.0);

// 9. 获取状态
cpVect pos = cpBodyGetPosition(body);
cpFloat angle = cpBodyGetAngle(body);

// 10. 清理
ChipmunkDemoFreeSpaceChildren(space);  // 释放所有子对象
cpSpaceFree(space);
```

### 7.2 质量属性计算

```c
// 圆形惯量
cpFloat moment = cpMomentForCircle(mass, innerRadius, outerRadius, offset);

// 线段惯量
cpFloat moment = cpMomentForSegment(mass, a, b, radius);

// 多边形惯量
cpFloat moment = cpMomentForPoly(mass, count, verts, offset, radius);

// 矩形惯量
cpFloat moment = cpMomentForBox(mass, width, height);
cpFloat moment = cpMomentForBox2(mass, box);
```

### 7.3 碰撞过滤

```c
// 定义过滤结构
cpShapeFilter filter = {
    .group = 0,              // 组，相同组不碰撞
    .categories = 0x0001,    // 所属分类
    .mask = 0x0002           // 碰撞掩码
};

// 创建常用过滤
cpShapeFilter GRAB_FILTER = {CP_NO_GROUP, GRABBABLE_MASK_BIT, GRABBABLE_MASK_BIT};
cpShapeFilter NOT_GRABABLE_FILTER = {CP_NO_GROUP, ~GRABBABLE_MASK_BIT, ~GRABBABLE_MASK_BIT};

// 设置到形状
cpShapeSetFilter(shape, filter);

// 查询时使用
cpSpacePointQueryNearest(space, point, 100.0f, GRAB_FILTER, &info);
```

### 7.4 空间查询

```c
// 点查询最近形状
cpPointQueryInfo info = {0};
cpShape *shape = cpSpacePointQueryNearest(space, point, 100.0f, filter, &info);
if(shape) {
    printf("Found shape at (%f, %f), distance=%f\n",
           info.point.x, info.point.y, info.distance);
}

// 线段查询
cpSegmentQueryInfo info = {0};
cpShape *hit = cpSpaceSegmentQueryFirst(space, start, end, 0.0f, filter, &info);
if(hit) {
    printf("Hit at (%f, %f), normal=(%f, %f)\n",
           info.point.x, info.point.y, info.normal.x, info.normal.y);
}

// 边界框查询
cpSpaceBBQuery(space, bb, filter, myCallback, data);

// 形状查询（检测形状是否碰撞）
cpBool collided = cpSpaceShapeQuery(space, shape, callback, data);
```

### 7.5 自定义集成

```c
// 自定义速度更新
void myVelocityFunc(cpBody *body, cpVect gravity, cpFloat damping, cpFloat dt) {
    // 自定义速度更新逻辑
    body->v = cpvadd(body->v, cpvmult(gravity, dt));
    body->v = cpvmult(body->v, 1.0 - damping * dt);
}
cpBodySetVelocityUpdateFunc(body, myVelocityFunc);

// 自定义位置更新
void myPositionFunc(cpBody *body, cpFloat dt) {
    body->p = cpvadd(body->p, cpvmult(body->v, dt));
    body->a = body->a + body->w * dt;
}
cpBodySetPositionUpdateFunc(body, myPositionFunc);
```

### 7.6 休眠系统

```c
// 设置休眠阈值
cpSpaceSetSleepTimeThreshold(space, 0.5f);     // 空闲0.5秒后入睡
cpSpaceSetIdleSpeedThreshold(space, 10.0f);    // 速度低于10入睡

// 手动控制休眠
cpBodySleep(body);                              // 使体入睡
cpBodySleepWithGroup(body, groupBody);           // 与组一起入睡
cpBodyActivate(body);                           // 唤醒

// 唤醒接触静态形状的动态体
cpBodyActivateStatic(body, filter);            // filter为NULL时唤醒所有
```

### 7.7 后步回调

```c
// 在时间步结束后执行（安全移除对象）
void myFreeCallback(cpSpace *space, void *key, void *data) {
    // 在此处释放对象是安全的
}
cpSpaceAddPostStepCallback(space, myFreeCallback, key, data);
```

---

## 8. 示例程序

### 8.1 基础物理模拟

```c
#include "chipmunk/chipmunk.h"

int main() {
    // 创建空间
    cpSpace *space = cpSpaceNew();
    cpSpaceSetGravity(space, cpv(0, -100));

    // 创建地面
    cpBody *staticBody = cpSpaceGetStaticBody(space);
    cpShape *ground = cpSegmentShapeNew(staticBody, cpv(-400, -300), cpv(400, -300), 0);
    cpShapeSetElasticity(ground, 0.8f);
    cpShapeSetFriction(ground, 0.5f);
    cpSpaceAddShape(space, ground);

    // 创建墙壁
    cpShape *wall1 = cpSegmentShapeNew(staticBody, cpv(-400, -300), cpv(-400, 300), 0);
    cpShapeSetElasticity(wall1, 0.8f);
    cpShapeSetFriction(wall1, 0.5f);
    cpSpaceAddShape(space, wall1);

    cpShape *wall2 = cpSegmentShapeNew(staticBody, cpv(400, -300), cpv(400, 300), 0);
    cpShapeSetElasticity(wall2, 0.8f);
    cpShapeSetFriction(wall2, 0.5f);
    cpSpaceAddShape(space, wall2);

    // 创建多个动态球
    for(int i = 0; i < 10; i++) {
        cpFloat radius = 15.0f;
        cpFloat mass = 1.0f;
        cpBody *ball = cpBodyNew(mass, cpMomentForCircle(mass, 0, radius, cpvzero));
        cpBodySetPosition(ball, cpv(-200 + i * 40, 200));
        cpBodySetVelocity(ball, cpv(0, -50));

        cpShape *ballShape = cpCircleShapeNew(ball, radius, cpvzero);
        cpShapeSetElasticity(ballShape, 0.9f);
        cpShapeSetFriction(ballShape, 0.5f);

        cpSpaceAddBody(space, ball);
        cpSpaceAddShape(space, ballShape);
    }

    // 模拟循环
    for(int step = 0; step < 600; step++) {
        cpSpaceStep(space, 1.0/60.0);

        // 打印第一个球的位置
        cpBody *ball = cpSpaceGetDynamicBodies(space)[0];
        printf("Step %d: pos=(%.2f, %.2f), vel=(%.2f, %.2f)\n",
               step,
               cpBodyGetPosition(ball).x,
               cpBodyGetPosition(ball).y,
               cpBodyGetVelocity(ball).x,
               cpBodyGetVelocity(ball).y);
    }

    // 清理
    ChipmunkDemoFreeSpaceChildren(space);
    cpSpaceFree(space);

    return 0;
}
```

### 8.2 关节连接示例

```c
#include "chipmunk/chipmunk.h"

int main() {
    cpSpace *space = cpSpaceNew();
    cpSpaceSetGravity(space, cpv(0, -500));
    cpSpaceSetIterations(space, 10);

    // 静态梁
    cpBody *staticBody = cpSpaceGetStaticBody(space);
    cpShape *beam = cpSegmentShapeNew(staticBody, cpv(-200, 100), cpv(200, 100), 10);
    cpShapeSetElasticity(beam, 0);
    cpShapeSetFriction(beam, 1);
    cpSpaceAddShape(space, beam);

    // 创建两个球
    cpFloat radius = 20;
    cpFloat mass = 1;

    cpBody *ball1 = cpBodyNew(mass, cpMomentForCircle(mass, 0, radius, cpvzero));
    cpBodySetPosition(ball1, cpv(-50, 200));
    cpSpaceAddBody(space, ball1);
    cpShape *shape1 = cpCircleShapeNew(ball1, radius, cpvzero);
    cpShapeSetFriction(shape1, 0.8);
    cpSpaceAddShape(space, shape1);

    cpBody *ball2 = cpBodyNew(mass, cpMomentForCircle(mass, 0, radius, cpvzero));
    cpBodySetPosition(ball2, cpv(50, 200));
    cpSpaceAddBody(space, ball2);
    cpShape *shape2 = cpCircleShapeNew(ball2, radius, cpvzero);
    cpShapeSetFriction(shape2, 0.8);
    cpSpaceAddShape(space, shape2);

    // 添加销关节连接两个球
    cpConstraint *pin = cpPinJointNew(ball1, ball2, cpv(radius, 0), cpv(-radius, 0));
    cpSpaceAddConstraint(space, pin);

    // 添加阻尼弹簧到静态体
    cpConstraint *spring = cpDampedSpringNew(ball1, staticBody,
        cpv(0, 0),              // ball1上的锚点
        cpv(-50, 200),          // 静态体上的锚点（世界坐标）
        50.0,                   // 自然长度
        200.0,                  // 刚度
        20.0);                  // 阻尼
    cpSpaceAddConstraint(space, spring);

    // 模拟
    for(int i = 0; i < 300; i++) {
        cpSpaceStep(space, 1.0/60.0);
        printf("Ball1: (%.1f, %.1f) Ball2: (%.1f, %.1f)\n",
               cpBodyGetPosition(ball1).x, cpBodyGetPosition(ball1).y,
               cpBodyGetPosition(ball2).x, cpBodyGetPosition(ball2).y);
    }

    ChipmunkDemoFreeSpaceChildren(space);
    cpSpaceFree(space);
    return 0;
}
```

### 8.3 碰撞回调示例

```c
#include "chipmunk/chipmunk.h"

// 定义碰撞类型
typedef enum {
    COLLISION_BALL = 1,
    COLLISION_WALL = 2,
    COLLISION_SENSOR = 3
} CollisionType;

// 碰撞开始回调
cpBool onCollisionBegin(cpArbiter *arb, cpSpace *space, cpDataPointer data) {
    cpShape *a, *b;
    cpArbiterGetShapes(arb, &a, &b);

    printf("Collision begin between types %lu and %lu\n",
           (unsigned long)cpShapeGetCollisionType(a),
           (unsigned long)cpShapeGetCollisionType(b));

    // 返回cpTrue继续正常碰撞处理
    // 返回cpFalse使这对碰撞不产生物理响应
    return cpTrue;
}

// 解算前回调（可用于修改碰撞参数）
cpBool onCollisionPreSolve(cpArbiter *arb, cpSpace *space, cpDataPointer data) {
    // 可以修改摩擦系数、弹性等
    cpArbiterSetFriction(arb, 1.0);
    return cpTrue;  // 返回cpFalse跳过本次碰撞
}

// 解算后回调（用于音效、粒子效果等）
void onCollisionPostSolve(cpArbiter *arb, cpSpace *space, cpDataPointer data) {
    cpVect impulse = cpArbiterTotalImpulse(arb);
    printf("Collision impulse: (%.2f, %.2f)\n", impulse.x, impulse.y);
}

// 分离回调
void onCollisionSeparate(cpArbiter *arb, cpSpace *space, cpDataPointer data) {
    printf("Collision separate\n");
}

int main() {
    cpSpace *space = cpSpaceNew();
    cpSpaceSetGravity(space, cpv(0, -100));

    // 添加碰撞处理器
    cpCollisionHandler *handler = cpSpaceAddCollisionHandler(space,
        COLLISION_BALL, COLLISION_WALL);
    handler->beginFunc = onCollisionBegin;
    handler->preSolveFunc = onCollisionPreSolve;
    handler->postSolveFunc = onCollisionPostSolve;
    handler->separateFunc = onCollisionSeparate;

    // 创建静态墙
    cpBody *staticBody = cpSpaceGetStaticBody(space);
    cpShape *wall = cpSegmentShapeNew(staticBody, cpv(-300, -200), cpv(300, -200), 0);
    cpShapeSetCollisionType(wall, COLLISION_WALL);
    cpSpaceAddShape(space, wall);

    // 创建动态球
    cpBody *ball = cpBodyNew(1, cpMomentForCircle(1, 0, 20, cpvzero));
    cpBodySetPosition(ball, cpv(0, 200));
    cpShape *ballShape = cpCircleShapeNew(ball, 20, cpvzero);
    cpShapeSetCollisionType(ballShape, COLLISION_BALL);
    cpSpaceAddBody(space, ball);
    cpSpaceAddShape(space, ballShape);

    // 模拟
    for(int i = 0; i < 120; i++) {
        cpSpaceStep(space, 1.0/60.0);
    }

    ChipmunkDemoFreeSpaceChildren(space);
    cpSpaceFree(space);
    return 0;
}
```

### 8.4 空间查询示例

```c
#include "chipmunk/chipmunk.h"

// 点查询回调
void pointQueryCallback(cpShape *shape, cpVect point, cpFloat distance, cpVect gradient, void *data) {
    printf("  Found shape at distance %.2f\n", distance);
}

// 线段查询回调
void segmentQueryCallback(cpShape *shape, cpVect p, cpVect n, cpFloat t, void *data) {
    printf("  Hit at t=%.2f, point=(%.1f, %.1f), normal=(%.1f, %.1f)\n",
           t, p.x, p.y, n.x, n.y);
}

int main() {
    cpSpace *space = cpSpaceNew();

    // 添加一些形状
    cpBody *body = cpSpaceAddBody(space, cpBodyNew(1, INFINITY));
    cpBodySetPosition(body, cpv(0, 0));
    cpShape *circle = cpCircleShapeNew(body, 30, cpvzero);
    cpSpaceAddShape(space, circle);

    // 边界
    cpBody *staticBody = cpSpaceGetStaticBody(space);
    cpShape *ground = cpSegmentShapeNew(staticBody, cpv(-300, -200), cpv(300, -200), 0);
    cpSpaceAddShape(space, ground);

    // 点查询
    printf("Point query at (50, 0):\n");
    cpPointQueryInfo info = {0};
    cpShape *nearest = cpSpacePointQueryNearest(space, cpv(50, 0), 100,
        CP_SHAPE_FILTER_ALL, &info);
    if(nearest) {
        printf("  Nearest shape, distance=%.2f, point=(%.1f, %.1f)\n",
               info.distance, info.point.x, info.point.y);
    }

    // 线段查询
    printf("\nSegment query from (-100, 100) to (100, -100):\n");
    cpSegmentQueryInfo segInfo = {0};
    cpShape *hit = cpSpaceSegmentQueryFirst(space,
        cpv(-100, 100), cpv(100, -100), 0, CP_SHAPE_FILTER_ALL, &segInfo);
    if(hit) {
        printf("  Hit shape, alpha=%.2f, point=(%.1f, %.1f), normal=(%.1f, %.1f)\n",
               segInfo.alpha, segInfo.point.x, segInfo.point.y,
               segInfo.normal.x, segInfo.normal.y);
    }

    // 边界框查询
    printf("\nBB query for (-50, -50, 50, 50):\n");
    cpBB bb = cpBBNew(-50, -50, 50, 50);
    cpSpaceBBQuery(space, bb, CP_SHAPE_FILTER_ALL, (cpSpaceBBQueryFunc)pointQueryCallback, NULL);

    ChipmunkDemoFreeSpaceChildren(space);
    cpSpaceFree(space);
    return 0;
}
```

### 8.5 堆叠金字塔示例

```c
#include "chipmunk/chipmunk.h"

int main() {
    cpSpace *space = cpSpaceNew();
    cpSpaceSetGravity(space, cpv(0, -300));
    cpSpaceSetIterations(space, 10);
    cpSpaceSetSleepTimeThreshold(space, 0.5f);

    // 地面
    cpBody *staticBody = cpSpaceGetStaticBody(space);
    cpShape *ground = cpSegmentShapeNew(staticBody, cpv(-320, -240), cpv(320, -240), 0);
    cpShapeSetElasticity(ground, 0.5f);
    cpShapeSetFriction(ground, 0.8f);
    cpSpaceAddShape(space, ground);

    // 创建金字塔
    int rows = 8;
    for(int i = 0; i < rows; i++) {
        for(int j = 0; j <= i; j++) {
            cpFloat size = 30.0f;
            cpFloat mass = 1.0f;

            // 创建动态体
            cpBody *box = cpBodyNew(mass, cpMomentForBox(mass, size, size));

            // 位置：金字塔形状
            cpFloat x = (j - i/2.0f) * size * 1.1f;
            cpFloat y = -240 + (rows - i) * size * 0.9f;
            cpBodySetPosition(box, cpv(x, y));

            // 随机初始旋转
            cpBodySetAngle(box, (cpFloat)(rand() % 100) / 100.0f * 0.1f);

            cpSpaceAddBody(space, box);

            // 创建碰撞形状
            cpShape *boxShape = cpBoxShapeNew(box, size, size, 0.5f);
            cpShapeSetElasticity(boxShape, 0.0f);
            cpShapeSetFriction(boxShape, 0.8f);
            cpSpaceAddShape(space, boxShape);
        }
    }

    // 模拟并每30步打印状态
    for(int step = 0; step < 300; step++) {
        cpSpaceStep(space, 1.0/60.0);

        if(step % 30 == 0) {
            printf("Step %d\n", step);
        }
    }

    ChipmunkDemoFreeSpaceChildren(space);
    cpSpaceFree(space);
    return 0;
}
```

---

## 9. 碰撞检测流程

```
cpSpaceStep(space, dt)
│
├─ 1. 位置积分 (position integration)
│   └─ cpBodyUpdatePosition() for each body
│
├─ 2. 形状边界框更新 (shape cache BB)
│   └─ cpShapeCacheBB() for each shape
│
├─ 3. Broad-phase碰撞检测
│   └─ cpSpatialIndexReindexQuery()
│      └─ 使用空间索引（BBTree/SpaceHash/Sweep1D）
│      └─ 返回潜在碰撞对列表
│
├─ 4. Narrow-phase碰撞检测
│   └─ cpCollideShapes() for each pair
│      ├─ GJK算法计算穿透深度和分离轴
│      └─ EPA算法（如重叠）计算接触点
│
├─ 5. Arbiter创建/更新
│   └─ cpArbiterInit() 或更新现有arbiter
│
├─ 6. 碰撞回调
│   ├─ beginFunc (首次碰撞)
│   ├─ preSolveFunc (每次解算前)
│   └─ 设置仲裁器状态
│
├─ 7. 休眠检测 (if enabled)
│   └─ cpSpaceProcessComponents()
│
├─ 8. Pre-step (约束预解算)
│   └─ cpConstraintPreStep()
│   └─ cpArbiterPreStep()
│
├─ 9. Velocity积分
│   └─ cpBodyUpdateVelocity() for each body
│
├─ 10. 求解器迭代 (solver iterations)
│   ├─ 接触冲量
│   └─ 约束冲量
│
├─ 11. Post-solve回调
│   └─ postSolveFunc
│
├─ 12. Separation回调
│   └─ separateFunc (当arbiter移除时)
│
└─ 13. 后步回调
    └─ postStepCallbacks
```

---

## 附录：编译说明

### CMake (跨平台)
```bash
mkdir build && cd build
cmake ..
make
```

### Xcode (macOS/iOS)
```bash
open xcode/Chipmunk2D.xcodeproj
```

### Visual Studio (Windows)
打开 `msvc/Chipmunk2D.sln`

---

## 附录：数据结构图表

### 体-形状-约束关系

```
cpBody
├── shapeList → cpShape → (attached to body)
├── arbiterList → cpArbiter → (contact with other bodies)
└── constraintList → cpConstraint → (connects to other body)
```

### 空间结构

```
cpSpace
├── dynamicBodies []     → cpBody (CP_BODY_TYPE_DYNAMIC)
├── staticBodies []      → cpBody (CP_BODY_TYPE_STATIC)
├── sleepingComponents []→ cpBody (sleeping groups)
├── staticShapes         → cpSpatialIndex
├── dynamicShapes        → cpSpatialIndex
├── constraints []       → cpConstraint
├── arbiters []          → cpArbiter (active contacts)
└── staticBody           → cpBody (for static shapes)
```

---

本文档由 Chipmunk2D 仓库源码分析生成。
如需更详细的信息，请参考源码注释和官方文档。