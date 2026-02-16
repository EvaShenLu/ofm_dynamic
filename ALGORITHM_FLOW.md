# OFM (One-Step Flow Maps) 算法详细流程

本文档详细描述了OFM流体模拟算法的完整计算流程。OFM在LFM（Leapfrog Flow Maps）基础上进行了关键改进，主要面向**动态边界**场景和**实时性能**优化。

## 目录
1. [算法概述](#算法概述)
2. [数据结构](#数据结构)
3. [初始化阶段](#初始化阶段)
4. [主循环流程](#主循环流程)
5. [Advance阶段详解](#advance阶段详解)
6. [Reinit阶段详解](#reinit阶段详解)
7. [动态边界处理](#动态边界处理)
8. [关键函数说明](#关键函数说明)
9. [函数所在文件对照表](#函数所在文件对照表)

---

## 算法概述

OFM (One-Step Flow Maps) 是一种面向实时流体模拟的Flow Map算法，在LFM基础上做了以下核心简化与扩展：

- **One-Step 积分**：每帧仅执行一次Advance和一次Reinit，相当于`reinit_every_ = 1`
- **Flow Maps（流映射）**：保留后向Flow Map (ψ, T) 和前向Flow Map (φ, F)
- **BFECC 误差修正**：沿用LFM的BFECC进行advection误差补偿
- **动态边界**：通过实时体素化支持任意运动固体与流体的交互
- 在MAC（Marker-And-Cell）网格上使用交错网格存储速度场

### 核心概念

1. **Flow Maps（流映射）**：
   - **后向Flow Map (ψ, T)**：记录粒子从初始位置到当前位置的轨迹
   - **前向Flow Map (φ, F)**：记录粒子从当前位置回到初始位置的轨迹
   - **T和F**：切向量，用于Pullback时正确变换向量场

2. **One-Step 方案**：
   - 每帧只执行一次Advance和一次Reinit
   - 中间速度场仅保存一份（`mid_u_x_`, `mid_u_y_`, `mid_u_z_`）
   - 时间步长：`dt = 1.0f / frame_rate`（整帧时间）

3. **BFECC 修正**：
   - 与LFM相同：通过前向、后向advection计算误差并修正

---

## 数据结构

### 速度场存储（MAC网格）
- `u_x_`, `u_y_`, `u_z_`：当前速度场的三个分量（交错网格）
- `init_u_x_`, `init_u_y_`, `init_u_z_`：初始速度场（reinit时刻的速度）
- `mid_u_x_`, `mid_u_y_`, `mid_u_z_`：**单份**中间速度场（Advance后的投影速度）
- `tmp_u_x_`, `tmp_u_y_`, `tmp_u_z_`：临时缓冲区
- `err_u_x_`, `err_u_y_`, `err_u_z_`：误差场（用于BFECC）

### Flow Maps
- **后向Flow Map**：
  - `psi_x_`, `psi_y_`, `psi_z_`：后向流映射的位置（3D向量）
  - `T_x_`, `T_y_`, `T_z_`：后向流映射的切向量（3D向量）
- **前向Flow Map**：
  - `phi_x_`, `phi_y_`, `phi_z_`：前向流映射的位置（3D向量）
  - `F_x_`, `F_y_`, `F_z_`：前向流映射的切向量（3D向量）

### 动态边界相关
- `use_dynamic_solid_`：是否启用动态固体
- `voxel_tex_`：体素化固体纹理（CUDA Surface）
- `velocity_tex_`：固体表面速度纹理
- `voxelized_velocity_scaler_`：固体速度缩放系数

### 其他
- `step_`：当前步数
- `dx_`：网格间距
- `grid_origin_`：网格原点

---

## 初始化阶段

### `InitOFMAsync()` 流程  
**所在文件**：声明 `src/ofm/ofm_init.h`，实现 `src/ofm/ofm_init.cu`

```cpp
void InitOFMAsync(OFM& _ofm, const OFMConfiguration& _config, cudaStream_t _stream)
```

#### 步骤1：内存分配
- 调用`_ofm.Alloc(tile_dim)`分配所有缓冲区
- MAC网格尺寸与LFM相同：
  - X分量：`(tile_dim.x + 1) × tile_dim.y × tile_dim.z × 512`
  - Y分量：`tile_dim.x × (tile_dim.y + 1) × tile_dim.z × 512`
  - Z分量：`tile_dim.x × tile_dim.y × (tile_dim.z + 1) × 512`
- 中心网格：`tile_dim.x × tile_dim.y × tile_dim.z × 512`

#### 步骤2：设置模拟参数
- `step_ = 0`：初始化步数
- `dx_`：网格间距
- `grid_origin_`：网格原点

#### 步骤3：设置边界条件
- 设置壁面边界条件（`SetWallBcAsync`，定义于 `src/ofm/ofm_util.cu`）
- 若使用静态固体，从SDF文件加载并设置边界（`SetBcByPhiAsync`）
- 设置入口速度（`inlet_norm_`, `inlet_angle_`）

#### 步骤4：动态固体配置
- `use_dynamic_solid_`：是否启用动态固体（从配置读取）
- 若启用，需在外部绑定`voxel_tex_`和`velocity_tex_`（在`physics.cu`的`init`中完成）

#### 步骤5：初始化Poisson求解器
- 设置系数矩阵（`SetCoefByIsBcAsync`）
- 构建AMGPCG求解器（`BuildAsync`）
- `solve_by_tol_ = false`，`max_iter_ = 6`（固定迭代次数）

#### 步骤6：BFECC clamp
- `use_bfecc_clamp_`：是否使用BFECC clamping

---

## 主循环流程

### `PhysicsEngineUser::step()` 主循环  
**所在文件**：`proj/dynamic_obstacle/physics.cu`

```cpp
void PhysicsEngineUser::step()
{
    // 1. 渲染数据准备
    GetCenteralVecAsync(*(ofm_.u_), ofm_.tile_dim_, ...);
    GetVorNormAsync(*(ofm_.vor_norm_), ofm_.tile_dim_, *(ofm_.u_), ofm_.dx_, streamToRun);
    writeToVorticity(...);

    // 2. 物理模拟（每帧一次 Advance + 一次 Reinit）
    float dt = 1.0f / static_cast<float>(frame_rate);
    if (current_frame > 0)
        ofm_.UpdateBoundary(streamToRun);  // 动态边界更新
    ofm_.AdvanceAsync(dt, streamToRun);
    ofm_.ReinitAsync(dt, streamToRun);
    current_frame++;
}
```

**关键点**：
- 每帧执行**一次**`AdvanceAsync`和**一次**`ReinitAsync`（One-Step）
- 时间步长：`dt = 1.0f / frame_rate`（整帧时间）
- 若启用动态固体，每帧先调用`UpdateBoundary`更新边界

---

## Advance阶段详解

### `AdvanceAsync(float _dt, cudaStream_t _stream)` 流程  
**所在文件**：声明 `src/ofm/ofm.h`，实现 `src/ofm/ofm.cu`

每帧执行一次，逻辑相比LFM大幅简化。

#### 步骤1：固定时间步长和速度场源

```cpp
float mid_dt = 0.5f * _dt;           // 固定为半时间步
last_proj_u = init_u;                // 使用初始速度场
src_u = init_u;                       // 源速度场也是初始速度场
```

**与LFM对比**：LFM根据`step_ % reinit_every_`选择`mid_dt`（0.5dt、dt或2dt）和不同的`src_u`；OFM始终使用`mid_dt = 0.5 * dt`和`init_u`。

#### 步骤2：Advection（平流）

```cpp
AdvectN2XAsync(*tmp_u_x_, tile_dim_, *src_u_x, *last_proj_u_x, *last_proj_u_y, *last_proj_u_z, dx_, mid_dt, _stream);
AdvectN2YAsync(...);
AdvectN2ZAsync(...);
```

使用RK2方法对速度场进行advection，与LFM相同。

#### 步骤3：设置入口边界条件

```cpp
SetInletAsync(*bc_val_x_, *bc_val_y_, tile_dim_, inlet_norm_, inlet_angle_, _stream);
```

#### 步骤4：压力投影（Projection）

```cpp
ProjectAsync(_stream);
```

流程与LFM一致：设置边界 → 计算散度 → 求解Poisson → 应用压力梯度。

#### 步骤5：保存中间速度场

```cpp
mid_u_x_.swap(tmp_u_x_);
mid_u_y_.swap(tmp_u_y_);
mid_u_z_.swap(tmp_u_z_);
step_++;
```

仅保存一份中间速度场，供Reinit阶段使用。

---

## Reinit阶段详解

### `ReinitAsync(float _dt, cudaStream_t _stream)` 流程  
**所在文件**：声明 `src/ofm/ofm.h`，实现 `src/ofm/ofm.cu`

每帧执行一次，Flow Map积分仅用**单步**完成。

#### 阶段1：重置Flow Maps

```cpp
ResetForwardFlowMapAsync(_stream);
ResetBackwardFlowMapAsync(_stream);
```

#### 阶段2：计算后向Flow Map（单步积分）

```cpp
RKAxisAsync(*psi_x_, *T_x_, tile_dim_, x_tile_dim, *mid_u_x_, *mid_u_y_, *mid_u_z_, grid_origin_, dx_, _dt, _stream);
RKAxisAsync(*psi_y_, *T_y_, ...);
RKAxisAsync(*psi_z_, *T_z_, ...);
```

**关键点**：
- **单次调用**：仅用`mid_u_`（Advance得到的中间速度）积分一步，时间步长`_dt`
- LFM需循环`reinit_every_`次，反向/正向遍历多个中间速度

#### 阶段3：计算前向Flow Map（单步积分）

```cpp
RKAxisAsync(*phi_x_, *F_x_, ..., -_dt, _stream);
RKAxisAsync(*phi_y_, *F_y_, ..., -_dt, _stream);
RKAxisAsync(*phi_z_, *F_z_, ..., -_dt, _stream);
```

使用负时间步`-_dt`实现正向积分。

#### 阶段4：Pullback（使用后向Flow Map）

```cpp
PullbackAxisAsync(*u_x_, tile_dim_, x_tile_dim, *init_u_x_, *init_u_y_, *init_u_z_, *psi_x_, *T_x_, ...);
PullbackAxisAsync(*u_y_, ...);
PullbackAxisAsync(*u_z_, ...);
```

#### 阶段5：BFECC 误差计算与修正

```cpp
PullbackAxisAsync(*err_u_x_, ..., *u_x_, *u_y_, *u_z_, *phi_x_, *F_x_, ...);  // 前向Pullback
AddFieldsAsync(*err_u_x_, x_tile_dim, *err_u_x_, *init_u_x_, -1.0f, _stream); // err = u(phi) - init_u
PullbackAxisAsync(*init_u_x_, ..., *err_u_x_, *err_u_y_, *err_u_z_, *psi_x_, *T_x_, ...); // 误差Pullback
AddFieldsAsync(*tmp_u_x_, x_tile_dim, *u_x_, *init_u_x_, -0.5f, _stream);     // u_corrected = u - 0.5*err
```

与LFM的BFECC流程一致。

#### 阶段6：BFECC Clamping（可选）

```cpp
if (use_bfecc_clamp_) {
    BfeccClampAsync(*tmp_u_x_, x_tile_dim, x_max_ijk, *u_x_, _stream);
    ...
}
```

#### 阶段7：最终投影

```cpp
ProjectAsync(_stream);
```

#### 阶段8：更新初始速度场

```cpp
init_u_x_.swap(tmp_u_x_);
init_u_y_.swap(tmp_u_y_);
init_u_z_.swap(tmp_u_z_);
```

**说明**：OFM不包含烟雾场更新逻辑。

---

## 动态边界处理

### `UpdateBoundary(cudaStream_t _stream)` 流程  
**所在文件**：`src/ofm/ofm.cu`

当`use_dynamic_solid_`为真时，每帧在Advance之前调用。

#### 步骤1：从体素纹理设置边界

```cpp
SetBcBySurfaceAsync(*is_bc_x_, *is_bc_y_, *is_bc_z_, *bc_val_x_, *bc_val_y_, *bc_val_z_,
    tile_dim_, voxel_tex_, velocity_tex_, voxelized_velocity_scaler_, _stream);
```

- `voxel_tex_`：体素化固体（0/1 标记）
- `velocity_tex_`：固体表面速度（R32G32B32A32）
- 固体内部及表面的网格点被设为边界，速度取自`velocity_tex_`

#### 步骤2：更新Poisson系数矩阵

```cpp
SetCoefByIsBcAsync(..., *is_bc_x_, *is_bc_y_, *is_bc_z_, _stream);
```

#### 步骤3：重建投影矩阵

```cpp
amgpcg_.BuildAsync(6.0f, -1.0f, _stream);
```

因边界每帧可能变化，需重建AMGPCG矩阵。

---

## 关键函数说明

### 1. `RKAxisAsync()` - Flow Map 积分

**功能**：使用**TVDRK3**（TVD Runge-Kutta 3阶）积分Flow Map

**输入**：
- `psi_axis`, `T_axis`：Flow Map的位置和切向量（输入输出）
- `u_x`, `u_y`, `u_z`：速度场（OFM中为`mid_u_`）
- `dt`：时间步长（可为正或负）

**实现**：`ofm_util.cu` 中调用 `TVDRK3AxisKernel`，与LFM的RK2/RK4不同。

### 2. `SetBcBySurfaceAsync()` - 从体素纹理设置边界

**功能**：根据体素纹理和速度纹理设置边界条件

**输入**：
- `voxel_surface`：体素化固体纹理（uint8）
- `velocity_surface`：固体表面速度纹理（float4）
- `vel_scaler`：速度缩放

**算法**：遍历网格，若体素为固体则标记为边界并写入对应速度。

**所在文件**：`src/ofm/ofm_util.cu`

### 3. 其他工具函数

- `PullbackAxisAsync`、`AdvectN2X/Y/ZAsync`、`ProjectAsync`、`CalcDivAsync`、`ApplyPressureAsync` 等与LFM逻辑一致，接口略有差异（如显式传入`tile_dim`、`grid_origin`等）。

---

## 函数所在文件对照表

| 函数名 | 声明 | 定义/实现 |
|--------|------|-----------|
| `InitOFMAsync` | `src/ofm/ofm_init.h` | `src/ofm/ofm_init.cu` |
| `AdvanceAsync` | `src/ofm/ofm.h` | `src/ofm/ofm.cu` |
| `ReinitAsync` | `src/ofm/ofm.h` | `src/ofm/ofm.cu` |
| `UpdateBoundary` | `src/ofm/ofm.h` | `src/ofm/ofm.cu` |
| `ResetForwardFlowMapAsync` | `src/ofm/ofm.h` | `src/ofm/ofm.cu` |
| `ResetBackwardFlowMapAsync` | `src/ofm/ofm.h` | `src/ofm/ofm.cu` |
| `ProjectAsync` | `src/ofm/ofm.h` | `src/ofm/ofm.cu` |
| `RKAxisAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `PullbackAxisAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `AdvectN2X/Y/ZAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `SetBcBySurfaceAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `SetBcByPhiAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `SetInletAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `SetBcAxisAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `CalcDivAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `ApplyPressureAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `SetWallBcAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `SetCoefByIsBcAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `AddFieldsAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `BfeccClampAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `GetCenteralVecAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `GetVorNormAsync` | `src/ofm/ofm_util.h` | `src/ofm/ofm_util.cu` |
| `PhysicsEngineUser::step` | `proj/dynamic_obstacle/physics.h` | `proj/dynamic_obstacle/physics.cu` |

---

## 算法流程图总结

```
初始化
  ↓
主循环开始
  ↓
┌─────────────────────────────────┐
│ 渲染数据准备                      │
│ - 计算中心速度                    │
│ - 计算涡度                        │
└─────────────────────────────────┘
  ↓
┌─────────────────────────────────┐
│ 动态边界更新（若启用）             │
│ - SetBcBySurfaceAsync            │
│ - SetCoefByIsBcAsync             │
│ - amgpcg_.BuildAsync             │
└─────────────────────────────────┘
  ↓
┌─────────────────────────────────┐
│ Advance阶段（每帧一次）            │
│ 1. Advection（mid_dt=0.5*dt）    │
│ 2. 设置入口边界条件               │
│ 3. 压力投影                       │
│ 4. 保存到 mid_u_                 │
└─────────────────────────────────┘
  ↓
┌─────────────────────────────────┐
│ Reinit阶段（每帧一次）            │
│ 1. 重置Flow Maps                 │
│ 2. 后向Flow Map（单步，mid_u）   │
│ 3. 前向Flow Map（单步，-dt）      │
│ 4. Pullback（后向）               │
│ 5. BFECC 误差计算与修正           │
│ 6. BFECC Clamping（可选）        │
│ 7. 最终投影                      │
│ 8. 更新 init_u_                  │
└─────────────────────────────────┘
  ↓
继续主循环
```

---

## 参考文献

- Sun et al., "Leapfrog Flow Maps for Real-Time Fluid Simulation", ACM TOG 2025
- Yutong Sun, "One-Step Flow Maps for Real-time Fluid Simulation with Dynamic Boundaries", Georgia Tech MSCS Thesis
- BFECC: Back and Forth Error Compensation and Correction
- MAC Grid: Marker-And-Cell staggered grid
