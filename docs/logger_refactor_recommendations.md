# Logger 组件重构建议

## 一、现状分析

### basic_framework 的 logger 组件特点

1. **底层实现**：基于 SEGGER RTT（Real-Time Transfer）
   - 无需占用串口/USB资源
   - 通过调试器直接传输，实时性好
   - 支持多缓冲区，可同时输出到多个终端

2. **日志级别**：三级日志系统
   - `LOGINFO()` - 信息级别（绿色）
   - `LOGWARNING()` - 警告级别（黄色）
   - `LOGERROR()` - 错误级别（红色）

3. **功能特性**：
   - 支持颜色输出，便于区分日志级别
   - 编译时可通过 `DISABLE_LOG_SYSTEM` 宏完全禁用日志
   - 提供 `Float2Str()` 函数处理浮点数（RTT不支持 %f）
   - 提供 `LOG_CLEAR()` 清屏功能
   - 提供 `PrintLog()` 函数用于自定义输出

4. **架构位置**：位于 `bsp/log/` 目录，符合 BSP 层设计

### robomaster-control 的现状

1. **底层实现**：基于 USB CDC
   - 需要占用 USB 资源
   - 依赖 USB 设备初始化
   - 可能阻塞或丢失数据

2. **日志接口**：
   - 只有 `USB_CDC_Printf()` 函数
   - 没有日志级别区分
   - 没有统一的日志接口

3. **使用情况**：
   - `gimbal_controller.c` 中直接使用 `USB_CDC_Printf()` 输出调试信息
   - `printing.c` 中有一些诊断函数，但都是直接调用 USB CDC

4. **架构位置**：位于 `modules/debug_print/` 目录

## 二、重构建议

### 方案一：完全迁移到 SEGGER RTT（推荐）

#### 优点
- 与 basic_framework 保持一致，便于代码复用
- 不占用串口/USB资源，可同时使用 USB 做其他用途
- 实时性更好，通过调试器直接传输
- 支持多终端输出
- 性能开销更小

#### 缺点
- 需要添加 SEGGER RTT 中间件依赖
- 调试时必须连接调试器才能看到日志
- 需要配置调试器支持 RTT

#### 实施步骤

1. **添加 SEGGER RTT 依赖**
   ```
   Middlewares/Third_Party/SEGGER/RTT/
   ├── SEGGER_RTT.c
   ├── SEGGER_RTT.h
   ├── SEGGER_RTT_printf.c
   ├── SEGGER_RTT_ASM_ARMv7M.s
   └── Config/
       └── SEGGER_RTT_Conf.h
   ```

2. **创建 bsp/log 目录结构**
   ```
   bsp/log/
   ├── bsp_log.c
   ├── bsp_log.h
   └── bsp_log.md
   ```

3. **实现日志接口**
   - 复制 basic_framework 的 `bsp_log.h` 和 `bsp_log.c`
   - 根据项目需求调整配置

4. **更新 CMakeLists.txt**
   - 添加 SEGGER RTT 源文件
   - 添加 bsp/log 到包含路径
   - 添加 bsp_log.c 到编译列表

5. **替换现有日志调用**
   - 将 `USB_CDC_Printf()` 替换为 `LOGINFO()` / `LOGWARNING()` / `LOGERROR()`
   - 根据日志内容选择合适的级别

6. **初始化日志系统**
   - 在 `main.c` 的初始化阶段调用 `BSPLogInit()`

### 方案二：改进现有 USB CDC 日志系统（兼容方案）

如果项目必须使用 USB CDC 或暂时无法迁移到 RTT，可以改进现有系统：

#### 实施步骤

1. **重构 printing.h/c，添加日志级别**
   ```c
   // 定义日志级别
   #define LOG_LEVEL_NONE  0
   #define LOG_LEVEL_ERROR 1
   #define LOG_LEVEL_WARN  2
   #define LOG_LEVEL_INFO  3
   #define LOG_LEVEL_DEBUG 4
   
   // 编译时配置日志级别
   #ifndef LOG_LEVEL
   #define LOG_LEVEL LOG_LEVEL_INFO
   #endif
   
   // 日志宏定义
   #define LOGINFO(fmt, ...)  do { if (LOG_LEVEL >= LOG_LEVEL_INFO)  USB_CDC_Printf("[INFO] " fmt "\r\n", ##__VA_ARGS__); } while(0)
   #define LOGWARNING(fmt, ...) do { if (LOG_LEVEL >= LOG_LEVEL_WARN)  USB_CDC_Printf("[WARN] " fmt "\r\n", ##__VA_ARGS__); } while(0)
   #define LOGERROR(fmt, ...) do { if (LOG_LEVEL >= LOG_LEVEL_ERROR) USB_CDC_Printf("[ERROR] " fmt "\r\n", ##__VA_ARGS__); } while(0)
   #define LOGDEBUG(fmt, ...) do { if (LOG_LEVEL >= LOG_LEVEL_DEBUG) USB_CDC_Printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__); } while(0)
   ```

2. **添加编译时禁用功能**
   ```c
   #ifdef DISABLE_LOG_SYSTEM
   #define LOGINFO(...)
   #define LOGWARNING(...)
   #define LOGERROR(...)
   #define LOGDEBUG(...)
   #endif
   ```

3. **保持向后兼容**
   - 保留 `USB_CDC_Printf()` 函数供特殊用途使用
   - 逐步迁移现有代码到新的日志接口

### 方案三：双模式支持（最灵活）

同时支持 SEGGER RTT 和 USB CDC，通过编译选项选择：

```c
// bsp_log.h
#ifdef USE_SEGGER_RTT
    #include "SEGGER_RTT.h"
    #define LOG_OUTPUT(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)
#else
    #include "usbd_cdc_if.h"
    #define LOG_OUTPUT(fmt, ...) USB_CDC_Printf(fmt, ##__VA_ARGS__)
#endif
```

## 三、具体重构建议

### 1. 目录结构建议

**推荐采用方案一，目录结构如下：**
```
robomaster-control/
├── bsp/                    # 新增 BSP 层
│   └── log/
│       ├── bsp_log.c
│       ├── bsp_log.h
│       └── bsp_log.md
├── Middlewares/
│   └── Third_Party/
│       └── SEGGER/
│           └── RTT/        # 新增
│               ├── SEGGER_RTT.c
│               ├── SEGGER_RTT.h
│               ├── SEGGER_RTT_printf.c
│               ├── SEGGER_RTT_ASM_ARMv7M.s
│               └── Config/
│                   └── SEGGER_RTT_Conf.h
└── modules/
    └── debug_print/        # 保留，但功能迁移到 bsp/log
        ├── printing.c      # 保留诊断函数，但使用新的日志接口
        └── printing.h
```

### 2. 代码迁移示例

**迁移前（gimbal_controller.c）：**
```c
USB_CDC_Printf("YAW_CSV,%lu,%.2f,%.2f,%d,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f\r\n",
               timestamp, yaw->angle_target, current, ...);
```

**迁移后：**
```c
// 调试信息使用 LOGDEBUG
LOGDEBUG("YAW_CSV,%lu,%.2f,%.2f,%d,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f",
         timestamp, yaw->angle_target, current, ...);

// 或者对于重要的状态信息使用 LOGINFO
LOGINFO("Gimbal yaw target=%.2f current=%.2f", yaw->angle_target, current);
```

### 3. 初始化流程

在 `main.c` 中添加：
```c
#include "bsp_log.h"

int main(void) {
    // ... HAL 初始化 ...
    
    // 初始化日志系统（在系统初始化早期调用）
    BSPLogInit();
    
    LOGINFO("System initialized");
    
    // ... 其他初始化 ...
}
```

### 4. CMakeLists.txt 更新

```cmake
# 添加 SEGGER RTT 源文件
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.c
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_printf.c
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s
    bsp/log/bsp_log.c
)

# 添加包含路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/log
    Middlewares/Third_Party/SEGGER/RTT
    Middlewares/Third_Party/SEGGER/Config
)
```

### 5. 编译选项

在 CMakeLists.txt 或编译配置中添加：
```cmake
# Release 版本禁用日志
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        DISABLE_LOG_SYSTEM=1
    )
endif()
```

## 四、迁移优先级

### 高优先级
1. ✅ 创建 `bsp/log` 目录和基础文件
2. ✅ 添加 SEGGER RTT 依赖
3. ✅ 实现日志接口（LOGINFO/LOGWARNING/LOGERROR）
4. ✅ 在 main.c 中初始化日志系统

### 中优先级
5. ⚠️ 迁移 `gimbal_controller.c` 中的日志调用
6. ⚠️ 更新 `printing.c` 中的诊断函数使用新接口
7. ⚠️ 添加编译时禁用功能

### 低优先级
8. 📝 添加日志文档说明
9. 📝 考虑添加日志过滤功能（按模块/级别）
10. 📝 添加时间戳功能（如果需要）

## 五、注意事项

1. **浮点数处理**：如果使用 SEGGER RTT，注意 RTT 不支持 `%f` 格式化，需要使用 `Float2Str()` 函数转换

2. **调试器配置**：使用 RTT 时，需要确保调试器支持 RTT（J-Link、DAP-Link 等）

3. **性能考虑**：日志输出不应影响实时性，考虑使用非阻塞模式

4. **向后兼容**：如果保留 USB CDC 功能，确保不影响现有功能

5. **测试验证**：迁移后需要验证所有日志输出正常工作

## 六、参考资源

- basic_framework: `bsp/log/bsp_log.h` 和 `bsp_log.c`
- SEGGER RTT 官方文档
- basic_framework 的 `bsp_log.md` 使用说明

