# STM32CubeProgrammer 详细烧录指南

## 📋 准备工作

### 1. 确认文件位置
编译生成的文件位于：
```
build_infantry/NYUSH_Infantry.elf  (推荐使用)
build_infantry/NYUSH_Infantry.hex  (备用)
```

完整路径：
```
C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control\build_infantry\NYUSH_Infantry.elf
```

### 2. 硬件连接

#### 方式A：ST-Link（推荐）
- **连接线**：ST-Link调试器
- **连接接口**：STM32的SWD接口
  - **SWDIO** (SWD Data I/O) → ST-Link的SWDIO
  - **SWCLK** (SWD Clock) → ST-Link的SWCLK  
  - **GND** (Ground) → ST-Link的GND
  - **3.3V** (Power) → ST-Link的3.3V（可选，如果STM32已供电）
- **USB连接**：ST-Link的USB端连接到电脑

#### 方式B：USB DFU（如果STM32支持）
- 按住STM32的**BOOT0**按钮
- 按一下**RESET**按钮
- 松开**BOOT0**按钮
- STM32进入DFU模式（设备管理器显示为"STM32 BOOTLOADER"）

---

## 🔧 详细烧录步骤

### 步骤1：打开STM32CubeProgrammer

1. 启动 **STM32CubeProgrammer** 应用程序
2. 等待程序完全加载

### 步骤2：选择连接方式

#### 方式A：ST-Link连接

1. **选择连接类型**
   - 在左侧面板，点击 **"ST-LINK"** 图标
   - 或者从顶部菜单：`File` → `Connect` → `ST-LINK`

2. **配置连接设置**
   - **Port**: 选择 `SWD`（默认）
   - **Mode**: 选择 `Normal`（默认）
   - **Frequency**: 选择 `4.0 MHz`（默认，或根据你的ST-Link选择）

3. **刷新并连接**
   - 点击右上角的 **"Refresh"** 按钮 🔄
   - 等待识别ST-Link设备
   - 如果成功，会显示：
     ```
     ST-LINK SN: [序列号]
     Firmware version: [版本号]
     ```

4. **连接目标**
   - 点击 **"Connect"** 按钮
   - 等待连接成功（可能需要几秒钟）
   - 如果成功，会显示：
     ```
     Device ID: 0x413
     Device name: STM32F407xx
     Flash Size: 1024 KBytes
     Device CPU: ARM Cortex-M4
     ```

#### 方式B：USB DFU连接

1. **选择连接类型**
   - 在左侧面板，点击 **"USB"** 图标

2. **刷新并连接**
   - 点击 **"Refresh"** 按钮
   - 选择检测到的STM32 DFU设备
   - 点击 **"Connect"** 按钮

---

### 步骤3：选择烧录文件

1. **打开烧录界面**
   - 点击左侧的 **"Erasing & Programming"** 标签页
   - 或者从顶部菜单：`File` → `Open File`

2. **选择文件**
   - 在 **"File path"** 输入框右侧，点击 **"Browse"** 按钮
   - 浏览到文件位置：
     ```
     C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control\build_infantry\
     ```
   - 选择文件：**`NYUSH_Infantry.elf`**
   - 点击 **"Open"**

3. **确认文件信息**
   - 文件路径应该显示在输入框中
   - 文件大小应该显示（约112KB）

---

### 步骤4：配置烧录选项

在 **"Erasing & Programming"** 标签页中，配置以下选项：

#### 必需选项：

- ✅ **"Start address"**: 
  - **留空**（自动检测，通常是 `0x08000000`）
  - 或者手动输入：`0x08000000`

- ✅ **"Skip flash erase"**: 
  - **取消勾选**（需要擦除Flash以确保干净烧录）

- ✅ **"Verify programming"**: 
  - **勾选**（烧录后验证数据完整性）

- ✅ **"Run after programming"**: 
  - **勾选**（烧录后自动运行程序）

#### 可选选项：

- ⚪ **"Full chip erase"**: 
  - 如果遇到问题，可以勾选此选项完全擦除芯片
  - 通常不需要

- ⚪ **"Reset and run"**: 
  - 通常已包含在"Run after programming"中
  - 可以额外勾选以确保复位

---

### 步骤5：开始烧录

1. **检查所有设置**
   - 确认文件路径正确
   - 确认所有选项已正确配置
   - 确认STM32已连接

2. **开始烧录**
   - 点击 **"Start Programming"** 按钮
   - 或者按快捷键：`Ctrl + P`

3. **观察进度**
   - 底部状态栏会显示进度：
     ```
     Erasing memory...
     Downloading File...
     Verifying...
     ```
   - 进度条会显示百分比

4. **等待完成**
   - 烧录过程通常需要5-15秒
   - 完成后会显示：
     ```
     File download complete
     ```

---

### 步骤6：验证烧录结果

1. **检查状态信息**
   - 在底部日志区域，应该看到：
     ```
     File download complete
     Verification OK
     ```

2. **检查设备状态**
   - 如果勾选了"Run after programming"，STM32应该自动运行
   - LED应该开始闪烁（如果代码中有LED控制）
   - USB CDC设备应该被识别（如果代码中有USB CDC）

3. **查看内存使用情况**
   - 在 **"Memory & File editing"** 标签页
   - 可以看到Flash和RAM的使用情况：
     ```
     Flash: 112500 bytes used / 1048576 bytes total (10.73%)
     RAM: 37296 bytes used / 131072 bytes total (28.45%)
     ```

---

### 步骤7：断开连接

1. **断开连接**
   - 点击 **"Disconnect"** 按钮
   - 或者从顶部菜单：`File` → `Disconnect`

2. **断开硬件**
   - 拔掉ST-Link的USB线（如果不再需要调试）
   - 或者保持连接以便后续调试

---

## ⚠️ 常见问题排查

### 问题1：找不到ST-Link

**症状**：
- 点击"Refresh"后没有显示ST-Link设备
- 显示"ST-LINK not found"

**解决方法**：
1. **检查驱动**
   - 打开设备管理器（Win+X → 设备管理器）
   - 查看"通用串行总线控制器"或"其他设备"
   - 如果看到"未知设备"或黄色感叹号，需要安装驱动

2. **安装ST-Link驱动**
   - 方法A：从ST官网下载ST-Link驱动
   - 方法B：STM32CubeProgrammer安装包中包含驱动
   - 方法C：使用STM32 ST-Link Utility安装驱动

3. **检查USB连接**
   - 尝试不同的USB端口
   - 使用USB 2.0端口（而不是USB 3.0）
   - 检查USB线是否完好

4. **重启STM32CubeProgrammer**
   - 完全关闭程序
   - 重新打开

---

### 问题2：连接失败

**症状**：
- 点击"Connect"后显示连接失败
- 显示"Connection failed"或"Target not found"

**解决方法**：
1. **检查硬件连接**
   - SWDIO、SWCLK、GND是否正确连接
   - 连接线是否松动
   - 尝试重新插拔连接线

2. **检查STM32供电**
   - STM32必须上电（3.3V）
   - 如果使用ST-Link供电，确保3.3V线已连接
   - 如果STM32独立供电，确保电源正常

3. **检查BOOT引脚**
   - BOOT0应该接地（0V）用于正常启动
   - 如果BOOT0接高电平，STM32会进入系统存储器模式

4. **尝试降低频率**
   - 在连接设置中，将频率从4.0MHz降低到1.0MHz
   - 某些ST-Link或STM32可能不支持高速

5. **检查复位引脚**
   - 确保RESET引脚没有被拉低
   - 尝试手动按一下RESET按钮

---

### 问题3：烧录失败

**症状**：
- 显示"Programming failed"
- 显示"Verification failed"
- 显示"Flash erase failed"

**解决方法**：
1. **完全擦除Flash**
   - 勾选 **"Full chip erase"** 选项
   - 先点击 **"Full chip erase"** 按钮
   - 等待擦除完成
   - 然后再尝试烧录

2. **检查文件**
   - 确认文件路径正确
   - 确认文件没有损坏
   - 尝试重新编译生成新的.elf文件

3. **检查Flash保护**
   - 在 **"Option Bytes"** 标签页
   - 检查写保护（WRP）和读保护（RDP）设置
   - 如果被保护，需要先解除保护

4. **降低烧录速度**
   - 在连接设置中降低频率
   - 某些情况下高速烧录可能不稳定

5. **检查STM32型号**
   - 确认STM32型号匹配（STM32F407xx）
   - 如果型号不匹配，需要修改代码重新编译

---

### 问题4：烧录后不运行

**症状**：
- 烧录成功，但STM32没有运行
- LED不闪烁，USB设备不被识别

**解决方法**：
1. **检查BOOT0引脚**
   - BOOT0必须接地（0V）才能从Flash启动
   - 如果BOOT0接高电平，STM32不会运行用户程序

2. **手动复位**
   - 按一下STM32的RESET按钮
   - 或者断开并重新连接电源

3. **检查"Run after programming"**
   - 确认已勾选此选项
   - 如果没有勾选，需要手动复位

4. **检查代码**
   - 确认代码编译成功
   - 检查是否有初始化错误
   - 查看是否有硬件依赖（如CAN总线、电机等）

5. **使用调试器**
   - 连接ST-Link调试器
   - 使用STM32CubeIDE或其他调试工具
   - 查看程序是否在运行，卡在哪里

---

### 问题5：USB CDC不被识别

**症状**：
- 烧录成功，但电脑不识别USB CDC设备
- 设备管理器中看不到COM口

**解决方法**：
1. **检查USB连接**
   - 确认USB线连接到STM32的USB接口（不是ST-Link）
   - 尝试不同的USB端口
   - 检查USB线是否支持数据传输（不只是充电）

2. **检查代码**
   - 确认代码中已初始化USB CDC
   - 检查USB配置是否正确

3. **安装USB驱动**
   - Windows通常自动识别USB CDC设备
   - 如果没有，可能需要安装STM32虚拟COM端口驱动

4. **检查设备管理器**
   - 打开设备管理器
   - 查看"端口(COM和LPT)"
   - 如果看到"STM32 Virtual COM Port"，说明已识别

---

## 📝 快速参考命令

### 生成hex文件（如果需要）
```powershell
cd build_infantry
arm-none-eabi-objcopy -O ihex NYUSH_Infantry.elf NYUSH_Infantry.hex
```

### 查看文件信息
```powershell
arm-none-eabi-size NYUSH_Infantry.elf
```

### 查看内存映射
```powershell
arm-none-eabi-objdump -h NYUSH_Infantry.elf
```

---

## ✅ 完整工作流程总结

```powershell
# 1. 编译代码
cmake --preset Debug -DROBOT_TYPE=infantry_standard -B build_infantry
cmake --build build_infantry

# 2. （可选）生成hex文件
arm-none-eabi-objcopy -O ihex build_infantry/NYUSH_Infantry.elf build_infantry/NYUSH_Infantry.hex

# 3. 使用STM32CubeProgrammer烧录
#    - 打开STM32CubeProgrammer
#    - 选择ST-LINK → Refresh → Connect
#    - 选择Erasing & Programming标签
#    - Browse选择NYUSH_Infantry.elf文件
#    - 勾选Verify和Run after programming
#    - 点击Start Programming
#    - 等待完成，点击Disconnect

# 4. 测试
#    - 连接USB到STM32（如果使用USB CDC）
#    - 运行Python脚本测试通信
python3 script/cmd_vel_forwarder.py --port COM3 --ros2
```

---

## 🎯 关键要点

1. **文件位置**：`build_infantry/NYUSH_Infantry.elf`
2. **连接方式**：ST-LINK（SWD接口）
3. **必须勾选**：Verify programming、Run after programming
4. **BOOT0**：必须接地（0V）才能运行
5. **验证**：烧录后LED应该闪烁，USB CDC应该被识别

---

## 📞 需要帮助？

如果遇到问题：
1. 查看STM32CubeProgrammer的日志输出
2. 检查硬件连接
3. 尝试完全擦除Flash后重新烧录
4. 查看本文档的"常见问题排查"部分
