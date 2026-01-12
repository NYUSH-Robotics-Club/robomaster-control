# STM32烧录指南

## 📋 使用STM32CubeProgrammer烧录

### 方法1：ST-Link（推荐）

#### 准备工作
1. **硬件连接**
   - 将ST-Link调试器连接到STM32的SWD接口（SWDIO, SWCLK, GND, 3.3V）
   - 将ST-Link的USB端连接到电脑

2. **确认文件位置**
   - 编译生成的`.elf`文件：`build_infantry/NYUSH_Infantry.elf`
   - 或者可以生成`.hex`文件（见下方）

#### 烧录步骤

**步骤1：打开STM32CubeProgrammer**
- 启动STM32CubeProgrammer应用程序

**步骤2：选择连接方式**
1. 在左侧选择 **"ST-LINK"**
2. 点击右上角的 **"Refresh"** 按钮，等待识别ST-Link
3. 确认显示：
   - `ST-LINK SN: [序列号]`
   - `Target: STM32F407xx`
   - `Voltage: 3.3V`

**步骤3：连接目标**
1. 点击 **"Connect"** 按钮
2. 如果成功，会显示：
   - `Device ID: 0x[ID]`
   - `Flash Size: 1024 KBytes`
   - `Device name: STM32F407xx`

**步骤4：选择文件**
1. 点击左侧的 **"Erasing & Programming"** 标签
2. 在 **"File path"** 中点击 **"Browse"**
3. 选择文件：
   ```
   build_infantry/NYUSH_Infantry.elf
   ```
   或者如果生成了`.hex`文件：
   ```
   build_infantry/NYUSH_Infantry.hex
   ```

**步骤5：设置烧录选项**
- ✅ **"Start address"**: 留空（自动检测）
- ✅ **"Skip flash erase"**: 取消勾选（需要擦除）
- ✅ **"Verify programming"**: 勾选（验证烧录）
- ✅ **"Run after programming"**: 勾选（烧录后自动运行）

**步骤6：开始烧录**
1. 点击 **"Start Programming"** 按钮
2. 等待进度条完成
3. 看到 **"File download complete"** 表示成功

**步骤7：断开连接**
- 点击 **"Disconnect"** 按钮
- 拔掉ST-Link

---

### 方法2：USB DFU（如果STM32支持DFU模式）

#### 准备工作
1. **进入DFU模式**
   - 按住STM32的BOOT0按钮
   - 按一下RESET按钮
   - 松开BOOT0按钮
   - STM32会进入DFU模式（在设备管理器中显示为"STM32 BOOTLOADER"）

2. **生成DFU文件（可选）**
   ```powershell
   # 如果需要.dfu文件，可以使用dfu-util工具转换
   ```

#### 烧录步骤

**步骤1：打开STM32CubeProgrammer**
- 启动STM32CubeProgrammer

**步骤2：选择连接方式**
1. 在左侧选择 **"USB"**
2. 点击 **"Refresh"** 按钮
3. 选择检测到的STM32 DFU设备

**步骤3：连接目标**
- 点击 **"Connect"** 按钮

**步骤4-7：** 同ST-Link方法（步骤4-7）

---

## 🔧 生成.hex文件（可选）

如果STM32CubeProgrammer不支持`.elf`文件，可以生成`.hex`文件：

```powershell
# 使用arm-none-eabi-objcopy转换
arm-none-eabi-objcopy -O ihex build_infantry/NYUSH_Infantry.elf build_infantry/NYUSH_Infantry.hex
```

或者生成`.bin`文件：

```powershell
arm-none-eabi-objcopy -O binary build_infantry/NYUSH_Infantry.elf build_infantry/NYUSH_Infantry.bin
```

---

## ⚠️ 常见问题

### 问题1：找不到ST-Link
- **检查**：ST-Link驱动是否安装
- **解决**：安装STM32 ST-Link Utility或STM32CubeProgrammer自带的驱动

### 问题2：连接失败
- **检查**：
  - ST-Link是否正确连接（SWDIO, SWCLK, GND）
  - STM32是否上电（3.3V）
  - 连接线是否松动
- **解决**：重新插拔连接线，检查硬件连接

### 问题3：烧录失败
- **检查**：
  - 文件路径是否正确
  - STM32 Flash是否被保护
- **解决**：
  - 尝试先擦除Flash：点击 **"Full chip erase"**
  - 检查STM32的写保护设置

### 问题4：烧录后不运行
- **检查**：
  - BOOT0引脚是否接地（正常启动模式）
  - 是否勾选了"Run after programming"
- **解决**：
  - 确保BOOT0=0（接地）
  - 手动按RESET按钮

---

## 📝 快速命令参考

### 生成hex文件
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

## ✅ 验证烧录成功

烧录成功后，你应该能看到：
1. STM32的LED开始闪烁（如果有LED代码）
2. USB CDC设备被识别（如果代码中有USB CDC）
3. 可以通过串口/USB看到STM32的输出（如果代码中有printf）

---

## 🔄 完整工作流程

```powershell
# 1. 编译
cmake --preset Debug -DROBOT_TYPE=infantry_standard -B build_infantry
cmake --build build_infantry

# 2. （可选）生成hex文件
arm-none-eabi-objcopy -O ihex build_infantry/NYUSH_Infantry.elf build_infantry/NYUSH_Infantry.hex

# 3. 使用STM32CubeProgrammer烧录
#    打开STM32CubeProgrammer -> ST-LINK -> Connect -> 选择.elf文件 -> Start Programming

# 4. 测试
#    运行Python脚本测试通信
python3 script/cmd_vel_forwarder.py --port COM3 --ros2
```
