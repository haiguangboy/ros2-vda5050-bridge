# 使用说明

## 脚本执行方式

### ✅ 正确方式 (推荐)
```bash
# 使用bash执行（推荐）
bash check_dependencies.sh
bash build_and_test.sh
bash install_deps.sh

# 或者先添加执行权限，然后直接运行
chmod +x *.sh
./check_dependencies.sh
./build_and_test.sh
./install_deps.sh
```

### ❌ 避免的方式
```bash
# 不要使用sh，因为脚本使用了bash特性
sh check_dependencies.sh  # 可能出现语法错误
```

## 架构支持

本项目支持以下架构：
- **x86_64** (amd64) - Intel/AMD 64位处理器
- **ARM64** (aarch64) - ARM 64位处理器（如树莓派4、Apple M系列、NVIDIA Jetson等）

脚本会自动检测您的系统架构并使用相应的库文件路径。

## 故障排除

如果遇到问题，请按以下顺序排查：

1. **检查系统架构**
   ```bash
   dpkg --print-architecture
   ```

2. **运行完整的依赖检查**
   ```bash
   bash check_dependencies.sh
   ```

3. **如果mosquitto检测失败，运行专门的诊断脚本**
   ```bash
   bash test_mosquitto_detect.sh
   ```

4. **重新安装依赖**
   ```bash
   sudo apt install libmosquitto-dev
   ```

5. **清理并重新编译**
   ```bash
   rm -rf build install
   mkdir build && cd build
   cmake ..
   make -j$(nproc)
   make install
   ```

## 常见错误

### 错误1: "Syntax error: '(' unexpected"
**原因**: 使用了`sh`而不是`bash`
**解决**: 使用`bash script_name.sh`或`./script_name.sh`

### 错误2: "mosquitto库未找到"
**原因**: 系统架构不匹配或库未安装
**解决**: 运行`bash test_mosquitto_detect.sh`诊断

### 错误3: "paho-mqtt-cpp未安装"
**原因**: 使用了旧的编译脚本
**解决**: 使用更新后的`build_and_test.sh`，它已改用mosquitto