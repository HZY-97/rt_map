#!/bin/bash

# ============================================================
# rt_map 一键执行脚本
# 修改下面的参数即可，然后运行: bash run_rt_map.sh
# ============================================================

# 输入 PCD 文件路径
INPUT_FILE="/home/siasun/Documents/ori_trans/19-20_rt.pcd"

# 输出 PCD 文件路径
OUTPUT_FILE="/home/siasun/Documents/ori_trans/19-20_rt_1.pcd"

# 是否进行降采样 (true/false)
ENABLE_DOWNSAMPLE=false

# 降采样 leaf size（仅在 ENABLE_DOWNSAMPLE=true 时生效）
LEAF_SIZE=0.2

# 平移参数 (meters)
TX=0.0
TY=0.0
TZ=0.0

# 旋转参数 (degrees)
RX=0.0
RY=0.0
RZ=0.5

# ============================================================
# 下方为执行逻辑，无需修改
# ============================================================

set -e

EXECUTABLE="$(dirname "$0")/build/rt_map"

if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: 找不到可执行文件 $EXECUTABLE"
    echo "请先编译项目"
    exit 1
fi

if [ ! -f "$INPUT_FILE" ]; then
    echo "Error: 输入文件不存在: $INPUT_FILE"
    exit 1
fi

CMD=("$EXECUTABLE" "-i" "$INPUT_FILE" "-o" "$OUTPUT_FILE")

if [ "$ENABLE_DOWNSAMPLE" = true ]; then
    CMD+=("-d" "$LEAF_SIZE")
fi

CMD+=("-tx" "$TX" "-ty" "$TY" "-tz" "$TZ" "-rx" "$RX" "-ry" "$RY" "-rz" "$RZ")

echo "============================================"
echo "rt_map 一键执行脚本"
echo "============================================"
echo "输入文件:   $INPUT_FILE"
echo "输出文件:   $OUTPUT_FILE"
echo "降采样:     $ENABLE_DOWNSAMPLE"
if [ "$ENABLE_DOWNSAMPLE" = true ]; then
    echo "Leaf size:  $LEAF_SIZE"
fi
echo "平移 (tx ty tz): $TX $TY $TZ"
echo "旋转 (rx ry rz): $RX $RY $RZ"
echo "============================================"
echo "执行命令: ${CMD[*]}"
echo "============================================"

"${CMD[@]}"

echo "============================================"
echo "执行完成!"
echo "============================================"