# 创建目标目录（如果不存在）
HOOK_DIR=".git/hooks"
mkdir -p "$HOOK_DIR"

# 写入钩子内容
cat > "$HOOK_DIR/commit-msg" <<'EOF'
#!/bin/bash

# 定义颜色常量
RED="\033[31m"
BLUE="\033[34m"
GREEN="\033[32m"
RESET="\033[0m"  # 重置颜色

# 读取提交信息
commit_message=$(cat "$1")

# 定义正则表达式（简化为只需类型前缀）
regex="^(feat|fix|docs|style|refactor|test|chore|perf|revert):[[:space:]]+"

if ! [[ $commit_message =~ $regex ]]; then
  echo -e "${RED}提交信息不符合规范!${RESET}"
  echo -e "${BLUE}规范: <类型>: <描述>${RESET}"
  echo -e "${BLUE}类型说明:${RESET}"
  echo -e "${BLUE}feat      新功能${RESET}"
  echo -e "${BLUE}fix       bug修复${RESET}"
  echo -e "${BLUE}docs      文档变更${RESET}"
  echo -e "${BLUE}style     格式调整（空格/分号等，不影响代码）${RESET}"
  echo -e "${BLUE}refactor  代码重构（非功能/非bug修改）${RESET}"
  echo -e "${BLUE}test      测试相关${RESET}"
  echo -e "${BLUE}chore     构建/工具/配置等杂项${RESET}"
  echo -e "${BLUE}perf      性能优化${RESET}"
  echo -e "${BLUE}revert    回退提交${RESET}"
  echo -e "──────────────────────────────"
  echo -e "示例:"
  echo -e "fix: 修复uart接丢包的bug"
  echo -e ""
  echo -e "温馨提示: 冒号后面必须有空格！使用英文冒号"
  echo -e ""
  exit 1
fi
EOF

# 设置可执行权限
chmod +x "$HOOK_DIR/commit-msg"

# 验证安装
if [ -x "$HOOK_DIR/commit-msg" ]; then
  echo -e "\033[32m✓ 钩子安装成功\033[0m"
  echo "路径：$HOOK_DIR/commit-msg"
else
  echo -e "\033[31m✗ 安装失败，请检查权限\033[0m"
  exit 1
fi