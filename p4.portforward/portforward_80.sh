#!/bin/bash
############################################################
#  端口转发：把本机 10.88.120.142 上的端口转发到 RM65 机械臂
#
#  当前转发：
#      10.88.120.142:80   → 192.168.1.18:80    （web 页面，thttpd）
#      10.88.120.142:8060 → 192.168.1.18:8060  （WebSocket 实时通讯）
#      10.88.120.142:8090 → 192.168.1.18:8090  （登录/HTTP API 服务）
#
#  ⚠ 三个端口缺一不可，是同一套前端的三条通道：
#     · 80   ：静态页面
#     · 8090 ：HTTP API（登录、读写参数），login 时 JS 拼 location+":8090"
#     · 8060 ：WebSocket 实时状态推送，操作界面 JS 拼 baseUrl+":8060"
#     只转发部分端口会表现为：登录页能开/能登录，但操作中跳回登录或无响应。
#
#  实现：iptables DNAT（目的改写）+ MASQUERADE（源改写）
#        + FORWARD 放行。内核级转发，无需常驻进程。
#
#  用法（需要 root）:
#      sudo bash portforward_80.sh apply     # 启用（所有端口）
#      sudo bash portforward_80.sh remove    # 停用
#      sudo bash portforward_80.sh status    # 查看规则
#
#  持久化（重启后规则默认丢失，见文件末尾说明）
############################################################

# ──── 参数（按需修改）────
EXT_IP="10.88.120.142"   # 监听侧 IP（本机无线网卡）
TGT_IP="192.168.1.18"    # 目标 IP（机械臂）
# 需要转发的端口列表（源端口 = 目标端口）。增删端口只需改这里。
PORTS=(80 8060 8080 8090)
MARK="pf_rm65"           # 规则注释标记前缀（用于幂等增删）

# ──── root 检查 ────
if [ "$(id -u)" -ne 0 ]; then
  echo "✗ 需要 root 权限，请用 sudo 执行：sudo bash $0 ${1:-}"
  exit 1
fi

# ──── 清理本脚本旧规则（幂等：重复 apply 不会叠加）────
# 用 MARK 前缀匹配，删掉所有带该标记的规则
_clean() {
  iptables-save 2>/dev/null | grep -v "$MARK" | iptables-restore 2>/dev/null || true
}

# ──── 为单个端口添加规则 ────
_add_port() {
  local port="$1"
  local m="$MARK _$port"
  # 1) DNAT：外部访问（PREROUTING）+ 本机自身访问（OUTPUT）
  iptables -t nat -A PREROUTING  -d "$EXT_IP" -p tcp --dport "$port" \
           -j DNAT --to-destination "$TGT_IP:$port" -m comment --comment "$m"
  iptables -t nat -A OUTPUT      -d "$EXT_IP" -p tcp --dport "$port" \
           -j DNAT --to-destination "$TGT_IP:$port" -m comment --comment "$m"
  # 2) SNAT（MASQUERADE）：源改成出口网卡 IP，否则回包走错路
  iptables -t nat -A POSTROUTING -d "$TGT_IP" -p tcp --dport "$port" \
           -j MASQUERADE -m comment --comment "$m"
  # 3) FORWARD 放行（双向，防默认 DROP 拦截）
  iptables -A FORWARD -p tcp -d "$TGT_IP" --dport "$port" -j ACCEPT -m comment --comment "$m"
  iptables -A FORWARD -p tcp -s "$TGT_IP" --sport "$port" -j ACCEPT -m comment --comment "$m"
}

# ──── 启用 ────
apply() {
  _clean
  sysctl -w net.ipv4.ip_forward=1 >/dev/null
  for p in "${PORTS[@]}"; do
    _add_port "$p"
    echo "  ✓ $EXT_IP:$p  →  $TGT_IP:$p"
  done
  echo "✓ 端口转发已启用（共 ${#PORTS[@]} 个端口）"
}

# ──── 停用 ────
remove() {
  _clean
  echo "✓ 端口转发已移除（标记 $MARK）"
}

# ──── 查看状态 ────
status() {
  echo "=== nat 表规则 ==="
  iptables -t nat -S 2>/dev/null | grep "$MARK" || echo "  (无)"
  echo "=== filter/FORWARD 规则 ==="
  iptables -S 2>/dev/null | grep "$MARK" || echo "  (无)"
  echo "=== ip_forward ==="
  sysctl net.ipv4.ip_forward
}

case "${1:-}" in
  apply)  apply ;;
  remove) remove ;;
  status) status ;;
  *) echo "用法: sudo bash $0 {apply|remove|status}"; exit 1 ;;
esac
#
# ──── 持久化（重启后规则会丢失，如需开机自动生效）────
#
#   sudo apt install -y iptables-persistent
#   sudo bash portforward_80.sh apply
#   sudo netfilter-persistent save    # 保存到 /etc/iptables/rules.v4
#
# 验证（无需 root）：
#   curl -s -o /dev/null -w "%{http_code}\n" http://10.88.120.142:80     # 200
#   curl -s http://10.88.120.142:8090/                                   # JSON 响应
#   curl -s -o /dev/null -w "%{http_code}\n" \                          # 101 = WS 就绪
#     -H "Connection: Upgrade" -H "Upgrade: websocket" \
#     -H "Sec-WebSocket-Version: 13" -H "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==" \
#     http://10.88.120.142:8060/
