#!/bin/bash
#
# monitor_resources.sh
# 
# Monitor CPU, Memory, and Network bandwidth on NUC/Jetson
#
# Usage:
#   ./monitor_resources.sh              # Monitor continuously
#   ./monitor_resources.sh --once       # Single snapshot
#   ./monitor_resources.sh --duration 30  # Monitor for 30 seconds
#

DURATION=0
INTERVAL=2
ONCE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --once)
            ONCE=true
            shift
            ;;
        --duration)
            DURATION=$2
            shift 2
            ;;
        --interval)
            INTERVAL=$2
            shift 2
            ;;
        *)
            shift
            ;;
    esac
done

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get network interface (first non-lo interface)
get_net_interface() {
    ip -o link show | awk -F': ' '{print $2}' | grep -v lo | head -1
}

NET_IF=$(get_net_interface)

# Get initial network stats
get_net_bytes() {
    local iface=$1
    local rx=$(cat /sys/class/net/$iface/statistics/rx_bytes 2>/dev/null || echo 0)
    local tx=$(cat /sys/class/net/$iface/statistics/tx_bytes 2>/dev/null || echo 0)
    echo "$rx $tx"
}

# Print header
print_header() {
    echo ""
    echo "========================================================================"
    echo -e " ${BLUE}SYSTEM RESOURCE MONITOR${NC}"
    echo "========================================================================"
    echo " Host: $(hostname)"
    echo " CPU:  $(lscpu | grep 'Model name' | cut -d: -f2 | xargs)"
    echo " Cores: $(nproc) cores"
    echo " Interface: $NET_IF"
    echo "========================================================================"
    echo ""
    printf "%-20s %-12s %-12s %-12s %-15s %-15s\n" \
        "TIME" "CPU%" "MEM%" "MEM_GB" "RX_MB/s" "TX_MB/s"
    echo "------------------------------------------------------------------------"
}

# Get stats
get_stats() {
    # CPU usage (average across all cores)
    local cpu=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)
    if [[ -z "$cpu" ]]; then
        cpu=$(top -bn1 | grep "%Cpu" | awk '{sum+=$2} END {print sum/NR}')
    fi
    
    # Memory
    local mem_info=$(free -m | grep Mem)
    local mem_total=$(echo $mem_info | awk '{print $2}')
    local mem_used=$(echo $mem_info | awk '{print $3}')
    local mem_percent=$(awk "BEGIN {printf \"%.1f\", $mem_used/$mem_total*100}")
    local mem_gb=$(awk "BEGIN {printf \"%.2f\", $mem_used/1024}")
    
    echo "$cpu $mem_percent $mem_gb"
}

# Calculate bandwidth
calc_bandwidth() {
    local prev_rx=$1
    local prev_tx=$2
    local curr_rx=$3
    local curr_tx=$4
    local interval=$5
    
    local rx_diff=$((curr_rx - prev_rx))
    local tx_diff=$((curr_tx - prev_tx))
    
    local rx_mbs=$(awk "BEGIN {printf \"%.2f\", $rx_diff/$interval/1024/1024}")
    local tx_mbs=$(awk "BEGIN {printf \"%.2f\", $tx_diff/$interval/1024/1024}")
    
    echo "$rx_mbs $tx_mbs"
}

# Main monitoring loop
monitor() {
    print_header
    
    local start_time=$(date +%s)
    local prev_bytes=$(get_net_bytes $NET_IF)
    local prev_rx=$(echo $prev_bytes | awk '{print $1}')
    local prev_tx=$(echo $prev_bytes | awk '{print $2}')
    
    # Track peak values
    local peak_cpu=0
    local peak_mem_gb=0
    local peak_rx=0
    local peak_tx=0
    local count=0
    local total_cpu=0
    local total_mem=0
    
    sleep $INTERVAL
    
    while true; do
        local now=$(date +"%H:%M:%S")
        local stats=$(get_stats)
        local cpu=$(echo $stats | awk '{print $1}')
        local mem_pct=$(echo $stats | awk '{print $2}')
        local mem_gb=$(echo $stats | awk '{print $3}')
        
        # Network
        local curr_bytes=$(get_net_bytes $NET_IF)
        local curr_rx=$(echo $curr_bytes | awk '{print $1}')
        local curr_tx=$(echo $curr_bytes | awk '{print $2}')
        local bw=$(calc_bandwidth $prev_rx $prev_tx $curr_rx $curr_tx $INTERVAL)
        local rx_mbs=$(echo $bw | awk '{print $1}')
        local tx_mbs=$(echo $bw | awk '{print $2}')
        
        prev_rx=$curr_rx
        prev_tx=$curr_tx
        
        # Update peaks
        peak_cpu=$(awk "BEGIN {print ($cpu > $peak_cpu) ? $cpu : $peak_cpu}")
        peak_mem_gb=$(awk "BEGIN {print ($mem_gb > $peak_mem_gb) ? $mem_gb : $peak_mem_gb}")
        peak_rx=$(awk "BEGIN {print ($rx_mbs > $peak_rx) ? $rx_mbs : $peak_rx}")
        peak_tx=$(awk "BEGIN {print ($tx_mbs > $peak_tx) ? $tx_mbs : $peak_tx}")
        
        # Accumulate for average
        total_cpu=$(awk "BEGIN {print $total_cpu + $cpu}")
        total_mem=$(awk "BEGIN {print $total_mem + $mem_gb}")
        count=$((count + 1))
        
        # Color coding for CPU
        local cpu_color=$GREEN
        if (( $(echo "$cpu > 80" | bc -l) )); then
            cpu_color=$RED
        elif (( $(echo "$cpu > 50" | bc -l) )); then
            cpu_color=$YELLOW
        fi
        
        printf "%-20s ${cpu_color}%-12s${NC} %-12s %-12s %-15s %-15s\n" \
            "$now" "${cpu}%" "${mem_pct}%" "${mem_gb}GB" "${rx_mbs}" "${tx_mbs}"
        
        if $ONCE; then
            break
        fi
        
        # Check duration
        if [[ $DURATION -gt 0 ]]; then
            local elapsed=$(($(date +%s) - start_time))
            if [[ $elapsed -ge $DURATION ]]; then
                break
            fi
        fi
        
        sleep $INTERVAL
    done
    
    # Print summary
    if [[ $count -gt 1 ]]; then
        local avg_cpu=$(awk "BEGIN {printf \"%.1f\", $total_cpu/$count}")
        local avg_mem=$(awk "BEGIN {printf \"%.2f\", $total_mem/$count}")
        local total_bw=$(awk "BEGIN {printf \"%.2f\", $peak_rx + $peak_tx}")
        
        echo ""
        echo "========================================================================"
        echo -e " ${BLUE}SUMMARY${NC}"
        echo "========================================================================"
        echo -e " CPU Peak:      ${RED}${peak_cpu}%${NC}"
        echo -e " CPU Average:   ${avg_cpu}%"
        echo -e " Memory Peak:   ${RED}${peak_mem_gb} GB${NC}"
        echo -e " Memory Avg:    ${avg_mem} GB"
        echo -e " RX Peak:       ${peak_rx} MB/s"
        echo -e " TX Peak:       ${peak_tx} MB/s"
        echo -e " Total BW Peak: ${RED}${total_bw} MB/s${NC}"
        echo "========================================================================"
        echo ""
        echo "For documentation:"
        echo "  CPU: ${peak_cpu}% peak, Memory: ${peak_mem_gb} GB, Bandwidth: ${total_bw} MB/s"
    fi
}

# Run
monitor
