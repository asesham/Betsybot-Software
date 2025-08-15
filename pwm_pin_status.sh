#!/bin/bash




for chip in /sys/class/pwm/pwmchip*; do
    echo "=== $(basename $chip) ==="
    # Find all exported PWM channels
    for pwm in $chip/pwm*; do
        if [[ -d "$pwm" ]]; then
            chan=$(basename $pwm)
            enable=$(cat $pwm/enable 2>/dev/null)
            duty=$(cat $pwm/duty_cycle 2>/dev/null)
            period=$(cat $pwm/period 2>/dev/null)
            if [[ -n "$period" && "$period" -ne 0 ]]; then
                freq=$(echo "scale=2; 1000000000 / $period" | bc)
                duty_pct=$(echo "scale=2; ($duty / $period) * 100" | bc)
            else
                freq="N/A"
                duty_pct="N/A"
            fi
            echo "$chan: enable=$enable, freq=${freq}Hz, duty=${duty_pct}%"
        fi
    done
done
