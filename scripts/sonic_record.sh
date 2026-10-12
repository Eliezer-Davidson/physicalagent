#!/usr/bin/env bash
# Record the MuJoCo viewer window to frames, then encode to mp4 + a contact strip.
#
#   ./scripts/sonic_record.sh <window-id> <seconds> <outdir>
#
# Uses xwd per frame because on this box that is the only screenshot route that
# works: gnome-screenshot/import/scrot/grim are absent, `xwd -root` fails with
# BadMatch under Wayland, and the GNOME DBus portal returns AccessDenied to a
# non-interactive caller. An individual XWayland *client* window still holds
# pixels, so `xwd -id` succeeds where everything else fails. ffmpeg then reads
# the .xwd files directly — no ImageMagick on this machine.
set -uo pipefail

WIN="${1:?window id, e.g. 0x80052e}"
SECS="${2:-20}"
OUT="${3:-/tmp/sonic_sim/rec}"
# 2, not 4: xwd on a 1726x1121 window is expensive enough that 4 fps starved the
# simulator to ~5% of realtime on 2026-07-31, which destabilised the robot. The
# recorder must not change the thing it is recording.
FPS=2

mkdir -p "$OUT/frames"
rm -f "$OUT/frames"/*.xwd "$OUT/frames"/*.png 2>/dev/null

echo "[rec] capturing $WIN for ${SECS}s at ~${FPS}fps -> $OUT"
END=$(( $(date +%s) + SECS ))
i=0
while [ "$(date +%s)" -lt "$END" ]; do
    printf -v n "%04d" "$i"
    xwd -id "$WIN" -out "$OUT/frames/f$n.xwd" 2>/dev/null || true
    i=$((i+1))
    sleep "$(awk "BEGIN{print 1/$FPS}")"
done
echo "[rec] captured $i frames"

# xwd -> png
for f in "$OUT/frames"/*.xwd; do
    ffmpeg -y -loglevel error -i "$f" "${f%.xwd}.png" 2>/dev/null || true
done

ffmpeg -y -loglevel error -framerate $FPS -pattern_type glob \
    -i "$OUT/frames/*.png" -c:v libx264 -pix_fmt yuv420p -vf scale=960:-2 \
    "$OUT/run.mp4" 2>/dev/null && echo "[rec] video: $OUT/run.mp4"

# contact strip of 6 evenly spaced frames, downscaled, for quick inspection
mapfile -t pngs < <(ls "$OUT/frames"/*.png 2>/dev/null)
N=${#pngs[@]}
if [ "$N" -ge 6 ]; then
    sel=()
    for k in 0 1 2 3 4 5; do
        idx=$(( k * (N-1) / 5 ))
        sel+=("${pngs[$idx]}")
    done
    ffmpeg -y -loglevel error \
        -i "${sel[0]}" -i "${sel[1]}" -i "${sel[2]}" \
        -i "${sel[3]}" -i "${sel[4]}" -i "${sel[5]}" \
        -filter_complex "[0:v]scale=520:-1[a];[1:v]scale=520:-1[b];[2:v]scale=520:-1[c];\
[3:v]scale=520:-1[d];[4:v]scale=520:-1[e];[5:v]scale=520:-1[f];\
[a][b][c]hstack=inputs=3[top];[d][e][f]hstack=inputs=3[bot];[top][bot]vstack=inputs=2[out]" \
        -map "[out]" "$OUT/strip.png" 2>/dev/null && echo "[rec] strip: $OUT/strip.png"
fi
