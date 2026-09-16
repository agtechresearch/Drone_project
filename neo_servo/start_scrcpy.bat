@echo off
REM Galaxy Note 9 mirroring, window fixed at screen top-left (0, 0)
REM --window-x 0 --window-y 0  : place window at top-left
REM --window-title "NoteNine"  : must match phone.window_title in config.yaml
REM --stay-awake               : keep phone screen on while USB is connected
REM --video-bit-rate 16M       : reduce compression loss at native resolution
scrcpy --window-x 0 --window-y 0 --window-title "NoteNine" --stay-awake --video-bit-rate 16M
pause