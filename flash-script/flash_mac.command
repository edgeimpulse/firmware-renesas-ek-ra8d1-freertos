cd -- "$(dirname "$BASH_SOURCE")"
JLinkExe -device R7FA8D1BH -if SWD -speed 12000 downloadfw.jlink
