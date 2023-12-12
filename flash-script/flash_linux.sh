#!/bin/bash
set -e

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

JLinkExe -device R7FA8D1BH -if SWD -speed 12000 downloadfw.jlink
