#!/bin/bash

# 環境操作用ユーティリティ

function build {
    echo "Dockerイメージを作成します"
    docker build -t issl:latest .
}

function run {
    echo "イメージからコンテナを作成します"
    docker run -it -d --cap-add=SYS_PTRACE --security-opt="seccomp=unconfined" -v /$(realpath ../../../../work):/home/s2e/work --name issl-1 -p 2222:22 issl:latest
}

function enter {
    echo "コンテナ内のbashにアタッチします"
    winpty docker run -it --rm -p 2222:22 -v /$(realpath ../../../../work):/home/s2e/work issl:latest /bin/bash
}
subcommand="$1"
shift

case $subcommand in
    build)
        build
        ;;
    run)
        run
        ;;
    enter)
        enter
        ;;
    *)
        echo "default"
        ;;
esac
