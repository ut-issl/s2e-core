#!/bin/bash

# 環境操作用ユーティリティ

function build {
    echo "Dockerイメージを作成します"
    # docker build -t issl:latest .
    cd ..
    docker build -f Docker_Ubuntu/Dockerfile -t issl:latest .
    cd ./Docker_Ubuntu
}

function enter {
    echo "コンテナ内のbashにアタッチします"
    winpty docker run -it --rm -p 2222:22 -v /$(realpath ../../../../work):/home/s2e/work issl:latest /bin/bash
}

function run_core {
    echo "s2e-coreのみをシェアするイメージからコンテナを作成します"
    docker run -it -d --cap-add=SYS_PTRACE --security-opt="seccomp=unconfined" -v /$(realpath ../../../s2e-core):/home/s2e/work/s2e-core --name issl-1 -p 2222:22 issl:latest
}

function run_with_work {
    echo "workディレクトリをシェアするイメージからコンテナを作成します"
    docker run -it -d --cap-add=SYS_PTRACE --security-opt="seccomp=unconfined" -v /$(realpath ../../../../work):/home/s2e/work --name issl-1 -p 2222:22 issl:latest
}

subcommand="$1"
shift

case $subcommand in
    build)
        build
        ;;
    run_core)
        run_core
        ;;
    enter)
        enter
        ;;
    *)
        echo "default"
        ;;
esac
