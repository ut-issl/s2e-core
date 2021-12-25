    FROM ubuntu:latest

    # ssh
    RUN apt-get update &&\
        apt-get install -y --no-install-recommends openssh-server && \
        apt-get install -y tzdata && \
        sed -ri 's/^PermitRootLogin\s+.*/PermitRootLogin yes/' /etc/ssh/sshd_config && \
        echo "root:password" | chpasswd
    # timezone setting
    ENV TZ=Asia/Tokyo
    
    # Tool
    RUN apt-get update --fix-missing && \
        apt-get install -y --no-install-recommends \
        vim \
        git \
        csh \
        g++ \
        ruby \
        gdb \
        snapd \
        curl \
        xauth

    RUN apt-get update && \
        apt-get install -y \
        cmake \
        gcc-multilib g++-multilib


    #
    # 一般ユーザ用設定
    #
    RUN apt-get update && apt-get -y install sudo
    # 各種環境変数を設定
    ENV USER s2e
    ENV HOME /home/${USER}
    ENV SHELL /bin/bash
    # ホストのGROUP_IとUIDを記述しておくと後で便利
    ENV GROUP 1002
    ENV UID 1002
    # グループを追加
    RUN groupadd -g ${GROUP} ${USER}
    # 一般ユーザアカウントを追加
    RUN useradd -m -s ${SHELL} -g ${GROUP} -u ${UID} ${USER}
    # 一般ユーザのパスワード設定
    RUN echo "${USER}:${USER}" | chpasswd
    RUN adduser ${USER} sudo
    # 日本語環境追加
    RUN echo 'export LANG=ja_JP.UTF-8' >> ${HOME}/.bashrc
    RUN echo 'export LANGUAGE=ja_JP:ja' >> ${HOME}/.bashrc

    # 22:SSH
    EXPOSE 22
    WORKDIR /root
    CMD service ssh start && /bin/bash

