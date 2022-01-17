#pragma once
//#include "../../../stdafx.h"
#include "SCIDriver.h"
#include "Utils/ITCTMChannel.h"

using namespace System;
using namespace System::ServiceModel;
using namespace System::ServiceModel::Channels;

// SILS_GSTOS_IFとプロセス間通信を行い、
// ・テレメの送信（搭載ソフトが生成していれば）
// ・コマンドの受信（GSTOSから送られてきていれば）
// を行うクラス
// IPCのためにマネージドなクラスにしたが……なんとかなった
// gcrootを使うことでネイティブなクラスでの利用も可能
ref class TMTCDriver {
public:
  TMTCDriver(int port_id);
  void ReceiveCommand();
  void SendTelemetryIfAny();

private:
  ITCTMChannel ^ tctm_if_;
  const unsigned char kPortId = 7;

  void Initialize();
};
