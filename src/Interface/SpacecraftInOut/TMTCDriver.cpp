#include "TMTCDriver.h"

typedef cli::array<Byte> arr;

TMTCDriver::TMTCDriver(int port_id) : kPortId(port_id) { Initialize(); }

void TMTCDriver::Initialize() {
  try {
    ChannelFactory<ITCTMChannel ^> ^ pipeFactory =
        gcnew ChannelFactory<ITCTMChannel ^>(gcnew NetNamedPipeBinding(), gcnew EndpointAddress("net.pipe://localhost/TMTC_SILS"));
    tctm_if_ = pipeFactory->CreateChannel();
    tctm_if_->Cmd_to_SILS();  // 試し送信 つながってなかったらこれで例外吐く
  } catch (Exception ^) {
    // Console::WriteLine("Failed to connect GSTOS interface...");
    tctm_if_ = nullptr;
    return;
  }

  Console::WriteLine("Successfully connected to GSTOS interface.");
  SCIDriver::ConnectPort(kPortId);
}

void TMTCDriver::ReceiveCommand() {
  if (tctm_if_ == nullptr) return;
  arr ^ cmdbuf;
  try {
    cmdbuf = tctm_if_->Cmd_to_SILS();
  } catch (Exception ^) {
    // 1回でも失敗したら、以後GSTOSとの通信を試みない（それで良いの？）
    tctm_if_ = nullptr;
    return;
  }
  if (cmdbuf == nullptr) return;

  int cnt = cmdbuf->Length;
  if (cnt < 0) return;
  pin_ptr<Byte> buf = &cmdbuf[0];
  SCIDriver::SendToSC(kPortId, (Byte*)buf, 0, cnt);
}

void TMTCDriver::SendTelemetryIfAny() {
  if (tctm_if_ == nullptr) return;
  Byte buf[1024];
  int cnt = SCIDriver::ReceiveFromSC(kPortId, buf, 0, 1024);
  if (cnt <= 0) return;

  arr ^ tlmbuf = gcnew arr(cnt);
  for (int i = 0; i < cnt; i++) {
    tlmbuf[i] = buf[i];
  }

  try {
    tctm_if_->Tlm_to_GSTOS(tlmbuf);
  } catch (Exception ^) {
    tctm_if_ = nullptr;
    return;
  }
}