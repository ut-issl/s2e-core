#include "HardwareMessage.h"

HardwareMessage::HardwareMessage(int port_id, bool enable,
                                 unsigned int baudrate,
                                 unsigned int obc_com_period)
    : enable_(enable), obc_com_period_(obc_com_period), kBaudRate(baudrate) {
  hils_com_port_ =
      new ComPortInterface(port_id, kBaudRate, kTxMessageSize, kRxMessageSize);
  // IsLogEnabled = true; // InitHardwareMessage.cpp内で設定
  ZeroPad(txbuff, sizeof(txbuff));
  ZeroPad(rxbuff, sizeof(rxbuff));
  ZeroPad(&hw_msg_, sizeof(hw_msg_));
}

HardwareMessage::HardwareMessage() : enable_(false), kBaudRate(0) {}

HardwareMessage::~HardwareMessage() { delete hils_com_port_; }

int HardwareMessage::Initialize() {
  if (!enable_)
    return -1;

  int ret = hils_com_port_->Initialize();
  if (ret == -1) {
    // ポート初期設定失敗
    // printf((hw_msg->hils_com_port->kPortName +
    // "の初期設定に失敗しました。").c_str());
    printf("COMポートの初期設定に失敗しました。");
  } else if (ret < -1) {
    // ポートOPEN失敗
    // printf((hw_msg->hils_com_port->kPortName +
    // "のOPENに失敗しました。").c_str());
    printf("COMポートのOPENに失敗しました。");
  }
  // printf((hw_msg->hils_com_port->kPortName +
  // "のOPENに成功しました。\n").c_str());
  printf("COMポートのOPENに成功しました。");

  return 0;
}

int HardwareMessage::ReceiveAndInterpretMsg() {
  if (!enable_)
    return -1;

  int bytes2read = hils_com_port_->BytesToRead();
  // printf("Bytes to read: %d\r", bytes2read);
  if (bytes2read < (int)kRxMessageSize) {
    // まだメッセージサイズ分のデータが受信バッファに蓄積されていない
    return -1;
  }

  int offset = 0;

  // メッセージサイズ分読み込み
  int received_bytes = hils_com_port_->Receive(rxbuff, offset, kRxMessageSize);
  // ヘッダーが一致するまでポインタを移動
  while (memcmp(rxbuff + offset, kHeader, kHeaderSize)) {
    if (offset >= kRxMessageSize - kHeaderSize)
      return -1; // ヘッダー見つからなかった
    offset++;
  }
  // ヘッダーが見つかった。
  // メッセージの全体を取得するため、offset bytes分さらに読み込む。
  if (offset != 0)
    hils_com_port_->Receive(rxbuff, kRxMessageSize, offset);

  // Buffer内に残ったデータを破棄（常に最新のデータを読むため）
  // if (hils_com_port_->DiscardInBuffer() != 0) return -1;

  uint8_t *msg_ptr = rxbuff + offset; // メッセージの開始ポインタ

  // フッターチェック
  uint8_t *footer_ptr = msg_ptr + kRxMessageSize - kFooterSize;
  if (memcmp(footer_ptr, kFooter, kFooterSize))
    return -1;

  // TODO: CRCチェック
  uint8_t *crc_ptr = footer_ptr - sizeof(crc_t);

  uint8_t *ptr = msg_ptr;
  size_t size;

  // Header
  size = sizeof(hw_msg_.header);
  memcpy(&(hw_msg_.header), ptr, size);
  ptr += size;

  // OBC TI
  size = sizeof(hw_msg_.obc_ti);
  endian_memcpy(&(hw_msg_.obc_ti), ptr, size);
  ptr += size;
  printf("\rOBC TI: %d\n", hw_msg_.obc_ti); // for debug

  // SADA
  size = sizeof(hw_msg_.sada.limit_switch_status);
  endian_memcpy(&(hw_msg_.sada.limit_switch_status), ptr, size);
  ptr += size;

  size = sizeof(hw_msg_.sada.position_steps);
  endian_memcpy(&(hw_msg_.sada.position_steps), ptr, size);
  ptr += size;

  size = sizeof(hw_msg_.sada.position_degrees);
  endian_memcpy(&(hw_msg_.sada.position_degrees), ptr, size);
  ptr += size;

  size = sizeof(hw_msg_.sada.step_period);
  endian_memcpy(&(hw_msg_.sada.step_period), ptr, size);
  ptr += size;

  size = sizeof(hw_msg_.sada.steps_remaining);
  endian_memcpy(&(hw_msg_.sada.steps_remaining), ptr, size);
  ptr += size;

  size = sizeof(hw_msg_.sada.steps_performed);
  endian_memcpy(&(hw_msg_.sada.steps_performed), ptr, size);
  ptr += size;

  size = sizeof(hw_msg_.sada.steps_since_home);
  endian_memcpy(&(hw_msg_.sada.steps_since_home), ptr, size);
  ptr += size;

  size = sizeof(hw_msg_.sada.spare);
  memcpy(&(hw_msg_.sada.spare), ptr, size);
  ptr += size;

  // Valve control
  size = sizeof(hw_msg_.valve_ctrl.TV_idx);
  endian_memcpy(&(hw_msg_.valve_ctrl.TV_idx), ptr, size);
  ptr += size;
  printf("\rTV_idx: %d\n", hw_msg_.valve_ctrl.TV_idx);

  size = sizeof(hw_msg_.valve_ctrl.cumulative_rv_open_counter);
  endian_memcpy(&(hw_msg_.valve_ctrl.cumulative_rv_open_counter), ptr, size);
  ptr += size;
  printf("\rcumulative_rv_open_counter: %d\n",
         hw_msg_.valve_ctrl.cumulative_rv_open_counter);

  printf("thrust [mN]: ");
  for (int i = 0; i < VALVE_THRUSTER_MAX; i++) {
    size = sizeof(hw_msg_.valve_ctrl.thruster_thrust[i]);
    endian_memcpy(&(hw_msg_.valve_ctrl.thruster_thrust[i]), ptr, size);
    ptr += size;
    printf("%f\t", hw_msg_.valve_ctrl.thruster_thrust[i]);
  }
  printf("\n");

  size = sizeof(hw_msg_.valve_ctrl.spare);
  memcpy(&(hw_msg_.valve_ctrl.spare), ptr, size);
  ptr += size;

  // Thermal control
  size = sizeof(hw_msg_.thermal_ctrl.spare);
  memcpy(&(hw_msg_.thermal_ctrl.spare), ptr, size);
  ptr += size;

  // Spare
  size = sizeof(hw_msg_.spare);
  memcpy(&(hw_msg_.spare), ptr, size);
  ptr += size;

  // CRC
  size = sizeof(hw_msg_.crc);
  endian_memcpy(&(hw_msg_.crc), ptr, size);
  ptr += size; // memcpyではない？

  // Footer
  size = sizeof(hw_msg_.footer);
  memcpy(&(hw_msg_.footer), ptr, size);
  ptr += size;

  return 0;
}

bool HardwareMessage::Enable() const { return enable_; }

string HardwareMessage::GetLogHeader() const {
  string str_tmp = "";
  // str_tmp += WriteVector("q_t_i2b", "tar", "-", 4);
  str_tmp += WriteScalar("OBC_TI");
  str_tmp += WriteScalar("Valve_TV_idx");
  for (int i = 0; i < VALVE_THRUSTER_MAX; i++) {
    str_tmp += WriteScalar("Thrust_" + to_string(i) + "[mN]");
  }

  return str_tmp;
}

string HardwareMessage::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteScalar(hw_msg_.obc_ti);
  str_tmp += WriteScalar((unsigned int)hw_msg_.valve_ctrl.TV_idx);
  for (int i = 0; i < VALVE_THRUSTER_MAX; i++) {
    str_tmp += WriteScalar(hw_msg_.valve_ctrl.thruster_thrust[i]);
  }
  return str_tmp;
}

void HardwareMessage::ZeroPad(void *ptr, size_t size) {
  // ptrからsizeバイトを0埋めする。
  uint8_t *p = (uint8_t *)ptr;

  for (int i = 0; i < size; i++) {
    *(p + i) = 0;
  }
}
