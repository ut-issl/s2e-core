#pragma once
// コンポとポート番号の対応表

enum PowerPortConfig {
  // port_idの順に登録してください。番号がかけている場合はNONE_1,NONE_2…として書いてください
  // COMPONENT_MAXは消さず、その上に全てのポートに対応するコンポを書いてください
  OBC_BUS,
  GYRO_5V,
  COMPONENT_MAX
};

// これはOBCソフトウェア側との対応と合わせること！
enum UARTPortConfig { GYRO = 0, UART_COMPONENT_MAX };
