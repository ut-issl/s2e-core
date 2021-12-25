#pragma once
#include <map>
#include "Ports/PowerPort.h"
class ComponentBase;

// PowerPortをまとめるクラス
// 搭載ソフトとは関わってこないが、便宜上このフォルダに配置する
class PowerDriver
{
public:
  // ポート番号port_id、電流制限値currentLimit[A]で、componentに接続された電源ポートを作成する
  static int ConnectPort(int port_id, double currentLimit, ComponentBase* component);

  // PowerPortsに登録されたorgのアドレスをaltに置き換える
  static void ReplaceComponent(ComponentBase * org, ComponentBase * alt);

  // PowerPortsに登録されたorgを削除する
  static void RemoveComponent(ComponentBase * component);

  // port_idに接続されたコンポの現在の消費電流[A]を取得する
  static double GetCurrent(int port_id);

  // port_idの電源ポートの現在の電圧[V]を取得する
  static double GetVoltage(int port_id);

  // port_idの電源ポートの現在の電圧[V]を設定する
  static void SetVoltage(int port_id, double voltage);

private:
  static std::map<int, PowerPort*> ports_;
};

