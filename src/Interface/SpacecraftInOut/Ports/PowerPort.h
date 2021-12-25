#pragma once
class ComponentBase;

// PCUについている各コンポへの電源ポートを模するクラス
// PCU内部ソフトウェアの認識している各ch電源ON/OFF状態と、実際のハードウェアでの電源ON/OFF状態が
// 異なる場合があるので、それを再現するためPCUとコンポの間に1枚噛ませる。
class PowerPort
{
public:
  PowerPort(int port_id, double currentLimit, ComponentBase* component);
  ~PowerPort();
  
  // 接続されたコンポの現在の消費電流[A]を返す
  double GetCurrent();
  
  // 電源ポートの現在の電圧[V]を返す
  double GetVoltage();

  // 電源ポートの現在の電圧[V]を設定する
  void SetVoltage(double voltage);

  // 接続先コンポーネントを返す
  ComponentBase* GetComponent();

  // 接続先コンポーネントを設定する
  void SetComponent(ComponentBase* component);

private:

  // 電源ポートの上限電流[A]
  const double kCurrentLimit;

  // 電源ポートのポートID
  const int kPortId;

  // 接続先のコンポーネント
  ComponentBase* component_;

  // 電源ポートの電圧[V]
  double voltage_;
};

