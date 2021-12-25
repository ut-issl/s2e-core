#include "PowerDriver.h"

std::map<int, PowerPort*> PowerDriver::ports_;

int PowerDriver::ConnectPort(int port_id, double currentLimit, ComponentBase * component)
{
  if (ports_[port_id] != nullptr)
  {
    // 既に使用されているポートを指定された
    return -1;
  }
  ports_[port_id] = new PowerPort(port_id, currentLimit, component);
  return 0;
}

void PowerDriver::ReplaceComponent(ComponentBase* org, ComponentBase* alt)
{
  // ポインタがorgの要素を検索（複数個ありうる）
  for (auto itr = ports_.begin(); itr != ports_.end();)
  {
    if (itr->second == nullptr)
    {
      ++itr; continue;
    }
    if (itr->second->GetComponent() == org)
    {
      if (alt == nullptr)
      {
        // 後置インクリメントしないとerase時にiteratorがおかしくなる
        // http://d.hatena.ne.jp/Ox8000ffff/20100225/1267201650
        ports_.erase(itr++);
      }
      else
      {
        (++itr)->second->SetComponent(alt);
      }
    }
    else ++itr;
  }
}

void PowerDriver::RemoveComponent(ComponentBase* component)
{
  ReplaceComponent(component, nullptr);
}

double PowerDriver::GetCurrent(int port_id)
{
  PowerPort* port = ports_[port_id];
  if (port == nullptr) return 0;
  return port->GetCurrent();
}

double PowerDriver::GetVoltage(int port_id)
{
  PowerPort* port = ports_[port_id];
  if (port == nullptr) return 0;
  return port->GetVoltage();
}

void PowerDriver::SetVoltage(int port_id, double voltage)
{
  PowerPort* port = ports_[port_id];
  if (port == nullptr) return;
  return port->SetVoltage(voltage);
}
