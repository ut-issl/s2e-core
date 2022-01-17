#pragma once
class ITickable {
public:
  virtual void Tick(int count) = 0;
  // Calculate high-frequency disturbances
  // Override this only for components that need fast updates.
  virtual void FastTick(int fast_count) = 0;

  // Whether or not high-frequency disturbances need to be calculated
  inline bool GetNeedsFastUpdate() { return needs_fast_update_; }
  inline void SetNeedsFastUpdate(bool need_fast_update) {
    needs_fast_update_ = need_fast_update;
  }

protected:
  // Whether or not high-frequency disturbances need to be calculated
  bool needs_fast_update_ = false;
};