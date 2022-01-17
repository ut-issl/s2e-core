#pragma once
#include "Singleton.h"
#include <list>
#include <map>
#include <string>

// 参考
// https://setuna-kanata.hatenadiary.org/entry/20090225/1235570379
// https://setuna-kanata.hatenadiary.org/entry/20090226/1235658062
// http://www.lancarse.co.jp/blog/?p=1039

// 実行時のオーバーヘッドを減らすために，CRTPで実装している
// http://agtn.hatenablog.com/entry/2016/06/16/192708
// https://theolizer.com/cpp-school2/cpp-school2-19/

// 最終的にはBoostとかの汎用ライブラリを使った方が維持負担が少ないのでは．

class Event;
template <typename ContextType> class ActionInterface;
template <typename ContextType> class StateInterface;
template <typename ContextType> class StateMachine;

// 状態遷移やイベント処理の際に実行されるActionのクラス．
template <typename ContextType> class ActionInterface {
public:
  virtual void Exe(ContextType *context) = 0;
};

// ステートマシンに処理を要求するイベントのクラス．
class Event {
private:
  int value_;

public:
  Event(int value) : value_(value) {}

  // std::mapを使用するために演算子をオーバーロード
  bool operator==(Event const &e) const { return value_ == e.value_; }
  bool operator<(Event const &e) const { return value_ < e.value_; }
};

// 状態および状態間の処理や関係を与えるクラス．
template <typename ContextType> class StateInterface {
public:
  using context_t = ContextType;
  using state_t = StateInterface<context_t>;
  using action_t = ActionInterface<context_t>;
  typedef std::map<Event, state_t *> transition_map;
  typedef std::map<Event, action_t *> action_map;

public:
  // 現在の実装では，Entry，Exitは遷移の際にChainして呼び出されるが，Exeに関しては，その状態のメンバ関数のみが実行される！！
  virtual void Entry() {}
  virtual void Exe() {}
  virtual void Exit() {}

protected:
  context_t *context_;
  state_t *parent_; // 親状態（状態は階層化されている）
  string name_;     //デバッグ用
  transition_map transition_;
  action_map action_;

protected:
  StateInterface(string const &name) : name_(name) {}

public:
  // 階層化された親状態のSetter
  void SetParent(state_t *parent, context_t *context) {
    this->parent_ = parent;
    this->context_ = context;
  }

  // 親状態のGetter
  StateInterface *GetParent() const { return parent_; }

public:
  void AddTransition(Event const &e, state_t *state) {
    transition_.insert(std::make_pair(e, state));
  }

  bool Transit(Event const &e) {
    typename transition_map::iterator it = transition_.find(e);
    if (it != transition_.end()) {
      context_->ChangeState(it->second, this);
      typename action_map::iterator it_action = action_.find(e);
      if (it_action != action_.end())
        it_action->second->Exe(context_);
      return true;
    }
    return false;
  }

public:
  void AddAction(Event const &e, action_t *act) {
    action_.insert(std::make_pair(e, act));
  }
};

// 階層化ステートマシンの実装．
template <typename ContextType> class StateMachine {
protected:
  typedef ContextType context_t;
  typedef StateInterface<context_t> state_t;
  typedef ActionInterface<context_t> action_t;

protected:
  //開始状態．一意なので，Singletonを継承．
  class Start : public state_t, public Singleton<Start> {
    friend class Singleton<Start>;
    Start() : state_t("Start") {} // デバッグ用にStateに命名
  };
  //終了状態．一意なので，Singletonを継承．
  class End : public state_t, public Singleton<End> {
    friend class Singleton<End>;
    End() : state_t("End") {}
  };

private:
  state_t *current_;

public:
  StateMachine()
      : current_(Start::GetInstance()) //　Startステートで初期化
  {}

public:
  void Update() { current_->Exe(); }

  void ChangeState(state_t *dest_state, state_t *src_state = 0) {
    if (src_state == 0)
      src_state = current_;

    //自己遷移
    if (src_state == dest_state) {
      src_state->Exit();
      dest_state->Entry();
      return;
    }

    // 自己でない場合，状態の木構造を探索して，必要な状態遷移の行動列を生成する．

    typedef std::list<state_t *> state_list;
    // src_stateからrootまでの経路
    state_list src_top;
    for (state_t *s = src_state; s != 0; s = s->GetParent())
      src_top.push_front(s);
    // dest_stateからrootまでの経路
    state_list dest_top;
    for (state_t *s = dest_state; s != 0; s = s->GetParent())
      dest_top.push_front(s);

    //最も近い共通の親をrootから検索する
    state_t *parent = 0;
    typename state_list::iterator it_src = src_top.begin();
    typename state_list::iterator it_dest = dest_top.begin();
    while (it_src != src_top.end() && it_dest != dest_top.end()) {
      if (*it_src != *it_dest)
        break;
      ++it_src;
      ++it_dest;
    }
    // it_srcが先頭でない時の--it_srcが共通の親
    if (it_src != src_top.begin()) {
      --it_src;
      parent = *it_src;
    }

    //退場動作．子→親の順にExit
    for (state_t *s = current_; s != parent; s = s->GetParent())
      s->Exit();

    //入場動作．親→子の順にEntry
    for (; it_dest != dest_top.end(); ++it_dest)
      (*it_dest)->Entry();

    current_ = dest_state;
  }

  context_t *Derived() { return static_cast<context_t *>(this); }

  void ProcessEvent(Event const &e) {
    //イベントによるtransition
    // Exit & Entry でカバーできない特殊処理が必要な遷移を実施．
    // ChangeStateした上で，自分で処理できない場合は親に渡す
    state_t *s = current_;
    while (
        s != 0 &&
        !s->Transit(
            e)) // Transitはtransition_が存在するかどうかをBoolで返す．ので，Transitに関しては子は親の処理を実行しない（はず）．
      s = s->GetParent();
  }

  void AddTransition(state_t *current, Event const &e, state_t *next,
                     action_t *action) {
    current->AddTransition(e, next);
    if (action)
      current->AddAction(e, action);
  }

public:
  bool is_end() const { return current_ == End::GetInstance(); }
};
