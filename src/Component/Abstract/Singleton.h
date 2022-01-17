#pragma once

// インスタンスがただ一つに定まるときに継承させることで，制約をかけている．
// オレオレ実装なので，なんかライブラリとか使って肩に乗りたいが．．
template <typename T> class Singleton {
protected:
  Singleton() {}
  Singleton(Singleton const &);
  ~Singleton() {}

  Singleton &operator=(Singleton const &);

public:
  static T *GetInstance() {
    static T instance;
    return &instance;
  }
};
