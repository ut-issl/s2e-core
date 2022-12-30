/**
 * @file Singleton.h
 * @brief Class to restrict that class that the class can have only one instance at a time
 * @note TODO: Is this needed? Currently this is used in StateMachine only. And we need to use other library if we want to use.
 */

#pragma once

template <typename T>
class Singleton {
 protected:
  Singleton() {}
  Singleton(Singleton const&);
  ~Singleton() {}

  Singleton& operator=(Singleton const&);

 public:
  static T* GetInstance() {
    static T instance;
    return &instance;
  }
};
