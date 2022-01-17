#pragma once

#include <string>

// 必要ないが、利便性のためにinclude
#include "LogUtility.h"

// ログに書けるクラス

class ILoggable {
  // インターフェースにしたかったが、イネーブル周りの書き直しが面倒になるので
  // 抽象クラスとします　多重継承してください
public:
  // csvのヘッダー行の文字列を取得する
  virtual std::string GetLogHeader() const = 0;

  // csvの値行の文字列を取得する
  virtual std::string GetLogValue() const = 0;

  bool IsLogEnabled = true;
};
