#pragma once
#include <Interface/LogOutput/ILoggable.h>
// HipDataを要素に持つベクタが欲しかったので，STLのベクタを使用している
//計算に使うベクタは歴代nlabのベクタを使用しているので，混在していてまずい...
#include <Interface/LogOutput/ILoggable.h>

#include <Library/math/Quaternion.hpp>
#include <Library/math/Vector.hpp>
#include <vector>

struct HipData {
  int hip_num;  //番号
  double vmag;  //視等級
  double ra;    //赤経
  double de;    //赤緯
};

class HipparcosCatalogue : public ILoggable {
 public:
  HipparcosCatalogue(double max_magnitude, std::string catalogue_path);
  virtual ~HipparcosCatalogue();
  bool ReadContents(const std::string& filename, const char delimiter);
  //ヒッパルコス星表のデータは視等級順に並べているので、等級の順位を引数に取る
  int GetCatalogueSize() const { return hip_catalogue.size(); }
  int GetHipID(int rank) const { return hip_catalogue[rank].hip_num; }
  double GetVmag(int rank) const { return hip_catalogue[rank].vmag; }
  double GetRA(int rank) const { return hip_catalogue[rank].ra; }
  double GetDE(int rank) const { return hip_catalogue[rank].de; }
  libra::Vector<3> GetStarDir_i(int rank) const;  //恒星の方向を返す
  libra::Vector<3> GetStarDir_b(int rank,
                                Quaternion q_i2b) const;  //恒星の方向を返す

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  bool IsCalcEnabled = true;

 private:
  std::vector<HipData> hip_catalogue;  // CSVデータの格納先
  double max_magnitude_;
  std::string catalogue_path_;
};
