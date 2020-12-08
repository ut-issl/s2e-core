#include <iostream>
#include "../../Environment/Global/GnssSatellites.h"
#include "Initialize.h"

GnssSatellites* InitGnssSatellites(string file_name)
{   
  IniAccess ini_file(file_name);
  char* section = "GNSS_SATELLIES";
  string directory_path = ini_file.ReadString(section, "directory_path");
  string file_head = "COD0MGXFIN_";
  string year = "2020";
  int gps_day = 260; //　1/1を1日目とする日 https://www.ngs.noaa.gov/CORS/Gpscal.shtml
  string file_footer = "0000_01D_05M_ORB.SP3";
  int num_of_days = 3;
  vector<vector<string>> each_line(num_of_days);
  int satellite_num = 0; //衛星数
  int time_stamp_num = 0; //1ファイルの中で何時刻書かれているか

  //とりあえず
  //3つ選び出して、真ん中の日付を基準にする。
  //現状このコードだと時刻のみに依存してGNSS位置は、COD0MGXFIN_20202600000_01D_05M_ORB.sp3 ~ COD0MGXFIN_20202620000_01D_05M_ORB.SP3から取り出して、COD0MGXFIN_20202610000_01D_05M_ORB.SP3のみを見ることになる
  //真ん中一つだけにする理由は0と24付近の小さい値で補間した時に精度悪化するのを防ぐため

  //CODEのsp3ファイルを使用 
  //http://mgex.igs.org/IGS_MGEX_Products.php
  //ftp://igs.ign.fr/pub/igs/products/mgex/

  GnssSatellites* gnss_satellites;
  gnss_satellites = new GnssSatellites(satellite_num, time_stamp_num, num_of_days);
  gnss_satellites->IsCalcEnabled = ini_file.ReadEnable(section, CALC_LABEL);

  if (gnss_satellites->IsCalcEnabled){
    for(int i = 0;i < 3;++i){
      string file_name = directory_path + "/" +  file_head + year + to_string(gps_day + i) + file_footer;
      ifstream ifs(file_name);
      if (!ifs.is_open()) {
          cout << "sp3 file not found" << endl;
          abort();
      }
      string str;

      //1行目の処理
      getline(ifs, str);
      istringstream iss{str};
      for(int j = 0;j < 11;++j){
        //何時刻分書かれているか見る
        string each;
        iss >> each;
        if(j == 6){
          //289になっていることがある(というか多分なっている(恐らくラストは予想))
          int tmp_time_stamp_num = stoi(each);
          if(tmp_time_stamp_num%24) tmp_time_stamp_num -= tmp_time_stamp_num%24;
          if(time_stamp_num == 0) time_stamp_num = tmp_time_stamp_num;
          else if(time_stamp_num != tmp_time_stamp_num){
            cout << ".sp3 of time_stamp_num has something wrong" << endl;
            abort();
          }
        }
      }

      //2行目 用は特に無し
      getline(ifs, str);

      //3行目
      getline(ifs, str);
      istringstream iss3{str};
      for(int j = 0;j < 3;++j){
        string each;
        iss3 >> each;
        if(j == 1){
          //衛星数を取る
          int tmp_satellite_num = stoi(each);
          if(satellite_num == 0) satellite_num = tmp_satellite_num;
          else if(satellite_num != tmp_satellite_num){
            cout << ".sp3 of satellite_num has something wrong" << endl;
            abort();
          }
        }
      }

      //その後
      bool flag = false;
      int read_line_num = 0;
      while(getline(ifs, str)){
          if(!flag && str.front() != '*') continue;
          flag = true;
          each_line.at(i).push_back(str);
          ++read_line_num;
          if(read_line_num == time_stamp_num*(satellite_num + 1)) break;
      }

      ifs.close();
    }

    gnss_satellites->ReadSP3(each_line);
  }

  return gnss_satellites;
}