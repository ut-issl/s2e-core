/**
 * @file relative_orbit_models.cpp
 * @brief Functions to calculate Yamanaka-ANkersen STM for relative orbit
 */
#include "relative_orbit_carter.hpp"

#include <environment/global/physical_constants.hpp>

#include "./sgp4/sgp4unit.h"  // for getgravconst()

namespace orbit {

RelativeOrbitCarter::RelativeOrbitCarter() {}

RelativeOrbitCarter::~RelativeOrbitCarter() {}

void RelativeOrbitCarter::CalculateInitialInverseMatrix(double orbit_radius_m, double gravity_constant_m3_s2, double f_ref_rad,
                                                                  OrbitalElements* reference_oe) {
  double n = sqrt(gravity_constant_m3_s2 / pow(orbit_radius_m, 3));
  double e = reference_oe->GetEccentricity();
  double E_rad = 2 * atan(sqrt((1 - e) / (1 + e)) * tan(f_ref_rad / 2));
  double k = e * cos(f_ref_rad) + 1;
  double K1 = pow(1 - e * e, -2.5) * (-1.5 * e * E_rad + (1 + e * e) * sin(E_rad) - e * sin(2. * E_rad) / 4.);
  double K2 = pow(1 - e * e, -2.5) * (0.5 * E_rad - 0.25 * sin(2 * E_rad) - e * pow(sin(E_rad), 3) / 3.);
  double phi1 = sin(f_ref_rad) * k;
  double phi2 = 2 * e * phi1 * (sin(f_ref_rad) / pow(k, 3) - K2) - cos(f_ref_rad) / k;
  double phi3 = (6 * e * phi1 * K2 - (2 * pow(sin(f_ref_rad), 2) / pow(k, 2)) - (pow(cos(f_ref_rad), 2) / k) - pow(cos(f_ref_rad), 2));
  double phi1_prime = cos(f_ref_rad) * k - e * pow(sin(f_ref_rad), 2);
  double sigma4 = atan(tan(f_ref_rad / 2) * sqrt(-(e - 1) / (e + 1)));
  double term1_phi2_prime = sin(f_ref_rad) / k;
  double term2_phi2_prime = 2 * e * sin(f_ref_rad) * k * (cos(f_ref_rad) / pow(k, 3)) - (cos(f_ref_rad) / pow(1 - pow(e, 2), 5.0 / 2.0));
  double phi2_prime = term1_phi2_prime + term2_phi2_prime;
  double phi3_prime = k - 4 * e * pow(sin(f_ref_rad), 3) / pow(E_rad / 2., 3) + 2 * cos(f_ref_rad) * sin(f_ref_rad) / pow(E_rad / 2., 2) 
                    - 6 * e * cos(f_ref_rad) * E_rad / 2. / pow(1 - pow(e, 2), 5.0 / 2.0);
  double S1 = -cos(f_ref_rad) - (e / 2.0) * pow(cos(f_ref_rad), 2);
  double S2 = 3 * e * k * k * K2 - (sin(f_ref_rad) / k);
  double S3 = -6 * k * k * K2 - ((2 + k) / (2 * k)) * sin(2 * f_ref_rad); 
  initial_inverse_matrix_[0][0] = 4*S2 + phi2_prime;  
  initial_inverse_matrix_[0][1] = 0.0;              
  initial_inverse_matrix_[0][2] = 0.0;              
  initial_inverse_matrix_[0][3] = -phi2;            
  initial_inverse_matrix_[0][4] = 2*S2;             
  initial_inverse_matrix_[0][5] = 0.0;              
  initial_inverse_matrix_[1][0] = 0.0;              
  initial_inverse_matrix_[1][1] = 0.0;              
  initial_inverse_matrix_[1][2] = 0.0;              
  initial_inverse_matrix_[1][3] = -2;               
  initial_inverse_matrix_[1][4] = -1;               
  initial_inverse_matrix_[1][5] = 0.0;              
  initial_inverse_matrix_[2][0] = 0.0;              
  initial_inverse_matrix_[2][1] = 0.0;              
  initial_inverse_matrix_[2][2] = cos(f_ref_rad);   
  initial_inverse_matrix_[2][3] = 0.0;              
  initial_inverse_matrix_[2][4] = 0.0;              
  initial_inverse_matrix_[2][5] = -sin(f_ref_rad);  
  initial_inverse_matrix_[3][0] = -(4*S1 + phi1_prime);
  initial_inverse_matrix_[3][1] = 0.0;               
  initial_inverse_matrix_[3][2] = 0.0;               
  initial_inverse_matrix_[3][3] = phi1;              
  initial_inverse_matrix_[3][4] = -2*S1;             
  initial_inverse_matrix_[3][5] = 0.0;               
  initial_inverse_matrix_[4][0] = 2*S3 + phi3_prime; 
  initial_inverse_matrix_[4][1] = 0.0;               
  initial_inverse_matrix_[4][2] = -1;                
  initial_inverse_matrix_[4][3] = -phi3;             
  initial_inverse_matrix_[4][4] = S3;                
  initial_inverse_matrix_[4][5] = 0.0;               
  initial_inverse_matrix_[5][0] = 0.0;               
  initial_inverse_matrix_[5][1] = 0.0;               
  initial_inverse_matrix_[5][2] = sin(f_ref_rad);    
  initial_inverse_matrix_[5][3] = 0.0;               
  initial_inverse_matrix_[5][4] = 0.0;               
  initial_inverse_matrix_[5][5] = cos(f_ref_rad);    
}

math::Matrix<6, 6> RelativeOrbitCarter::CalculateSTM(double orbit_radius_m, double gravity_constant_m3_s2, double f_ref_rad, OrbitalElements* reference_oe){
  math::Matrix<6, 6> stm;
  // ここでstmを計算してください
  double n = sqrt(gravity_constant_m3_s2 / pow(orbit_radius_m, 3));
  double e = reference_oe->GetEccentricity();
  double E_rad = 2 * atan(sqrt((1 - e) / (1 + e)) * tan(f_ref_rad / 2));
  double k = e * cos(f_ref_rad) + 1;
  double K1 = pow(1 - e * e, -2.5) * (-1.5 * e * E_rad + (1 + e * e) * sin(E_rad) - e * sin(2. * E_rad) / 4.);
  double K2 = pow(1 - e * e, -2.5) * (0.5 * E_rad - 0.25 * sin(2 * E_rad) - e * pow(sin(E_rad), 3) / 3.);
  double phi1 = sin(f_ref_rad) * k;
  double phi2 = 2 * e * phi1 * (sin(f_ref_rad) / pow(k, 3) - K2) - cos(f_ref_rad) / k;
  double phi3 = (6 * e * phi1 * K2 - (2 * pow(sin(f_ref_rad), 2) / pow(k, 2)) - (pow(cos(f_ref_rad), 2) / k) - pow(cos(f_ref_rad), 2));
  double phi1_prime = cos(f_ref_rad) * k - e * pow(sin(f_ref_rad), 2);
  double sigma4 = atan(tan(f_ref_rad / 2) * sqrt(-(e - 1) / (e + 1)));
  double term1_phi2_prime = sin(f_ref_rad) / k;
  double term2_phi2_prime = 2 * e * sin(f_ref_rad) * k * (cos(f_ref_rad) / pow(k, 3)) - (cos(f_ref_rad) / pow(1 - pow(e, 2), 5.0 / 2.0));
  double phi2_prime = term1_phi2_prime + term2_phi2_prime;
  double phi3_prime = k - 4 * e * pow(sin(f_ref_rad), 3) / pow(E_rad / 2., 3) + 2 * cos(f_ref_rad) * sin(f_ref_rad) / pow(E_rad / 2., 2) 
                    - 6 * e * cos(f_ref_rad) * E_rad / 2. / pow(1 - pow(e, 2), 5.0 / 2.0);
  double S1 = -cos(f_ref_rad) - (e / 2.0) * pow(cos(f_ref_rad), 2);
  double S2 = 3 * e * k * k * K2 - (sin(f_ref_rad) / k);
  double S3 = -6 * k * k * K2 - ((2 + k) / (2 * k)) * sin(2 * f_ref_rad); 
  stm[0][0] = phi1;
  stm[0][1] = phi3;   
  stm[0][2] = 0.0;    
  stm[0][3] = phi2;   
  stm[0][4] = 0.0;    
  stm[0][5] = 0.0;    
  stm[1][0] = phi1_prime; 
  stm[1][1] = phi3_prime; 
  stm[1][2] = 0.0;        
  stm[1][3] = phi2_prime; 
  stm[1][4] = 0.0;        
  stm[1][5] = 0.0;        
  stm[2][0] = -2*S1; 
  stm[2][1] = -S3;   
  stm[2][2] = 0.0;   
  stm[2][3] = -2*S2; 
  stm[2][4] = -1;    
  stm[2][5] = 0.0;   
  stm[3][0] = -2*phi1; 
  stm[3][1] = -(2*phi3 + 1); 
  stm[3][2] = 0.0;           
  stm[3][3] = -2*phi2;       
  stm[3][4] = 0.0;           
  stm[3][5] = 0.0;           
  stm[4][0] = 0.0;            
  stm[4][1] = 0.0;            
  stm[4][2] = 0.0;            
  stm[4][3] = 0.0;            
  stm[4][4] = cos(f_ref_rad);  
  stm[4][5] = sin(f_ref_rad);  
  stm[5][0] = 0.0;            
  stm[5][1] = 0.0;            
  stm[5][2] = 0.0;            
  stm[5][3] = 0.0;            
  stm[5][4] = -sin(f_ref_rad); 
  stm[5][5] = cos(f_ref_rad);  
  return stm * initial_inverse_matrix_;
}

}  // namespace orbit
