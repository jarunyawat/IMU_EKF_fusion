#include <Arduino.h>
#include <Matrix.h>
#include <Wire.h>

// put function declarations here:
void setupIMU();
void readAcc(float&, float&, float&);
void readGyro(float&, float&, float&);
void printMat(Matrix mat);
const int8_t MPU_addr = 0x68;
const float gravity = -9.81;
float dt = 0.1;
float x_val[] = {0, 0}, p_val[] = {1, 0, 0, 1}, Q_val[] = {0.0174533, 0, 0, 0.0174533}, R_val[] = {1, 0, 0, 0, 1, 0, 0, 0 , 1};
float ax=0.0, ay=0.0, az=0.0, p=0.0, q=0.0, r=0.0;
Matrix x_mat(2, 1, x_val), p_mat(2, 2, p_val), Q_mat(2, 2, Q_val), R_mat(3, 3, R_val);
unsigned long timeStamp = millis();
float x_pred_val[] = {0,0};
Matrix x_pred_mat(2, 1, x_pred_val);
float f_val[] = {0,0,0,0};
Matrix f_mat(2, 2, f_val);
float p_f_val[] = {0, 0, 0, 0};
Matrix p_f_mat(2, 2, p_f_val);
float pt_val[] = {0, 0, 0, 0};
Matrix pt_mat(2, 2, pt_val);
float p_bar_val[] = {5, 6, 7, 8};
Matrix p_bar_mat(2, 2, p_bar_val);
float h_val[] = {0,0,0,0,0,0};
Matrix h_mat(3, 2, h_val);
float ht_val[] = {0, 0, 0, 0, 0, 0};
Matrix ht_mat(2, 3, ht_val);
float p_bar_ht_val[] = {0, 0, 0, 0, 0, 0};
Matrix p_bar_ht_mat(2, 3, p_bar_ht_val);
float v_val[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
Matrix v_mat(3, 3, v_val);
float inv_val[] = {0,0,0,0,0,0,0,0,0};
Matrix inv_mat(3, 3, inv_val);
float k_val[] = {0,0,0,0,0,0};
Matrix k_mat(2, 3, k_val);
float a_pred_val[] = {0,0,0};
Matrix a_pred_mat(3, 1, a_pred_val);
float diff_val[] = {0,0,0};
Matrix diff_mat(3, 1, diff_val);
float correction_val[] = {0,0};
Matrix correction_mat(2, 1, correction_val);
float k_h_val[] = {0,0,0,0};
Matrix k_h_mat(2, 2, k_h_val);
float iden_val[] = {1 , 2, 3, 4};
Matrix iden_mat(2, 2, iden_val);

void setup(){
  Serial.begin(9600);
  Serial.print("IMU setup");
  setupIMU();
}

void loop(){
  readAcc(ax, ay, az);
  readGyro(p, q, r);
  if(millis()-timeStamp>100){
    timeStamp = millis();
    // ENU to NED 
    // float temp = ay;
    // ay = ax;
    // ax = temp;
    // az = -az;
    // temp = q;
    // q = p;
    // p = temp;
    // r = -r;
    ay = -ay;
    az = -az;
    q = -q;
    r = -r;
    // Calculate angle for Accelerometer
    float roll = atan2(ay,az);
    float pitch = asin(ax/gravity);
    
    // Prediction step
    x_pred_mat.at(0,0) = x_mat.at(0, 0) + dt*(p+q*sin(roll)*tan(pitch)+r);
    x_pred_mat.at(1,0) = x_mat.at(1, 0) + dt*(q*cos(roll)-r*sin(roll));
    f_mat.at(0,0) = dt*(q*cos(roll)*tan(pitch));
    f_mat.at(0,1) = dt*(q*sin(roll)*sq(1.0/cos(pitch)));
    f_mat.at(1,0) = dt*(-q*sin(roll)+r*cos(roll));
    f_mat.at(1,1) = 0.0;

    matMul(p_mat, f_mat, p_f_mat);

    matTranspose(p_mat, pt_mat);
    matMul(p_f_mat, pt_mat, p_bar_mat);
    matPlus(p_bar_mat, Q_mat, p_bar_mat);

    // // Update step
    h_mat.at(0,0) = 0.0;
    h_mat.at(0,1) = gravity*cos(pitch);
    h_mat.at(1,0) = -gravity*cos(roll)*cos(pitch);
    h_mat.at(1,1) = gravity*sin(roll)*cos(pitch);
    h_mat.at(2,0) = gravity*sin(roll)*cos(pitch);
    h_mat.at(2,1) = gravity*cos(roll)*sin(pitch);
    matTranspose(h_mat, ht_mat);

    matMul(p_bar_mat, ht_mat, p_bar_ht_mat);

    matMul(h_mat, p_bar_ht_mat, v_mat);
    matPlus(v_mat, R_mat, v_mat);

    float det_val = 0.000001 + (v_mat.at(0,0)*v_mat.at(1,1)*v_mat.at(2,2) + v_mat.at(0,1)*v_mat.at(1,2)*v_mat.at(2,0) + v_mat.at(0,2)*v_mat.at(1,0)*v_mat.at(2,1)) - 
                    (v_mat.at(2,0)*v_mat.at(1,1)*v_mat.at(0,2) + v_mat.at(2,1)*v_mat.at(1,2)*v_mat.at(0,0) + v_mat.at(2,2)*v_mat.at(1,0)*v_mat.at(0,1));

    inv_mat.at(0,0) = (v_mat.at(1,1)*v_mat.at(2,2)-v_mat.at(2,1)*v_mat.at(1,2))/det_val;
    inv_mat.at(0,1) = (-(v_mat.at(0,1)*v_mat.at(2,2)-v_mat.at(2,1)*v_mat.at(0,2)))/det_val;
    inv_mat.at(0,2) = (v_mat.at(0,1)*v_mat.at(1,2)-v_mat.at(1,1)*v_mat.at(0,2))/det_val;
    inv_mat.at(1,0) = (-(v_mat.at(1,0)*v_mat.at(2,2)-v_mat.at(2,0)*v_mat.at(1,2)))/det_val;
    inv_mat.at(1,1) = (v_mat.at(0,0)*v_mat.at(2,2)-v_mat.at(2,0)*v_mat.at(0,2))/det_val;
    inv_mat.at(1,2) = (-(v_mat.at(0,0)*v_mat.at(1,2)-v_mat.at(1,0)*v_mat.at(0,2)))/det_val;
    inv_mat.at(2,0) = (v_mat.at(1,0)*v_mat.at(2,1)-v_mat.at(2,0)*v_mat.at(1,1))/det_val;
    inv_mat.at(2,1) = (-(v_mat.at(0,0)*v_mat.at(2,1)-v_mat.at(2,0)*v_mat.at(0,1)))/det_val;
    inv_mat.at(2,2) = (v_mat.at(0,0)*v_mat.at(1,1)-v_mat.at(1,0)*v_mat.at(0,1))/det_val;
    
    matMul(p_bar_ht_mat, inv_mat, k_mat);

    a_pred_mat.at(0,0) = gravity*sin(x_pred_mat.at(1,0));
    a_pred_mat.at(1,0) = -cos(x_pred_mat.at(1,0))*sin(x_pred_mat.at(0,0))*gravity;
    a_pred_mat.at(2,0) = -cos(x_pred_mat.at(1,0))*cos(x_pred_mat.at(0,0))*gravity;
    diff_mat.at(0,0) = ax;
    diff_mat.at(1,0) = ay;
    diff_mat.at(2,0) = az;
    matSub(diff_mat, a_pred_mat, diff_mat);

    matMul(k_mat, diff_mat, correction_mat);
    matPlus(x_pred_mat, correction_mat, x_mat);

    matMul(k_mat, h_mat, k_h_mat);
    
    iden_mat.at(0,0) = 1;
    iden_mat.at(0,1) = 0;
    iden_mat.at(1,0) = 0;
    iden_mat.at(1,1) = 1;
    matSub(iden_mat, k_h_mat, iden_mat);
    matMul(iden_mat, p_bar_mat, p_mat);
    printMat(x_mat);
    // Serial.println("=============");
    // Serial.print(ax);
    // Serial.print("\t");
    // Serial.print(ay);
    // Serial.print("\t");
    // Serial.println(az);
  }
}

// put function definitions here:
void setupIMU(){
  Wire.begin();
  // Begin transmission
  Wire.beginTransmission(MPU_addr);
  // Setup Acc
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(50);
  // // Set up gyro
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void readAcc(float& ax, float& ay, float& az){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6 , true);
  int16_t xRaw = (Wire.read()<<8) | Wire.read();
  int16_t yRaw = (Wire.read()<<8) | Wire.read();
  int16_t zRaw = (Wire.read()<<8) | Wire.read();
  ax = (static_cast<float>(xRaw) / 16384.0) * gravity;
  ay = (static_cast<float>(yRaw) / 16384.0) * gravity;
  az = (static_cast<float>(zRaw) / 16384.0) * gravity;
}

void readGyro(float& p, float& q, float& r){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6 , true);
  int16_t xRaw = (Wire.read()<<8) | Wire.read();
  int16_t yRaw = (Wire.read()<<8) | Wire.read();
  int16_t zRaw = (Wire.read()<<8) | Wire.read();
  p = (static_cast<float>(xRaw) / 131.0) * (22.0/7.0) / 180.0;
  q = (static_cast<float>(yRaw) / 131.0) * (22.0/7.0) / 180.0;
  r = (static_cast<float>(zRaw) / 131.0) * (22.0/7.0) / 180.0;
}

void printMat(Matrix mat){
  Serial.println();
  for(int i=0;i<mat.getRow();i++){
    for(int j=0;j<mat.getCol();j++){
      Serial.print(mat.at(i, j));
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println("=================");
}