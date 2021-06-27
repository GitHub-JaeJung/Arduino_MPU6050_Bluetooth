#include <SoftwareSerial.h>
//#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>

MPU6050 accelgyro1(0x68);
MPU6050 accelgyro2(0x69);

int16_t ax1, ay1, az1;
int16_t gx1, gy1, gz1;


double angle1 = 0, deg ; // angle, deg data (각도계산)
double angle2 = 0;
double dgy_x ;
long mapping_value = 1000;
long normal_x, normal_y, normal_z, deltha_x[3], deltha_y[3], deltha_z[3], deltha, angle_1[3], angle_2[3] ;
long event_value = 1000;
boolean ACCESS_ACCEL = false;

int16_t ac_x, ac_y, ac_z, gy_x, gy_y, gy_z ; //acc, gyro data (acc, gyro 계산 수식)
double angleAcX;
double angleAcY;
double anglemain;
const double RADIAN_TO_DEGREE = 180 / 3.14159;
int Nomal_Angle = 40;

SoftwareSerial BtSerial(6, 7);

#define OUTPUT_READABLE_ACCELGYRO
#define mpu_add 0x68

void value_init();
//void ACCEL_ACCESS();

void setup() {
  Wire.begin();
  Serial.begin(115200);
  accelgyro1.initialize();
  accelgyro2.initialize();
  BtSerial.begin(115200);
  //ACCESS_ACCEL = ACCEL_ACCESS(); // 가속도 센서 값 맞게 나오는지 확인하기
}

void loop() {
  //  moving_data();

  if (BtSerial.available()) { //블루투스에서 넘어온 데이터가 있다면
    Serial.write(BtSerial.read()); //시리얼모니터에 데이터를 출력
  }
  if (Serial.available()) {    //시리얼모니터에 입력된 데이터가 있다면
    BtSerial.write(Serial.read());  //블루투스를 통해 입력된 데이터 전달
  }

  accelgyro1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);


  //Serial.print("a1/g1:\t");
  /*
    Serial.print(ax1); Serial.print("\t");
    Serial.print(ay1); Serial.print("\t");
    Serial.print(az1); Serial.print("\t");
    Serial.print(gx1); Serial.print("\t");
    Serial.print(gy1); Serial.print("\t");
    Serial.print(gz1);
    Serial.print("\n");
  */
  //Serial.println(moving);
  
  value_init(); // 변수 초기화
  accel_calculate(); // 가속도 측정 -> 각도계산
  //angle1 = abs(angle1); // 각도 변수에 저장
  //angle2 = abs(angle2); // 각도 변수에 저장
  
  //Serial.println( " ANGLE1 : " + String(angle1)); Serial.println( " ANGLE2 : " + String(angle2));
  //Serial.println( " ANGLE1 : " + String(angleAcX)); Serial.println( " ANGLE2 : " + String(angleAcY));
  //Serial.print("Angle x : ");
  //Serial.print(angleAcX);
  //Serial.print("\t");
  //Serial.print("\t\t Angle y : ");
  //Serial.println(angleAcY);
  Serial.println(anglemain);

  angle1 = abs(angleAcX);
  angle2 = abs(angleAcY);

  //BtSerial.print("Angle x : ");
  //BtSerial.print(angleAcX);
  //BtSerial.print("\t");
  //BtSerial.print("\t\t Angle y : ");
  //BtSerial.println(angleAcY);

  
  if ((angle1 < Nomal_Angle ) && (angle2 < Nomal_Angle )) { // 정상 상태에 돌아왔을 경우 (각도가 안정상태일 경우)
    //Serial.println("정상 진행");
    //BtSerial.println("정상 진행");
  }
  
}

void accel_calculate() {
  ac_x = 0; ac_y = 0; ac_z = 0;

  Wire.beginTransmission(mpu_add) ; // 번지수 찾기
  Wire.write(0x3B) ; // 가속도 데이터 보내달라고 컨트롤 신호 보내기
  Wire.endTransmission(false) ; // 기달리고,
  Wire.requestFrom(mpu_add, 6, true) ; // 데이터를 받아 처리

  // Data SHIFT
  ac_x = Wire.read() << 8 | Wire.read() ;
  ac_y = Wire.read() << 8 | Wire.read() ;
  ac_z = Wire.read() << 8 | Wire.read() ;

  //Serial.println("X : " + String(ac_x) + "Y : " + String(ac_y) + "Z : " + String(ac_z));
  //if (ACCESS_ACCEL) { } else {
  //  return;
  //}

  //맵핑화 시킨 것 – 즉 10000으로 맵핑시킴
  normal_x = map(int(ac_x), -16384, 16384, 0, mapping_value);
  normal_y = map(int(ac_y), -16384, 16384, 0, mapping_value);
  normal_z = map(int(ac_z), -16384, 16384, 0, mapping_value);

  //float accel_xz, accel_yz, angle3;
  //const float RADIANS_TO_DEGREES = 180 / 3.14159;

  // 각도1 계산하기
  //accel_yz = sqrt(pow(ac_x, 2) + pow(ac_y, 2));
  //angle1 = atan(-ac_z / accel_yz) * RADIANS_TO_DEGREES; //ex
  
  //accel_yz = sqrt(pow(ac_x, 2) + pow(ac_z, 2));
  //angle1 = atan(-ac_y / accel_yz) * RADIANS_TO_DEGREES;  //Roll
  
  //accel_yz = sqrt(pow(ac_y, 2) + pow(ac_z, 2));
  //angle1 = atan(-ac_x / accel_yz) * RADIANS_TO_DEGREES;  //pitch
  
  //Serial.println("angle1 : ");
  //Serial.println(angle1);


  // 테스트 중
  angleAcX = atan(ac_y / sqrt(pow(ac_x, 2) + pow(ac_z, 2)));
  angleAcX *= RADIAN_TO_DEGREE;   // 좌, 우 (Roll)
  
  angleAcY = atan(-ac_x / sqrt(pow(ac_y, 2) + pow(ac_z, 2)));
  angleAcY *= RADIAN_TO_DEGREE;   // 앞, 뒤 (Pitch)

  anglemain = sqrt(pow(angleAcX, 2) + pow(angleAcY, 2));
  //anglemain *= RADIAN_TO_DEGREE;

  // 각도2 계산하기
  //accel_xz = sqrt(pow(ac_z, 2) + pow(ac_y, 2));  // roll
  //angle2 = atan(-ac_x / accel_xz) * RADIANS_TO_DEGREES;
  //Serial.println("angle2 : ");
  //Serial.println(angle2);
}

void value_init() {
  normal_x = 0; normal_x = 0; normal_x = 0;
  for (int i = 0; i < 3; i++) {
    deltha_x[i] = 0;
    deltha_y[i] = 0;
    deltha_z[i] = 0;
    angle1 = 0;
    angle2 = 0;
  }
}
