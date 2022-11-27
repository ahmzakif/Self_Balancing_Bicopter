
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

Servo right_prop;
Servo left_prop;

/*MPU-6050 memberi kita data 16 bit sehingga kita harus membuat beberapa konstanta 16int
  untuk menyimpan data untuk akselerasi dan gyro*/

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float AccRawXc, AccRawYc, AccRawZc, GyRawXc, GyRawYc, GyRawZc;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, time, timePrev;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

//  PID CONSTANTS
double kp=4.50;
double ki=0.015;
double kd=3.00;

double throttle=1200; //nilai awal throttle ke motor
float desired_angle = 0; //sudut di mana kita ingin keseimbangan tetap stabil
float xFilter, yFilter; //pengaplikasian kalman filter

float Xt, Xt_update, Xt_prev;
float Pt, Pt_update, Pt_prev;
float Kt, R=100, Q=1;
float a_kalman;

//KALMAN FILTER
float kalman(float a)
{
  Xt_update = Xt_prev;
  Pt_update = Pt_prev + Q;
  Kt = Pt_update/(Pt_update + R);
  Xt = Xt_update + (Kt*(a - Xt_update));
  Pt = (1 - Kt)*Pt_update;
  Xt_prev = Xt;
  Pt_prev = Pt;
  a_kalman = Xt;

  return a_kalman;
}

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                         //Start communicating with the MPU-6050
  Wire.write(0x6B);                                     //Send the requested starting register
  Wire.write(0);                                        //Set the requested starting register
  Wire.endTransmission(true);                           //End the transmission
  
}

void read_mpu_6050_acc(){                                              
  Wire.beginTransmission(0x68);                         //Start communicating with the MPU-6050
  Wire.write(0x3B);                                     //Send the requested starting register
  Wire.endTransmission(false);                          //End the transmission
  Wire.requestFrom(0x68,6,true);                        //Request 6 register from the MPU-6050
  
  Acc_rawX=Wire.read()<<8|Wire.read();                  //Add the low and high byte to the acc raw x variable
  Acc_rawY=Wire.read()<<8|Wire.read();                  //Add the low and high byte to the acc raw y variable
  Acc_rawZ=Wire.read()<<8|Wire.read();                  //Add the low and high byte to the acc raw z variable
}

void read_mpu_6050_gyro(){
  Wire.beginTransmission(0x68);                         //Start communicating with the MPU-6050
  Wire.write(0x43);                                     //Send the requested starting register
  Wire.endTransmission(false);                          //End the transmission
  Wire.requestFrom(0x68,4,true);                        //Request 4 register from the MPU

  Gyr_rawX=Wire.read()<<8|Wire.read();                  //Add the low and high byte to the gyro raw x variable
  Gyr_rawY=Wire.read()<<8|Wire.read();                  //Add the low and high byte to the gyro raw y variable
  Gyr_rawZ=Wire.read()<<8|Wire.read();                  //Add the low and high byte to the gyro raw z variable
}

void calibrate_mpu6050(){
  for (int cal_int = 0; cal_int < 2000; cal_int++){

    read_mpu_6050_acc();      
    AccRawXc += Acc_rawX;     //Menambah offset sumbu x acceleration ke variabel kalibrasi
    AccRawYc += Acc_rawY;     //Menambah offset sumbu y acceleration ke variabel kalibrasi
    AccRawZc += Acc_rawZ;     //Menambah offset sumbu z acceleration ke variabel kalibrasi
    
    read_mpu_6050_gyro();
    GyRawXc += Gyr_rawX;     //Menambah offset sumbu x gyro ke variabel kalibrasi
    GyRawYc += Gyr_rawY;     //Menambah offset sumbu y gyro ke variabel kalibrasi
    GyRawZc += Gyr_rawZ;     //Menambah offset sumbu z gyro ke variabel kalibrasi
    
    Serial.print("Calibrate: ");
    Serial.println(cal_int);
    delay(3);
  }
  //Membagi variabel kalibrasi dengan 2000 untuk mendapatkan offset rata-rata
  AccRawXc /= 2000;
  AccRawYc /= 2000;
  AccRawZc /= 2000;
  GyRawXc /= 2000;
  GyRawYc /= 2000;
  GyRawZc /= 2000;
  
}

void setup() {
  Wire.begin(); //begin the wire comunication
  setup_mpu_6050_registers();
  Serial.begin(115200); //baud rate
  right_prop.attach(3); //attach the right motor to pin 
  left_prop.attach(5);  //attach the left motor to pin 


  time = millis(); //Memulai menghitung waktu dalam milliseconds
  /*Untuk memulai ESC, kita harus mengirim nilai min
   * dari PWM ke ESC sebelum menghubungkan baterai. Jika tidak
   * ESC tidak akan memulai atau masuk ke dalam mode konfigurasi.
   * Nilai min adalah 1000us dan max adalah 2000us,*/

  left_prop.writeMicroseconds(2000);
  delay(1000); 
  left_prop.writeMicroseconds(1000);

  right_prop.writeMicroseconds(2000);
  delay(1000);
  right_prop.writeMicroseconds(1000);
  delay(1000);
  
  calibrate_mpu6050();
  delay(5000); 
  
}//end of setup void

void loop() {

    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; 
  
  /*TimeStep adalah waktu yang telah berlalu sejak perulangan sebelumnya. 
   * Ini adalah nilai yang akan kita gunakan dalam rumus sebagai "elapsedTime" 
   * dalam hitungan detik. Kita bekerja dalam ms sehingga kita harus membagi nilai tersebut dengan 1000 
    to obtain seconds*/

    Acc_rawX -= AccRawXc;     //Mengurangi offset calibration value Dari data raw acc x 
    Acc_rawY -= AccRawYc;     //Mengurangi offset calibration value Dari data raw acc y 
    Acc_rawZ -= AccRawZc;     //Mengurangi offset calibration value Dari data raw acc z 

    Gyr_rawX -= GyRawXc;      //Mengurangi offset calibration value Dari data raw gyro x 
    Gyr_rawY -= GyRawYc;      //Mengurangi offset calibration value Dari data raw gyro y 
    Gyr_rawZ -= GyRawZc;      //Mengurangi offset calibration value Dari data raw gyro z 

    /*Bagian di mana kita harus menghitung sudut-sudut dengan menggunakan persamaan Euler*/
    /* - Sekarang, untuk mendapatkan nilai akselerasi dalam unit "g", pertama-tama kita harus membagi nilai mentah   
     * Nilai-nilai yang baru saja kita baca dengan 16384.0 karena itulah nilai yang diberikan oleh MPU6050 
     * dari datasheet.*/
    /* - Selanjutnya kita harus menghitung nilai radian ke derajat dengan membagi 180º dengan angka PI
    * yaitu 3.141592654 dan menyimpan nilai ini dalam variabel rad_to_deg. Agar tidak perlu
    * untuk menghitung nilai ini di setiap loop, kita telah melakukannya sekali saja sebelum setup void.
    */

    /* Sekarang kita bisa menerapkan rumus Euler. Atan akan menghitung arctangent. Pow(a,b)
     * pow(a,b) akan meningkatkan nilai a ke pangkat b. Dan akhirnya fungsi sqrt
     * akan menghitung akar kuadrat.*/
    read_mpu_6050_acc();
    /*---X---*/
    Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    /*---Y---*/
    Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;


  /*Sekarang untuk mendapatkan data gyro dalam derajat/sekon kita harus membagi terlebih dahulu
  nilai mentah dengan 131 karena itu nilai yang diberikan datasheet kepada kita*/
  read_mpu_6050_gyro();
  /*---X---*/
  Gyro_angle[0] = Gyr_rawX/131.0; 
  /*---Y---*/
  Gyro_angle[1] = Gyr_rawY/131.0;

  /*Sekarang untuk mendapatkan derajat, kita harus mengalikan derajat/detik
   *dengan elapsedTime.*/
  /*akhirnya kita dapat menerapkan filter akhir di mana kita menambahkan akselerasi
   *bagian yang mempengaruhi sudut dan tentu saja dikalikan dengan 0,98 */

  /*---X axis angle---*/
  Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
  /*---Y axis angle---*/
  Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
  
  /*PENGAPLIKASIAN KALMAN FILTER*/
  xFilter = (kalman(Total_angle[0])*100);
  yFilter = (kalman(Total_angle[1])*100);

  /*PRINT TO SERIAL MONITOR*/
  Serial.print("Angle: ");
  Serial.print(xFilter);
  Serial.print(" ");
  Serial.print("Left: ");
  Serial.print(pwmLeft);
  Serial.print(" ");
  Serial.print("Right: ");
  Serial.print(pwmRight);
  Serial.print(" ");
  Serial.print("PID: ");
  Serial.print(PID);
  Serial.print(" ");
  Serial.print("Error: ");
  Serial.println(error);
  
  
/*///////////////////////////P I D///////////////////////////////////*/

/*Pertama-tama hitung error antara sudut yang diinginkan dan 
*sudut yang sebenarnya diukur (filtered data)*/
error = xFilter - desired_angle;

/*Selanjutnya nilai proporsional PID hanya konstanta proporsional
*dikalikan dengan error*/
pid_p = kp*error;

/* Bagian integral seharusnya hanya bertindak jika kita dekat dengan
posisi yang diinginkan tetapi kita ingin menyetel error dengan baik. Itu
mengapa perlu dibuat operasi if untuk error antara -3 dan 3 derajat.
Untuk mengintegrasikan, kita hanya menjumlahkan nilai integral sebelumnya dengan
error dikalikan dengan konstanta integral. Ini akan mengintegrasikan (meningkatkan)
nilai setiap putaran sampai kita mencapai titik 0*/
if(-3 <error && error <3){
  pid_i = pid_i+(ki*error);
}

/* Bagian terakhir adalah derivate. Derivate bertindak atas kecepatan error.
Seperti yang kita ketahui kecepatan adalah jumlah error yang dihasilkan dalam 
jumlah tertentu,waktu tertentu dibagi dengan waktu tersebut. Untuk itu kita akan 
menggunakan variabel yang disebut previous_error.Kita substraksikan nilai itu 
dari aktual error dan membagi semua dengan waktu yang telah berlalu. 
Akhirnya kita kalikan hasilnya dengan konstanta derivative*/
pid_d = kd*((error - previous_error)/elapsedTime);

/*Nilai PID akhir adalah jumlah dari masing-masing 3 bagian ini*/
PID = pid_p + pid_i + pid_d;

/*Kita tahu bahwa nilai min dari sinyal PWM adalah 1000us dan max adalah 2000. Jadi 
itu memberi tahu kita bahwa nilai PID dapat berosilasi lebih dari -1000 dan 1000 karena 
ketika kita memiliki nilai 2000us nilai maksimum yang bisa kita kurangi adalah 1000 
dan ketika kita memiliki nilai 1000us untuk sinyal PWM, nilai maksimum yang bisa kita 
tambahkan adalah 1000 untuk mencapai maksimum 2000us*/
if(PID < -1000)
{
  PID=-1000;
}
if(PID > 1000)
{
  PID=1000;
}

/* menghitung lebar PWM. dengan menjumlahkan throttle 
yang diinginkan dan nilai PID*/
pwmLeft = throttle - (PID*150/100);
pwmRight = throttle + PID;

/* Selanjutnya melakukan threshold nilai PWM untuk memastikan bahwa 
kita tidak akan melewati nilai minimal dan nilai maksimal.*/

//Right
if(pwmRight < 1000)
{
  pwmRight= 1000;
}
if(pwmRight > 1500)
{
  pwmRight=1500;
}

//Left
if(pwmLeft < 1000)
{
  pwmLeft= 1000;
}
if(pwmLeft > 1500)
{
  pwmLeft=1500;
}

/* Dengan menggunakan fungsi servo, kita membuat pulsa PWM dengan perhitungan
lebar untuk setiap pulsa */

left_prop.writeMicroseconds(pwmLeft);
right_prop.writeMicroseconds(pwmRight);
previous_error = error; //Mengingat untuk menyimpan error sebelumnya.

}//end of loop void
