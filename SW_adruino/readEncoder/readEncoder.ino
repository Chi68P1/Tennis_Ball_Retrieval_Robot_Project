#include <Arduino.h>
#include <TimerOne.h> // Thư viện TimerOne

// Chọn chân cho encoder
const int encoderPinA = 2;  // Chân CLK hoặc A
const int encoderPinB = 3;  // Chân DT hoặc B

//Khai báo chân tín hiệu motor A
int enA = 11;
int in1 = 9;
int in2 = 8; 

//Khai báo chân tín hiệu cho motor B


int velocity;

// Biến
volatile long pulseCount = 0;
int previousEncoderState = 0;
volatile int PosCnt = 0;
volatile int CntVel = 0;

const int N = 1980;  // Số xung mỗi vòng quay
const unsigned long updateInterval = 5000;  // Cập nhật vận tốc mỗi giây (đơn vị micro giây)
unsigned long previousMillis = 0;

bool run = false;

float CurPos = 0;
float CurVel = 0;
float DesiredPos = 0;
float DesiredSpeed = 0;

float sampletime = 0.005;
float Kb = 21.50537634;
float alpha = 0.485;

int pwm;

void setup() {

  //Timer1.initialize(updateInterval); // Khởi tạo Timer 1 với chu kỳ 0.005 giây
  //Timer1.attachInterrupt(timerCallback); // Gắn hàm callback cho timer

  // Khởi tạo chân của encoder là INPUT_PULLUP để sử dụng résistor pull-up nội bộ
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); 

  // Khởi tạo kết nối Serial để hiển thị giá trị xung
  Serial.begin(115200);
  Serial.setTimeout(10); // Đặt timeout (ms) cho Serial để đọc dữ liệu từ Serial Monitor. Không set thì mặt định là 1s, tối thiểu 1ms


  // Sử dụng interrupts để đọc encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), handleEncoder, CHANGE);
}

void loop() {

  plotGraph1();
  plotGraph() ;
  // Hiển thị giá trị xung trên Serial Monitor
  //Serial.print("Pulse Count: ");
  //Serial.println(pulseCount);

  // Kiểm tra xem có dữ liệu đến trên cổng Serial không
  if (Serial.available() > 0) {
    // Đọc dữ liệu từ cổng UART
    char receivedChar = Serial.read();

    switch (receivedChar) {
      case 'e':
        run = false; // Dừng động cơ
        break;
      case 'r':
        run = true; // Cho phép động cơ chạy
        break;
      case 'v':
        // Đọc dữ liệu vị trí mong muốn
        if (run == true) {
          String inputString = Serial.readStringUntil('\0'); // Đọc dữ liệu sau 'v' cho tới kí tự null
          DesiredSpeed = inputString.toInt(); // Chuyển đổi thành số nguyên (rpm)
          //Serial.println(DesiredSpeed); // In ra màn hình giá trị của velocity
        }
      case 's':
        // Đọc dữ liệu vị trí mong muốn
        if (run == true) {
          String inputString = Serial.readStringUntil('\n'); // Đọc dữ liệu sau 'v' cho tới kí tự xuống dòng (println)
          DesiredPos = inputString.toInt(); // Chuyển đổi thành số nguyên (rad)
          Serial.println(DesiredPos); // In ra màn hình giá trị của velocity
        }
        break;
        break;
      default:
        // Xử lý dữ liệu khác nếu cần
        break;
    }
  }
}

void handleEncoder() {
  // Đọc trạng thái của chân encoder A và B
  int encoderStateA = digitalRead(encoderPinA);
  int encoderStateB = digitalRead(encoderPinB);

  // Kết hợp hai trạng thái để xác định hướng quay
  int encoderState = (encoderStateA << 1) | encoderStateB;

  // Xác định hướng quay
  if (encoderState != previousEncoderState) {
    if ((previousEncoderState == 1 && encoderState == 0) || (previousEncoderState == 3 && encoderState == 1) || (previousEncoderState == 0 && encoderState == 2) || (previousEncoderState == 2 && encoderState == 3)) {
      // Quay phải (tăng)
      pulseCount++;
    } else  {
      // Quay trái (giảm)
      pulseCount--;
    }
    previousEncoderState = encoderState;
    CntVel++;
    if (pulseCount>=1980) {
      pulseCount = 0;
      PosCnt++;
    }
    else if	(pulseCount<=-1980) {
      pulseCount = 0;
      PosCnt--;
    }
  }
}

float PID(float Desired, float Current, float p, float i, float d) {
	static float err_p = 0;
	static float iterm_p = 0;
	static float err_sat = 0;
	static float dterm_f_p = 0;
	
	float err, err_windup;
	float pterm, dterm, dterm_f, iterm;
	float pidterm, pid_sat;
	int16_t pidout;
	
	int HILIM = 255; 
  int LOLIM = -255;
	
	err = Desired - Current  ;

  pterm = p*err;
	
	dterm  = d*(err-err_p)/sampletime;

  dterm_f = alpha*dterm + (1-alpha)*dterm_f_p;

	iterm = iterm_p + i*err*sampletime;
	
	err_p = err;
	iterm_p = iterm;
  dterm_f_p = dterm_f;

	pidout = pterm + dterm_f + iterm;

  if (pidout > HILIM) {
    pidout = HILIM;
  }
  else if (pidout < LOLIM) {
    pidout = LOLIM;
  }
  else {
    if (pidout > 0) {
      pidout = 70;
    }
    else if (pidout < 0) {
      pidout = -70;
    }
  }  
	return pidout;
}

void motorDrive(float pwm) {
  if (run == true) {

    if (pwm > 0) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, pwm);
      //
    }
    else if (pwm < 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, -pwm);
      //
    }
    else {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, 0);
    }

    
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 0);
  }

}


void timerCallback() {

    //vị trí hiện tại 
  CurPos = PosCnt * 2 *3.14 + pulseCount * 2 * 3.14 / 1980;	 // rad

  // Tính số lượng xung mỗi giây
  float frequency = (float)CntVel / 0.005;

  // Tính RPM
  CurVel = (frequency / N) * 60;

  // Hiển thị RPM trên Serial Monitor

  //Serial.println(RPM);
  //Serial.println(run); 
  //Serial.println(DesiredSpeed);
  // Đặt lại pulseCount để tính lại RPM cho khoảng thời gian tiếp theo
  CntVel = 0;

  pwm = PID(DesiredPos,CurPos,1.150878385,0,0.053515845); // điều khiển vận tốc

  motorDrive(pwm);
}

void plotGraph1() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 50) { // 50ms
    previousMillis = currentMillis;

    if (run == true) {
      Serial.print("P");
      Serial.println(CurPos);
    }
  } 
}

void plotGraph() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 5) { // 50ms
    previousMillis = currentMillis;

    CurPos = PosCnt * 2 *3.14 + pulseCount * 2 * 3.14 / 1980;	 // rad

      // Tính số lượng xung mỗi giây
      float frequency = (float)CntVel / 0.005;

      // Tính RPM
      CurVel = (frequency / N) * 60;

      // Hiển thị RPM trên Serial Monitor

      //Serial.println(RPM);
      //Serial.println(run); 
      //Serial.println(DesiredSpeed);
      // Đặt lại pulseCount để tính lại RPM cho khoảng thời gian tiếp theo
      CntVel = 0;

      pwm = PID(DesiredPos,CurPos,1.150878385,0,0.053515845); // điều khiển vận tốc

      motorDrive(pwm);
  } 
}
