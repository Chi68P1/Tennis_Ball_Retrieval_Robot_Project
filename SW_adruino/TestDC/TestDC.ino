//Khai báo chân tín hiệu motor A
int enA = 5;
int in1 = 6;
int in2 = 7; 

//Khai báo chân tín hiệu cho motor B
int in3 = 8;
int in4 = 9; 
int enB = 10;

bool run = false;
int velocity,pwm;

void setup() {

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); 
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); 

    // Khởi tạo kết nối Serial để hiển thị giá trị xung
  Serial.begin(9600);
}

void loop() {
  // Kiểm tra xem có dữ liệu đến trên cổng Serial không
  if (Serial.available() > 0) {
    // Đọc dữ liệu từ cổng UART
    char receivedChar = Serial.read();
    switch (receivedChar) {
      case 'e':
        run = false; // Dừng động cơ
        Serial.println("Stop");
        break;
      case 'r':
        run = true; // Cho phép động cơ chạy
        Serial.println("Run");
        break;
      case 'v':
        // Đọc dữ liệu vị trí mong muốn
        if (run == true) {
          String inputString = Serial.readStringUntil('\0'); 
          velocity = inputString.toInt(); 
          pwm = velocity*255/100; // % pwm
          Serial.println(pwm);
        }
        break;
      default:
        // Xử lý dữ liệu khác nếu cần
        break;
    }
  }

  motorDrive(pwm);
}
void motorDrive(int pwm) {
  if (run == true) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, pwm);

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, pwm);
  }
  else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, 0);
  }
}
