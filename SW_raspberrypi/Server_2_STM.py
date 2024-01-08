from flask import Flask, render_template, Response, request
import serial
import cv2

app = Flask(__name__)
RunSystem = RightVel = LeftVel = EnablePickup = 0
cap = cv2.VideoCapture(0)  # Số 0 thường là camera mặc định trên máy tính, có thể là 1, 2,...

def send_data_to_uart(a, b, c, d, uart_port):
    try:
        # Khởi tạo đối tượng serial cho cổng UART
        ser = serial.Serial(uart_port)

        # Đảm bảo b và c luôn có 3 chữ số
        formatted_b = f"{b:04d}"
        formatted_c = f"{c:04d}"

        # Tạo dữ liệu để gửi
        data_to_send = f"{a}{formatted_b}{formatted_c}{d}\n"

        # Gửi dữ liệu đến cổng UART
        ser.write(data_to_send.encode())

    except Exception as ex:
        print(f"Error in send_data_to_uart: {ex}")

def generate_frames():
    while True:
        success, frame = cap.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def handle_key_event(key, event_type):
    global RunSystem, RightVel, LeftVel, EnablePickup

    # Kiểm tra phím và loại sự kiện
    if event_type == 'down':
        if key == 'a':
            RightVel = 3
            LeftVel = -3
            text = "Left!"
        elif key == 's':
            RightVel = -10
            LeftVel = -10
            text = "Back!"
        elif key == 'd':
            RightVel = -3
            LeftVel = 3
            text = "Right!"
        elif key == 'w':
            RightVel = 10
            LeftVel = 10
            text = "Forward!"
        elif key == 'r':
            RunSystem = 1
            text = "Run System!"
        elif key == 'space':
            RunSystem = 0
            text = "OFF System!"
        elif key == 'e':
            EnablePickup = 1
            text = "Enable Pickup!"
        elif key == 'o':
            EnablePickup = 0
            text = "Disable Pickup!"
        else:
            text = "Invalid"
    elif event_type == 'up':  # Handling key release events
        if key in ['a', 's', 'd', 'w', 'r', 'space', 'e', 'o']:
            RightVel = 0
            LeftVel = 0
            text = ""
        else:
            text = "Invalid"

    send_data_to_uart(RunSystem, RightVel, LeftVel, EnablePickup, 'COM4')
    return text

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/send_key', methods=['POST'])
def receive_key_event():
    try:
        key = request.form.get('key')
        event_type = request.form.get('event_type')

        text = handle_key_event(key, event_type)
        
        return text

    except Exception as ex:
        return f"Error: {ex}"

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
