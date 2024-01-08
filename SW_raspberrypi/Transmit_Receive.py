import serial
import keyboard
import threading

# Khởi tạo đối tượng serial với cổng UART tương ứng và tốc độ baud
ser = serial.Serial('COM4', 9600)  # Thay 'COMx' bằng cổng UART thực tế trên máy tính của bạn

if ser.is_open:
    print("Cổng UART COM4 đang kết nối.")
else:
    print("Cổng UART COM4 không kết nối.")

received_data = ""  # Biến lưu trữ dữ liệu nhận được

def on_key_event(e):
    if e.event_type == keyboard.KEY_DOWN:
        key = e.name
        data_to_send = key + '\n'  # Thêm xuống hàng vào dữ liệu
        ser.write(data_to_send.encode())

def receive_data():
    global received_data
    try:
        while True:
            data = ser.readline().decode('utf-8')
            if data:
                received_data = data

    except KeyboardInterrupt:
        print("Dừng nhận dữ liệu từ cổng UART.")

keyboard_thread = threading.Thread(target=keyboard.hook, args=(on_key_event,))
receive_thread = threading.Thread(target=receive_data)

keyboard_thread.start()
receive_thread.start()

try:
    while True:
        if received_data:
            print(received_data, end='')  # In dữ liệu mới
            received_data = ""  # Xóa dữ liệu đã in

except KeyboardInterrupt:
    ser.close()
