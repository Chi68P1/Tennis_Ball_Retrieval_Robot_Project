import serial

# Khởi tạo đối tượng serial với cổng UART tương ứng và tốc độ baud
ser = serial.Serial('COM9')  # Thay 'COMx' bằng cổng UART thực tế trên máy tính của bạn

if ser.is_open:
    print("Cổng UART COM4 đang kết nối.")
else:
    print("Cổng UART COM4 không kết nối.")

# Gửi số 115544
data_to_send = '111456\n'
ser.write(data_to_send.encode())

# Đóng cổng UART sau khi gửi xong
ser.close()
