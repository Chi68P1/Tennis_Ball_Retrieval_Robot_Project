import serial

# Khởi tạo đối tượng serial với cổng UART tương ứng và tốc độ baud
ser = serial.Serial('COM4', 9600)  # Thay 'COMx' bằng cổng UART thực tế trên máy tính của bạn

if ser.is_open:
    print("Cổng UART COM4 đang kết nối.")
else:
    print("Cổng UART COM4 không kết nối.")

try:
    while True:
        data = ser.readline().decode('utf-8')
        if data:
            print(data, end='')  # In dữ liệu từ cổng UART
except KeyboardInterrupt:
    ser.close()
