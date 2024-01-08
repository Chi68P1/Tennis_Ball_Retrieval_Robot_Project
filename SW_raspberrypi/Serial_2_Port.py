import serial

a = 1
b = 100
c = 100
d = 0

# Khởi tạo đối tượng serial cho cổng UART chạy
ser1 = serial.Serial('COM4', 9600)  # Thay 'COM4' bằng cổng UART thực tế trên máy tính của bạn

# Khởi tạo đối tượng serial cho cổng UART bắn tùm lum
#ser2 = serial.Serial('COM4', 9600)  # Thay 'COM5' bằng cổng UART thực tế trên máy tính của bạn

# Kiểm tra và hiển thị trạng thái kết nối cho cả hai cổng UART
if ser1.is_open:
    print("Cổng UART COM4 đang kết nối.")
else:
    print("Cổng UART COM4 không kết nối.")

#if ser2.is_open:
   # print("Cổng UART COM5 đang kết nối.")
#else:
   # print("Cổng UART COM5 không kết nối.")

try:
    # Gán giá trị cho a, b, c, d
    a = a 
    b = b  # Đổi giá trị của b tại đây
    c = c 
    d = d 

    # Đảm bảo b và c luôn có 3 chữ số
    formatted_b = f"{b:04d}"
    formatted_c = f"{c:04d}"

    # Tạo dữ liệu để gửi
    data_to_send_move = f"{a}{formatted_b}{formatted_c}\n"  # Thêm dấu xuống dòng để kết thúc dữ liệu
    data_to_send_shoot = f"{a}{formatted_b}{formatted_c}{d}\n"  # Thêm dấu xuống dòng để kết thúc dữ liệu

    # Gửi dữ liệu đến cổng UART 1
    ser1.write(data_to_send_shoot.encode())

    # Gửi dữ liệu đến cổng UART 2
    #ser2.write(data_to_send_shoot.encode())

except KeyboardInterrupt:
    # Handle Ctrl+C to gracefully exit the script
    print("\nKeyboardInterrupt: Stopping the script.")
finally:
    # Đóng cả hai cổng UART sau khi kết thúc chương trình
    ser1.close()
    #ser2.close()
