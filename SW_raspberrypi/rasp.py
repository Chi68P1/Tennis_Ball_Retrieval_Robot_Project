import cv2
import numpy as np
import serial

# Mở camera
cap = cv2.VideoCapture(0)

# Đặt kích thước mới cho frame
new_width, new_height = 320, 240

# Mở cổng UART
uart_port = '/dev/ttyACM0'  # Thay đổi cổng UART theo thiết bị thực tế
ser = serial.Serial(uart_port, 9600) # ls /dev/tty*

# Ngưỡng diện tích tối thiểu
min_contour_area = 100

while True:
    # Đọc frame từ camera và giảm kích thước
    ret, frame = cap.read()
    frame = cv2.resize(frame, (new_width, new_height))

    # Chuyển đổi frame sang không gian màu HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Xác định vùng màu xanh trong không gian màu HSV
    lower_pink = np.array([30, 50, 85])
    upper_pink = np.array([60, 255, 255])
    mask = cv2.inRange(hsv, lower_pink, upper_pink)

    # Tìm contours trong mask
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Lọc contours theo diện tích
    filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]

    # Tìm contour có diện tích lớn nhất
    max_contour = max(filtered_contours, key=cv2.contourArea, default=None)

    # Nếu có contour, tính toán tâm của hình chữ nhật và vẽ nó lên frame
    if max_contour is not None:
        x, y, w, h = cv2.boundingRect(max_contour)
        cX, cY = x + w // 2, y + h // 2
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (cX, cY), 5, (0, 255, 0), -1)

        # Đảm bảo b và c luôn có 3 chữ số
        formatted_x = f"{cX:03d}"
        formatted_y = f"{cY:03d}"

        # Tạo dữ liệu để gửi
        data_to_send = f"{formatted_x}{formatted_y}\n"

        # Gửi dữ liệu đến cổng UART
        ser.write(data_to_send.encode())

        print(f"Center coordinates: ({cX}, {cY})")
    else:
        # Tạo dữ liệu để gửi
        data_to_send = f"000000\n"

        # Gửi dữ liệu đến cổng UART
        ser.write(data_to_send.encode())

        print(f"Center coordinates: ({0}, {0})")


    # Hiển thị frame gốc và frame sau khi áp dụng mask
    cv2.imshow('Original Frame', frame)

    # Thoát nếu người dùng nhấn 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng camera và đóng cửa sổ hiển thị
cap.release()
cv2.destroyAllWindows()
