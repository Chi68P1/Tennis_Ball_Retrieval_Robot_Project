import usb.core
import usb.util

# Vendor ID và Product ID của thiết bị USB
vendor_id = 0x0483
product_id = 0x5740

# Tìm thiết bị USB với Vendor ID và Product ID tương ứng
dev = usb.core.find(idVendor=vendor_id, idProduct=product_id)

if dev is not None:
    print("Thiết bị USB đã được kết nối.")
    
    # Gửi dữ liệu
    data_to_send = b'Hello, USB!\n'
    dev.write(1, data_to_send)  # Thay 1 bằng endpoint tương ứng trên thiết bị
    
    # Đóng kết nối USB
    usb.util.dispose_resources(dev)
else:
    print("Không tìm thấy thiết bị USB.")
