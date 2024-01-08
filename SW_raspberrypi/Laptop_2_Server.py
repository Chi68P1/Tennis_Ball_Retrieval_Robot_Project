import requests
import keyboard

url = 'http://192.168.204.63:5000/send_key'

def send_key_event_to_server(e):
    try:
        if e.event_type in [keyboard.KEY_DOWN, keyboard.KEY_UP]:
            # Chỉ gửi yêu cầu HTTP khi có sự kiện bàn phím
            event_type = 'down' if e.event_type == keyboard.KEY_DOWN else 'up'
            data = {'key': e.name, 'event_type': event_type}
            response = requests.post(url, data=data)
            print(response.text)

    except Exception as ex:
        print(f"Error: {ex}")

# Đăng ký hàm xử lý sự kiện khi có phím được nhấn hoặc thả ra
keyboard.hook(send_key_event_to_server)

# Giữ chương trình chạy
keyboard.wait('esc')  # Đợi cho đến khi phím Esc được nhấn
