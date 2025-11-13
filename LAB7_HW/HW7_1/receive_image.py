import serial

ser = serial.Serial('COM4', 115200, timeout=1)  # 將 COM3 改成你的實際 COM port

# 讀取檔案大小
size = int(ser.readline().decode().strip())
print("Expecting bytes:", size)

# 讀取檔案資料
data = ser.read(size)

with open("recv.jpg", "wb") as f:
    f.write(data)

print("Image received!")
