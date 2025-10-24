from bluepy.btle import Peripheral, DefaultDelegate, Scanner, UUID
import time
import sys
import select
import tty
import termios

TARGET_NAME = "BlueNRG"

target_char_uuids = [
    UUID("00110000-0001-11E1-AC36-0002A5D5C51B"),
    UUID("00220000-0001-11E1-AC36-0002A5D5C51B"),
    UUID("00330000-0001-11E1-AC36-0002A5D5C51B")
]

write_char_uuid = UUID("56780000-0001-11E1-AC36-0002A5D5C51B")

class MyDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleNotification(self, cHandle, data):
        if len(data) >= 2:
            value = int.from_bytes(data, byteorder='little', signed=True)
            print(f"📥 Notification from handle {cHandle}: {value}")
        else:
            print(f"📥 Notification from handle {cHandle}: {data}")

def is_key_pressed(timeout=0.1):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    return dr

def enable_raw_mode():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return fd, old_settings

def restore_terminal(fd, old_settings):
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def toggle_notify(dev, char, enable=True):
    try:
        cccd = char.getDescriptors(forUUID=0x2902)[0]
        val = b'\x01\x00' if enable else b'\x00\x00'
        dev.writeCharacteristic(cccd.handle, val, withResponse=True)
        print(f"{'✅ 開啟' if enable else '⏸️ 暫停'} Notify: {char.uuid}")
    except Exception as e:
        print(f"⚠️ 切換通知時出錯: {e}")

# BLE 掃描與連線
scanner = Scanner()
print("🔍 掃描裝置中...")
devices = scanner.scan(5.0)

target_addr = None
for dev in devices:
    for (adtype, desc, value) in dev.getScanData():
        if desc == "Complete Local Name" and value == TARGET_NAME:
            target_addr = dev.addr
            print(f"✅ 找到 {TARGET_NAME}，MAC = {target_addr}")
            break
    if target_addr:
        break

if not target_addr:
    print("❌ 找不到裝置")
    sys.exit(1)

try:
    dev = Peripheral(target_addr, 'random')
    dev.setDelegate(MyDelegate())

    notify_chars = []
    write_char = None

    print("🔗 已連線，啟用通知中...")
    for svc in dev.getServices():
        for char in dev.getCharacteristics(startHnd=svc.hndStart, endHnd=svc.hndEnd):
            if char.uuid in target_char_uuids:
                toggle_notify(dev, char, True)
                notify_chars.append(char)
            if char.uuid == write_char_uuid:
                write_char = char

    if not write_char:
        raise Exception("❌ 找不到可寫入的 characteristic")

    print("📡 等待通知中，按 F 輸入 16 進制值（會暫停通知），Ctrl+C 離開\n")

    fd, old_settings = enable_raw_mode()

    while True:
        if is_key_pressed():
            key = sys.stdin.read(1)
            if key.lower() == 'f':
                # 暫停所有通知
                for c in notify_chars:
                    toggle_notify(dev, c, False)

                try:
                    hex_input = input("📝 請輸入要寫入的 16 進位值（例如 1A2B）：").strip()
                    if not all(c in "0123456789ABCDEFabcdef" for c in hex_input):
                        raise ValueError("❌ 包含無效字元")
                    if len(hex_input) != 4:
                        raise ValueError("❌ 請輸入 2 個位元組（4 個 16 進位字元）")

                    data = bytes.fromhex(hex_input)
                    write_char.write(data, withResponse=True)
                    print(f"✅ 已寫入 0x{hex_input.upper()}，原始位元組：{data.hex(' ')}")
                except Exception as e:
                    print(f"⚠️ 輸入或寫入錯誤：{e}")

                # 恢復通知
                for c in notify_chars:
                    toggle_notify(dev, c, True)
        else :
            if dev.waitForNotifications(0.3):
                continue

except KeyboardInterrupt:
    print("\n🛑 使用者中斷")

except Exception as e:
    print(f"❌ 發生錯誤: {e}")

finally:
    restore_terminal(fd, old_settings)
    dev.disconnect()
    print("🔌 已斷線")
