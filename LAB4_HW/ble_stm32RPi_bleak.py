import asyncio
from bleak import BleakScanner, BleakClient
import matplotlib.pyplot as plt
import msvcrt
import signal
import sys

TARGET_NAME = "BlueNRG"
TARGET_SAMPLE_COUNT = 100  # 每軸收集的樣本數

# UUIDs for 原始資料（oridata）與 濾波資料（MVdata）
oridata_char_uuids = [
    "00110000-0001-11E1-AC36-0002A5D5C51B",  # X
    "00220000-0001-11E1-AC36-0002A5D5C51B",  # Y
    "00330000-0001-11E1-AC36-0002A5D5C51B"   # Z
]
MVdata_char_uuids = [
    "AA110000-0001-11E1-AC36-0002A5D5C51B",  # X
    "AA220000-0001-11E1-AC36-0002A5D5C51B",  # Y
    "AA330000-0001-11E1-AC36-0002A5D5C51B"   # Z
]

# 初始化資料儲存區，UUID 字串轉小寫統一比對
oridata = {uuid.lower(): [] for uuid in oridata_char_uuids}
MVdata = {uuid.lower(): [] for uuid in MVdata_char_uuids}

def parse_value(data):
    if len(data) >= 2:
        return int.from_bytes(data, byteorder='little', signed=True)
    return None

# 接收 Notification 並儲存資料
async def notification_handler(uuid, data_dict):
    async def inner(sender, data):
        uuid_str = str(sender.uuid).lower()
        value = parse_value(data)
        if value is not None and uuid_str in data_dict:
            if len(data_dict[uuid_str]) < TARGET_SAMPLE_COUNT:
                data_dict[uuid_str].append(value)
    return inner

async def main():
    print("🔍 掃描裝置中...")
    devices = await BleakScanner.discover(timeout=5.0)
    target_device = next((d for d in devices if d.name == TARGET_NAME), None)

    if not target_device:
        print("❌ 找不到裝置")
        return

    print(f"✅ 找到 {TARGET_NAME}，MAC = {target_device.address}")

    async with BleakClient(target_device.address) as client:
        print("🔗 已連線，啟用通知中...")

        # 開啟所有通知並註冊對應處理器
        for uuid in oridata_char_uuids:
            await client.start_notify(uuid, await notification_handler(uuid.lower(), oridata))
            print(f"📡 原始資料 Notify 開啟: {uuid}")
        for uuid in MVdata_char_uuids:
            await client.start_notify(uuid, await notification_handler(uuid.lower(), MVdata))
            print(f"📡 濾波後資料 Notify 開啟: {uuid}")

        # 等待資料收集完成
        print("⏳ 接收資料中（每軸 1000 筆）...")
        while not all(len(oridata[u]) >= TARGET_SAMPLE_COUNT for u in oridata) or \
              not all(len(MVdata[u]) >= TARGET_SAMPLE_COUNT for u in MVdata):
            await asyncio.sleep(0.1)

        # 關閉通知
        print("🛑 資料接收完成，關閉 Notify")
        for uuid in oridata_char_uuids + MVdata_char_uuids:
            await client.stop_notify(uuid)

    # 繪圖比較
    print("📊 開始畫圖...")
    labels = ['X', 'Y', 'Z']
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))

    for i in range(3):
        uuid_ori = oridata_char_uuids[i].lower()
        uuid_mv = MVdata_char_uuids[i].lower()
        axs[i].plot(oridata[uuid_ori], label="原始資料")
        axs[i].plot(MVdata[uuid_mv], label="濾波後資料")
        axs[i].set_title(f"{labels[i]} 軸加速度比較")
        axs[i].legend()
        axs[i].grid(True)

    plt.tight_layout()
    plt.show()

def signal_handler(sig, frame):
    print("\n🛑 強制中斷")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    asyncio.run(main())
