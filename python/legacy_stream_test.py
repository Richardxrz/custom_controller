
def most_common(bytes):
    lists = []
    counts = []
    for i in range(len(bytes)-3):
        temp_list = bytes[i:i+4]
        print("temp_list: ", temp_list)
        if temp_list not in lists:
            lists.append(temp_list)
            counts.append(1)
        else:
            index = lists.index(temp_list)
            counts[index] += 1
    top5_idx = [i for i, v in sorted(enumerate(counts), key=lambda x: x[1], reverse=True)[:5]]
    return lists, counts, top5_idx

def decode_bytes(bytes):
    # 根据软件协议提取单帧数据（去除帧尾）
    index = bytes.find(b'\x00\x00\x80\x7f')  # 查找帧尾标志
    tail = bytes[index:index+4]  # 提取帧尾数据
    frame_data  = bytes[:index]  # 提取帧数据
    bytes = bytes[index+4:]  # 更新缓存区数据，去除已提取的数据
    return frame_data, bytes

# 将读取到的数据通过不同形式呈现
def print_frame_data(data):
    if data is None:
        print("No data received.")
        return
    print("Received data (bytes):", data)
    print("Received data (hex):", data.hex())
    print("Received data (string):", data.decode(errors='ignore'))


if __name__ == "__main__":
    raw_data =  "CF E4 6C 3E 4A 92 71 3F DC D1 49 3F 60 DF BB BD 00 00 80 7F 4A 92 A7 3E 72 D2 78 3F B0 17 39 3F A6 3D 43 BE 00 00 80 7F C2 05 D7 3E 74 96 7D 3F"
    bytes = bytes.fromhex(raw_data)
    print("这是hex转换成字节类型：",bytes)
    lists, counts, top5_index_list = most_common(bytes)
    print("lists: ", lists)
    print("top5 sequence index: ", top5_index_list)
    print("top5 sequence: ")
    for i in top5_index_list:
        print("sequence: ",lists[i], "; count: ", counts[i])