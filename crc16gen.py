POLY = 0xA001


def crc16(data):
    for _ in range(8):
        data = (data >> 1) ^ POLY if data & 1 else data >> 1
    return data


print(", ".join("0x{:04X}".format(crc16(x)) for x in range(256)))
