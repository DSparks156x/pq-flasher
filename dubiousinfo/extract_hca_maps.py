import struct

def extract_map(data, ptr):
    off = ptr - 0x5E000
    if off < 0 or off >= len(data):
        return None
    
    count = data[off]
    if count == 0:
        return None
        
    axis = [data[off + 1 + i] for i in range(count)]
    values = [data[off + 1 + count + i] for i in range(count)]
    
    return {"axis": axis, "values": values}

def main():
    with open('datasets/tt/tt_dataset_239.bin', 'rb') as f:
        d = f.read()
    
    print("TT Dataset 239 HCA Maps:")
    for i in [0x3F, 0x40, 0x41]:
        ptr = struct.unpack('<I', d[i*4 : i*4+4])[0]
        m = extract_map(d, ptr)
        if m:
            print(f"Map {i:02x}:")
            print(f"  Speed (km/h): {m['axis']}")
            print(f"  Gains (count): {m['values']}")
            
            # Calculate command to hit 380 internal counts
            # CAN = (380 * 128) / Gain
            commands = [round((380 * 128) / g) for g in m['values']]
            print(f"  Cmd to hit 380: {commands}")

if __name__ == "__main__":
    main()
