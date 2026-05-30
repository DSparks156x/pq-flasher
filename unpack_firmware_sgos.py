import sys, struct, os

def getWord32(img, pos):
    return struct.unpack("<I", img[pos:pos+4])[0]

def getWord24(img, pos):
    return struct.unpack(">I", b"\x00" + img[pos:pos+3])[0]

def decodeXOR(slice, key):
    slicexor = bytearray(len(slice))
    for nr in range(len(slice)):
        slicexor[nr] = slice[nr]^key[nr % len(key)]
    return slicexor

def decodeBCB(payload, length):
    p = 0
    if payload.startswith(b"\x1A\x01"):
        p = 2
    else:
        try:
            p = payload.index(b"\x1A\x01") + 2
        except ValueError:
            return payload[:length]

    res = bytearray()
    while p < len(payload):
        if p + 2 > len(payload): break
        l = struct.unpack(">H", payload[p:p+2])[0]
        p += 2
        fl = l >> 14
        l &= 0x3FFF
        if fl == 0: # literal
            res += payload[p : p + l]
            p += l
        elif fl == 1: # RLE
            res += bytearray([payload[p]] * l)
            p += 1
        elif fl == 3: # End
            break
    return res[:length]

def unpack_sgo(filename):
    print(f"Unpacking {filename}...")
    with open(filename, 'rb') as f:
        img = f.read()

    metastart = getWord32(img, 0x29)
    sgolen = getWord32(img, 0x2D)
    metalen = getWord32(img, metastart)
    blockstart = metastart + metalen + 4

    # We want to reconstruct a linear binary. 
    # V850 usually starts at 0, but these sgos might be offsets.
    # Let's just find the max address to see how much we need.
    max_addr = 0
    sections = []
    
    temp_blockstart = blockstart
    while temp_blockstart < sgolen:
        addr = getWord24(img, temp_blockstart)
        length = getWord24(img, temp_blockstart+0x4)
        sgoseclen = getWord32(img, temp_blockstart+0x15)
        sections.append((addr, length, temp_blockstart + 0x19, sgoseclen, img[temp_blockstart+0x3]))
        max_addr = max(max_addr, addr + length)
        temp_blockstart = temp_blockstart + 0x19 + sgoseclen

    print(f"Total sections: {len(sections)}, Max address: 0x{max_addr:08X}")
    
    # Create a buffer for the whole firmware
    # Based on the user's hint, sgos are 344kb (0x56000 bytes)
    # Full dump is 384kb (0x60000 bytes)
    # 0x56000 seems like a common size for the application area.
    
    # We'll use 0x60000 as a safe buffer size if we want to match full dump offsets,
    # but sgo addresses might be absolute.
    
    # In 02_patcher.py, addresses are around 0x5Dxxx, so they are likely absolute.
    # If it's a 384KB rack, memory is 0x00000 to 0x5FFFF.
    
    buffer = bytearray([0xFF] * 0x60000)
    
    for addr, length, blob_start, blob_len, crypt in sections:
        blob = img[blob_start : blob_start + blob_len]
        xor_data = decodeXOR(blob, b"\xFF")
        if crypt == 1:
            unpacked = decodeBCB(xor_data, length)
        else:
            unpacked = xor_data[:length]
        
        print(f"  Writing 0x{len(unpacked):06X} bytes to 0x{addr:06X}")
        if addr + len(unpacked) <= len(buffer):
            buffer[addr : addr + len(unpacked)] = unpacked
        else:
            print(f"  Warning: address 0x{addr:06X} is out of buffer range!")

    output_filename = filename.replace(".sgo", ".bin")
    with open(output_filename, 'wb') as out_f:
        out_f.write(buffer)
    print(f"Saved to {output_filename}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python unpack_firmware_sgos.py <file1.sgo> [file2.sgo] ...")
    else:
        for f in sys.argv[1:]:
            unpack_sgo(f)
