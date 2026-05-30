import struct
import os

def compare_datasets(f1, f2):
    d1 = open(f1, 'rb').read()
    d2 = open(f2, 'rb').read()
    
    ptrs1 = struct.unpack('<' + 'I' * 128, d1[:512])
    ptrs2 = struct.unpack('<' + 'I' * 128, d2[:512])
    
    print(f"Comparing {f1} and {f2}")
    for i in range(128):
        p1 = ptrs1[i]
        p2 = ptrs2[i]
        
        if p1 == 0 or p2 == 0:
            continue
            
        # Pointers are relative to 0x5E000
        off1 = p1 - 0x5E000
        off2 = p2 - 0x5E000
        
        if off1 < 0 or off1 >= len(d1) or off2 < 0 or off2 >= len(d2):
            continue
            
        # Read map header (usually 1 byte length, or similar)
        # We'll just compare the data at the offset
        len1 = 64 # arbitrary read
        data1 = d1[off1:off1+len1]
        data2 = d2[off2:off2+len1]
        
        if data1 != data2:
            print(f"Map {i:02x} (Off {off1:04x} vs {off2:04x}) DIFFERS")
            if all(b == 0 for b in data1):
                print(f"  TT Map {i:02x} is all ZEROS")
            if all(b == 0 for b in data2):
                print(f"  Golf Map {i:02x} is all ZEROS")

if __name__ == "__main__":
    compare_datasets('datasets/tt/tt_dataset_239.bin', 'datasets/golf/v074N2511K0___Lenkung.bin')
