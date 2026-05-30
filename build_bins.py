#!/usr/bin/env python3
import os
import struct
import crcmod

# Define CRC16 function matching 02_patcher.py
def crc16(dat):
    xmodem_crc_func = crcmod.mkCrcFun(0x11021, rev=False, initCrc=0x0000, xorOut=0x0000)
    crc = xmodem_crc_func(dat)
    return struct.pack(">H", crc)

def build_bin(base_path, dataset_path, output_path):
    print(f"Building {output_path}...")
    
    # 1. Read base firmware
    if not os.path.exists(base_path):
        raise FileNotFoundError(f"Base firmware not found: {base_path}")
    with open(base_path, "rb") as f:
        base_fw = bytearray(f.read())
        
    # 2. Read dataset
    if not os.path.exists(dataset_path):
        raise FileNotFoundError(f"Dataset file not found: {dataset_path}")
    with open(dataset_path, "rb") as f:
        dataset = f.read()
        
    # 3. Validate dataset
    if len(dataset) != 4096:
        raise ValueError(f"Expected dataset size of 4096 bytes, but got {len(dataset)} for {dataset_path}")
        
    # Compute CRC of first 4094 bytes
    dataset_data = dataset[:4094]
    expected_crc = dataset[4094:4096]
    computed_crc = crc16(dataset_data)
    
    if expected_crc != computed_crc:
        print(f"  [WARNING] Dataset internal CRC mismatch for {dataset_path}!")
        print(f"    Expected: {expected_crc.hex()}, Computed: {computed_crc.hex()}")
    else:
        print(f"  Dataset internal CRC verified: {computed_crc.hex()}")
        
    # 4. Replace 0x5E000 to 0x5EFFF block
    start_addr = 0x5E000
    end_addr = 0x5EFFF
    
    if len(base_fw) < end_addr + 1:
        raise ValueError(f"Base firmware is too short: {len(base_fw)} bytes")
        
    base_fw[start_addr:end_addr + 1] = dataset
    
    # 5. Verify the entire 0x5E block checksum at 0x5EFFE in the final firmware
    # In 3001, the checksum at 0x5EFFE is calculated over 0x5E000 to 0x5EFFE (which is exactly what we just replaced/checked)
    fw_crc_region = base_fw[0x5E000:0x5EFFE]
    fw_computed_crc = crc16(fw_crc_region)
    fw_stored_crc = base_fw[0x5EFFE:0x5F000]
    
    assert fw_stored_crc == fw_computed_crc, f"Final firmware CRC at 0x5EFFE mismatch! Stored: {fw_stored_crc.hex()}, Computed: {fw_computed_crc.hex()}"
    print(f"  Final firmware block CRC verified at 0x5EFFE: {fw_stored_crc.hex()}")
    
    # 6. Save final firmware
    with open(output_path, "wb") as f:
        f.write(base_fw)
    print(f"  Successfully saved to {output_path} (size: {len(base_fw)} bytes)\n")

if __name__ == "__main__":
    base_bin = "firmware/TT_3001_239.bin"
    targets = [
        ("datasets/golf/v074N2431K0___Lenkung.bin", "firmware/TT_3001_243.bin"),
        ("datasets/golf/v074N2511K0___Lenkung.bin", "firmware/TT_3001_251.bin"),
        ("datasets/golf/v074N2831K0_Mobi_pq35_3501.bin", "firmware/TT_3001_283.bin"),
        ("datasets/golf/v074N3111K0___Lenkung_3501.bin", "firmware/TT_3001_311.bin"),
    ]
    
    for dataset_path, output_path in targets:
        build_bin(base_bin, dataset_path, output_path)
    print("All bins built successfully!")
