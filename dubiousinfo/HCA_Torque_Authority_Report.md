# HCA & PLA Torque Authority Report (TT vs Golf 3001)

This report details the internal torque limits, multiplier logic, and "cheat code" flags identified in the VW PQ35 Steering Rack firmware (Version 3001/3501).

---

## 1. The HCA Torque Pipeline
The rack processes your CAN command in three distinct stages. A fault can occur at any stage if a limit is exceeded.

### Stage 1: The CAN Input Filter
Before any math happens, the CAN parser checks your raw command counts against a hardcoded table in ROM (located at `0x5D160`).
*   **Status 5 (Standard HCA):** Limited to **300 counts**.
*   **Status 7 (High Authority HCA):** Limited to **500 counts** (TT Profile).
*   **Behavior:** Sending 501 counts in Status 7 will cause an immediate communication fault.

### Stage 2: The Dataset Multiplier (Gain)
The accepted counts are multiplied by a Speed-Dependent Gain found in the **Dataset** (`0x5Exxx` area, Map 3F).
*   **Formula:** `Internal_Counts = CAN_Command * (Dataset_Gain / 128)`
*   **TT (Dataset 239) Gains:**
    *   0 km/h: **0.41x** (53/128)
    *   40 km/h: **0.91x** (117/128)
    *   83 km/h: **1.06x** (136/128)
    *   250 km/h: **1.33x** (171/128)
*   **Result:** At low speeds, your "300" command only requests ~123 counts of motor effort. This is why it feels weak but doesn't fault.

### Stage 3: The Firmware Floating-Point Limiters
The result of the multiplier is checked against floating-point constants at `0x5D1A0`.

| Limit Type | Golf Value | TT Value | Function |
| :--- | :--- | :--- | :--- |
| **Soft Limit** | 380.0 | **381.5** | Triggers Slew-Rate Bypass |
| **Hard Limit** | 510.0 | **501.5** | Triggers Assist/Safety Bypass |
| **Guardrail** | 3200.0 | 3200.0 | Absolute Motor Maximum (25 Nm) |

---

## 2. The "Two 1.5 Flags" Mystery
The `.5` fractional offset in the TT limits (`381.5` and `501.5`) are not tuning values—they are **binary flags** encoded into the float mantissa (bitmask `0xC000`).

### Flag 1: Slew-Rate Bypass (`381.5`)
*   **Golf (380.0):** Low bits are `0x0000`. The rack smoothly ramps torque requests over time (Smoothing).
*   **TT (381.5):** Low bits are `0xC000`. The firmware **disables the ramp**, allowing the motor to jump to your target torque instantly.

### Flag 2: Assist Suppression Bypass (`501.5`)
*   **Behavior:** Normally, the rack reduces Power Steering Assist when HCA is active to keep the car stable.
*   **TT Benefit:** The `501.5` flag branch **disables this reduction**. You get your HCA torque **PLUS 100% of the rack's power assist**, giving the rack much more physical "muscle" to turn the wheels.

---

## 3. Extraction & Calculation
To verify your specific rack's breakpoints, you can use the provided `extract_hca_maps.py` script.

### How to calculate your "Safe Max":
`Max_CAN_Command = 501 / (Dataset_Gain / 128)`

**Example (TT Dataset 239):**
*   **At 20 mph (~32 km/h):** Gain is ~0.85x. `501 / 0.85 = 589`. (You are safe at **500 counts**).
*   **At 40 mph (~65 km/h):** Gain is ~1.00x. `501 / 1.00 = 501`. (You are right on the edge).
*   **At 50 mph (~80 km/h):** Gain is ~1.05x. `501 / 1.05 = 477`. (**A 500 count command will fault here!**)

---

## 4. Summary for Testing
1.  **Use Status 7:** This is the only way to bypass the 300-count input limit and reach 500.
2.  **Low Speed is Safe:** Below 40 mph, you can likely send **500 counts** without faulting because the gain is low enough to keep you under the internal 501.5 hard limit.
3.  **High Speed Danger:** Above 40 mph, you **must reduce your command**. If your internal request (Cmd * Gain) exceeds 501.5, the rack will throw a safety fault.
