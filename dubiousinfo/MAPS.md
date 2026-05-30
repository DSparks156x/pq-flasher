# MQB/PQ Steering Dataset & HCA 7 Torque Pipeline

This document defines the definitive binary layout and mathematical logic for steering rack datasets. Use this for building high-precision map-based torque controllers.

---

## 1. Binary Structure & Addressing
*   **Format:** 4096-byte blob.
*   **Header:** Pointer Table from `0x000` to `0x1CF` (32-bit Little Endian pointers).
*   **Base Address:** `0x5E000`.
*   **File Offset:** `PointerValue - 0x5E000`.

---

## 2. Mathematical Scaling
| Parameter | Raw Scaling | Physical Unit |
| :--- | :--- | :--- |
| **Input Torque (X)** | 128 counts = 1.0 Nm | `Nm (Pinion Input)` |
| **Motor Assist (Y)** | 100 counts = 1.0 Nm | `Nm (Motor Assistance)` |
| **Vehicle Speed** | 1 count = 1 km/h | `km/h` |
| **HCA/Global Gains** | 128 counts = 1.0x | `Multiplier` |

---

## 3. Assist Map Architecture (5-Point)

The rack uses 5 primary groups of 8-point assist curves (Ptr `0x03` to `0x11`) and one fallback.

### 3.1 Primary Assist Interpolation
The rack interpolates between these groups using the **5-point Speed Axis** at Offset `0x7A8`.

| Speed BP | Ptr Index Group | Description |
| :--- | :---: | :--- |
| **0 km/h** | `0x03` | Parking |
| **15 km/h** | `0x06` | City |
| **50 km/h** | `0x09` | Urban |
| **100 km/h** | `0x0C` | Highway |
| **250 km/h** | `0x0F` | Top Speed |

### 3.2 Error Fallback
| Ptr Index Group | Description |
| :---: | :--- |
| **0x12** | **Limp Home.** Used when speed signal is lost. (Often identical to 15km/h or 250km/h curve). |

---

## 4. HCA / LKA Logic (6-Point)

LKA authority is handled by a separate pipeline with higher speed resolution.

### 4.1 HCA Speed Gain (6-Point Map)
*   **Pointer Index:** `0x3F` (Map at Offset `0x5D4`).
*   **X-Axis (Speed):** `[0, 11, 40, 83, 150, 250]`.
*   **Y-Axis (Gain):** Pre-scaling multiplier for the HCA CAN command.

### 4.2 ADAS Global Scalar
*   **Location:** Word at `0x63A`.
*   **Effect:** A global multiplier (usually ~0.9x to 1.1x) applied to all ADAS torque requests.

---

## 5. Final Torque Pipeline (HCA 7)

$$T_{virtual\_input} = HCA_{cmd} \cdot \left( \frac{Scalar}{128} \right) \cdot \left( \frac{Gain_{HCA}(Speed)}{128} \right)$$

$$T_{assist} = Interpolate\_5\_Speed(AssistMaps, Speed, T_{virtual\_input})$$

$$T_{final\_output} = T_{virtual\_input} + T_{assist}$$
