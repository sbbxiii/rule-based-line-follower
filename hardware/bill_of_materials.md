# 📦 Bill of Materials (BOM)

Comprehensive component listing and hardware specifications for the Rule-Based Autonomous Line Follower Mobile Robot.

---

## Component List

| Component | Quantity | Specifications / Rating | Function |
|:---|:---:|:---|:---|
| **Microcontroller Board** | 1 | **Arduino Uno R3** (ATmega328P @ 16 MHz, 32 KB Flash, 2 KB SRAM) | Central computing unit, analog sampling, FSM decision dispatch |
| **Motor Driver Module** | 1 | **L298N Dual H-Bridge Module** (Peak 2A per channel, integrated 5V regulator) | Bi-directional DC motor control with PWM speed regulation |
| **DC Drive Motors** | 2 | **6V Micro Metal Gearmotors** (10:1 gear reduction ratio, ~1000 RPM rated) | Differential drive propulsion and steering |
| **Optical Sensor Array** | 1 | **5-Channel Infrared Reflectance Array** (~15 mm pitch, analog outputs) | Track boundary and line detection |
| **Power Supply** | 1 | **7.4V 2S Li-ion Battery Pack** (~1500–2200 mAh) | High-current discharge power source for motors and electronics |
| **Chassis Platform** | 1 | **Perforated Prototyping Board / FR4 Chassis Plate** | Structural base, component mounting, vibration dampening |
| **Drive Wheels** | 2 | **High-Traction Rubber Wheels** (~32–34 mm diameter) | Rear axle ground traction |
| **Front Support / Glide** | 1 | **Low-friction skid / Omni caster ball** | Low-resistance front support and balance |
| **Interconnects** | 1 set | Multicolored ribbon cables, jumper wires, terminal blocks | Signal routing between MCU, driver, and sensor array |
| **Vibration Isolation** | 1 pc | High-density foam damping pad | Mounted under IR sensor array to suppress acoustic & surface vibration |

---

## Mechanical Specifications

- **Drive Architecture**: 2-Wheel Rear Differential Drive + Front Glide Skid
- **Overall Dimensions**: Approximately $210\text{ mm} \times 110\text{ mm} \times 60\text{ mm}$
- **Center of Gravity**: Rearward biased due to motor, driver, and battery placement
- **Sensor Standoff Height**: $8\text{--}12\text{ mm}$ above the floor surface
- **Track Line Compatibility**: Standard $3.82\text{ cm}$ dual electrical tape black line on high-reflectance background
