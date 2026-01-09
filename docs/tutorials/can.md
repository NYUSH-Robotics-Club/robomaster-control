# CAN explanation

## What is CAN

CAN bus(Controller Area Network System) that connectes all our electrical components together. CAN was created for the simple purpose that any two electrical componenet will have the capability to be able to send and recieve commands withou a actual processing unit.

A CAN wire is composed of two wires, CAN low and CAN high

![can-wire.png](../assets/can-wire.png)

<p align="center"><sub><strong>Figure 1</strong>: can wire</sub></p>

the can wire that we use for robomasters is specifically 1.25-mm pitch 2-pin GHR wire (this is for when you guys inevitably have to buy more)

## CAN Frame

Since the can wire is just two wires and have to ensure that all electrical components can communicate between each other, the CAN Frame(which is what the CAN message is sent to each other is called) is very complicated 

Below is a picture of what it is composed of.

![can-wire.png](../assets/can-frame.png)

<p align="center"><sub><strong>Figure 2</strong>: can frame</sub></p>

* **SOF**: the start of the frame, tells the other units that a CAN message is coming
* **CAN ID**: Identifies the message, allows certain units to ignore it if it only reads certain CAN ID messages.
* **RTR**: whether or not a unit is sending or requesting data
* **CONTROL**: the length of the data
* **DATA**: Contains the actual data values
* **CRC**: ensures whether or not the data is corrupted after sendign it
* **ACK**: whether or not your unit has received the data correctly
* **EOF**: end of frame, signifying your CAN frame ended

In our case, you only really need to worry about how CAN ID and the DATA part of the data frame works. 

## CAN ID

In terms of robomasters, there are 8 different Motor IDs from 1-8 that translates to CAN IDs. There are two different forms of CAN IDs here, one for feedback(motor sending data back to board) and one for control. However, depending on the motor, the feedback identifier for the CAN ID will be different.

### GM6020

For GM6020, the CAN ID for motor feedback starts at 0x205 up to 0x20B (x in this case signifies that it is a hex number) with a control identifier of 0x1FF for IDs 1-4 or 0x205 to 0x208 and 0x2FF for 0x209 - 0x20B

lets first talk about sending can

![send-can-gm6020.png](assets/send-can-gm6020.png)

<p align="center"><sub><strong>Figure 3</strong>: sending can</sub></p>

based on the photo, you can see that each can frame can control up to 4 motors. Since our current message is from -25000 - 25000, it does not fit within 8 bits/1byte as the largest number 1 byte could be is -128 to 127 for signed numbers (click [**here**](https://cspages.ucalgary.ca/~mgavrilo/215/Signed_Numbers.html#:~:text=The%20rule%20for%20signed%20and,1%20for%20a%20Negative%20number.) for explanation of what a signed number) either way, it does not fit within just one data field.

In order to fix this, our data number is split into two in stored in two data fields where they will later be conbined and used to represent the actual voltage value.

Here is how we do it within the code

```C
HAL_StatusTypeDef CAN_Manager_SendGM6020Current(CAN_HandleTypeDef *hcan, uint8_t motor_id, int16_t current)
{
    if (hcan == NULL) return HAL_ERROR;
    //if wrong motor id is given put a error
    if (motor_id < 1 || motor_id > 7) return HAL_ERROR;

    //Clamps the current so if it goes over the max it can only give max input
    if (current >  25000) current =  25000;
    if (current < -25000) current = -25000;

    //sets the id of the motor tepending on whether or not it is 1-4 or 5-7
    uint16_t stdId = (motor_id <= 4) ? 0x1FF : 0x2FF;
    
    //determines which data slot it is supposed to be 
    uint8_t  slot  = (motor_id <= 4) ? (uint8_t)(motor_id - 1) : (uint8_t)(motor_id - 5);


    CAN_TxHeaderTypeDef tx = (CAN_TxHeaderTypeDef){0};
    //the DATA field
    uint8_t d[8] = {0};
    uint32_t mb;

    //sets StdId, IDE, RTR, DLC respectively
    tx.StdId = stdId;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;

    //splits the current value into two to put into the data list
    d[slot*2 + 0] = (uint8_t)((current >> 8) & 0xFF);
    d[slot*2 + 1] = (uint8_t)( current       & 0xFF);

    //sends the CAN message
    HAL_StatusTypeDef st = HAL_CAN_AddTxMessage(hcan, &tx, d, &mb);
    extern CAN_Manager_t can1_manager; extern CAN_Manager_t can2_manager;
    CAN_Manager_t *m = NULL;
    if (hcan == can1_manager.hcan) m = &can1_manager; else if (hcan == can2_manager.hcan) m = &can2_manager;
    if (m) {
        if (st == HAL_OK) m->tx_ok++; else m->tx_err++;
        m->last_tx_time = HAL_GetTick();
    }
    return st;
}
```

Next is receiving CAN

![receive-can-gm6020.png](assets/receive-can-gm6020.png)

<p align="center"><sub><strong>Figure 4</strong>: receive can</sub></p>

This is the same as sending can, except the data fields just means something different, the identifiers are from 0x204 plus the motor id. From the motor, you can get the angle, speed, torque current, and motor temp.

below is the code that we use to read the feedback

```c
// Gimbal pitch/yaw GM6020 feedback (allow on CAN1 and CAN2)
    if (rx.IDE==CAN_ID_STD && rx.DLC==8 && rx.StdId>=0x205 && rx.StdId<=0x20B) {
        uint8_t gid = (uint8_t)(rx.StdId - 0x204);
        if (gid >= 1 && gid <= 7) {
            uint16_t angle_raw = (uint16_t)((d[0]<<8) | d[1]);
            int16_t  speed_rpm = (int16_t)((d[2]<<8) | d[3]);
            pitch_on_feedback(gid, angle_raw, speed_rpm);
        }
    }
```


### M3508 motor with C620 motor speed controller

the m3508 motor has no speed controller integrated within the motor itself, there for, it has a c620 speed controller.

For sending CAN, for ids 1-4/0x201-0x204, the sending CAN ID is 0x200, and for ids 5-8/0x205-0x208, the sending CAN ID is 0x1FF.

![c620-send-can-1.png](assets/c620-send-can-1.png)

<p align="center"><sub><strong>Figure 5</strong>: send can 1</sub></p>


![c620-send-can-2.png](assets/c620-send-can-2.png)

<p align="center"><sub><strong>Figure 5</strong>: send can 2</sub></p>

```c
HAL_StatusTypeDef CAN_Manager_SendMotorCurrents4(CAN_HandleTypeDef *hcan, uint16_t std_id,
                                                int16_t i1, int16_t i2, int16_t i3, int16_t i4)
{
    if (hcan == NULL) return HAL_ERROR;
    CAN_TxHeaderTypeDef tx = (CAN_TxHeaderTypeDef){0};
    uint8_t d[8];
    uint32_t mb;

    tx.StdId = std_id;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;

    //sends values for currents for four different motors
    d[0] = (uint8_t)(i1 >> 8); d[1] = (uint8_t)i1;
    d[2] = (uint8_t)(i2 >> 8); d[3] = (uint8_t)i2;
    d[4] = (uint8_t)(i3 >> 8); d[5] = (uint8_t)i3;
    d[6] = (uint8_t)(i4 >> 8); d[7] = (uint8_t)i4;

    HAL_StatusTypeDef st = HAL_CAN_AddTxMessage(hcan, &tx, d, &mb);
    // Update debug counters: find which manager this handle belongs to
    extern CAN_Manager_t can1_manager; extern CAN_Manager_t can2_manager;
    CAN_Manager_t *m = NULL;
    if (hcan == can1_manager.hcan) m = &can1_manager; else if (hcan == can2_manager.hcan) m = &can2_manager;
    if (m) {
        if (st == HAL_OK) m->tx_ok++; else m->tx_err++;
        m->last_tx_time = HAL_GetTick();
    }
    return st;
}
```

For receiving can from the motors, the identifier is based on the id plus 0x200, so if the motor id is 1, then the identifier is 0x201

![c620-send-receive.png](assets/c620-receive-can.png)

<p align="center"><sub><strong>Figure 5</strong>: receive can 2</sub></p>

below is the code

```c
if (rx.IDE==CAN_ID_STD && rx.DLC==8 && rx.StdId>=0x201 && rx.StdId<=0x20B) {
        uint8_t  mid   = rx.StdId - 0x201;
        if (mid < 8) {
            uint16_t angle = (d[0]<<8) | d[1];
            int16_t  speed = (int16_t)((d[2]<<8) | d[3]);
            int16_t  current = (int16_t)((d[4]<<8) | d[5]);
            uint8_t  temp = d[6];
            
            // Update chassis motor feedback (0-3)
            if (mid < 4 && manager->chassis_controller != NULL) {
                ChassisController_UpdateMotorFeedback(manager->chassis_controller, mid, angle, speed, current, temp, current_tick);
            }
            // Update shooter system motor feedback (4-7)
            else if (mid >= 4 && manager->shooter_controller != NULL) {
                ShooterController_UpdateMotorFeedback(manager->shooter_controller, mid, angle, speed, current, temp, current_tick);
            }
        }
    }
```

## Setting CAN IDs

The IDs of the motors are set on the motor themselves, the way to do it can be found within the official documentations. 

For GM6020 click  [**here**](docs/official-docs/RM_GM6020_Docs.pdf)

For C620/M3508 click  [**here**](docs/official-docs/Robomaster_C620_Docs.pdf)

## Hardware / Official-Doc Notes (GM6020 + C620)

### Common CAN bus facts for RoboMaster motors

* **Bitrate:** Both **GM6020** and **C620** CAN bus use **1 Mbps**.  
* **CAN cable wire colors (important):**

  * **CAN_H = Red**, **CAN_L = Black** for GM6020. 
  * **CAN_H = Red**, **CAN_L = Black** for C620 too (but the “A/B pin label” is reversed vs GM6020; follow the color). 
* **Termination resistance (120Ω):**

  * GM6020 enables/disables CAN terminal resistance via DIP **4th bit**. 
  * C620 enables/disables termination via a dedicated **Termination Resistance Switch (120Ω)**. 
  * Practical wiring rule (CAN best practice): only the **two physical ends** of a CAN bus should have termination ON; nodes in the middle should be OFF.

---

## GM6020 (Gimbal Motor) — Official ID / Termination / Protocol Details

### 1) Motor ID setting (DIP switch)

GM6020 motor ID is decided by DIP **Bit0–Bit2** (binary). Valid IDs are **1–7**; “000” is invalid. 

| Bit[2:0] | Motor ID | Feedback CAN ID |
| -------- | -------: | --------------- |
| 001      |        1 | 0x205           |
| 010      |        2 | 0x206           |
| 011      |        3 | 0x207           |
| 100      |        4 | 0x208           |
| 101      |        5 | 0x209           |
| 110      |        6 | 0x20A           |
| 111      |        7 | 0x20B           |

(From doc mapping) 

### 2) CAN terminal resistance (DIP switch)

DIP **4th bit** controls whether CAN terminal resistance is enabled. Toggle it **ON** to enable termination. 

### 3) Control identifiers & ranges (what you send)

GM6020 supports both CAN and PWM, and it can auto-switch based on the detected control signal. 

**Voltage control mode (most common in RM examples):**

* Control identifiers: **0x1FF** (motor IDs 1–4), **0x2FF** (motor IDs 5–7) 
* One frame controls **up to 4 motors**, each motor uses **2 bytes** (high/low). 
* Controllable “voltage value” range: **-25000 ~ 25000**. 

**Torque current control (only after enabling Current Ring):**
After firmware **v1.0.11.2+**, torque current can be controlled by enabling **Current Ring On/Off Switch** in RoboMaster Assistant (**v2.7+**). 
Docs also describe current command ranges **-16384 ~ 16384**, and corresponding max torque current **-3A ~ 3A**, with identifiers **0x1FE / 0x2FE**. 
Additionally, in “Current Control Mode” the receiving identifier is described as **0x204 + driver ID** (standard frame, DLC=8). 

> Practical note: your README currently describes the -25000~25000 “current”, but per official doc this range corresponds to **voltage control value** (not torque current). 

### 4) Feedback (what you receive)

Motor feedback contains: rotor mechanical angle, rotational speed, current, temperature. 
And the feedback identifier depends on the motor ID mapping (0x205–0x20B). 

### 5) LED quick diagnosis (useful for debugging wiring / IDs)

* Green blinks **N times** per second indicates current motor ID. 
* Orange blinks **twice** per second indicates **duplicate ID on the CAN bus**. 
* Driver cuts off output stream when in abnormal status (important when debugging “motor not moving”). 

---

## C620 + M3508 — Official ID / Termination / Protocol Details

### 1) CAN port bitrate & termination

* CAN bitrate: **1 Mbps**. 
* Termination resistance: hardware switch to connect/disconnect **120Ω**. 

### 2) Signal mode safety: DO NOT mix CAN + PWM

* The input signal mode (PWM vs CAN) **cannot be changed while the product is in use**; power off to change mode and restart. 
* **DO NOT plug CAN cable and PWM cable simultaneously**, or the motor may lose control; power off when switching modes. 

### 3) Speed controller ID setting (SET button)

C620 supports ID range **1–8**. 
Two official ways:

**A) Separate ID Setting (one-by-one)**

1. Press SET once to enter separate ID assignment (LED off). 
2. Press SET again **N times (≤8)** to set ID = N (LED blinks orange each successful press). 
3. Idle 3 seconds → auto-save; power cycle to apply. 
4. Multiple controllers on the same CAN bus **cannot share the same ID**. 

**B) Quick ID Setting (assign 1–8 by rotating motors)**

1. Press SET once on any controller, then press-and-hold until all status LEDs are solid green. 
2. Manually rotate each M3508 rotor (≥180°) in your chosen order; the corresponding controller gets IDs **1..8** in that order. 
3. Power cycle after assigning. 
4. If a rotor isn’t rotated, it keeps its original ID after power-on. 
5. Doc explicitly warns to connect/disconnect termination correctly or CAN may fail. 

### 4) Control identifiers & ranges (what you send)

C620 receiving message format: identifiers **0x200** and **0x1FF** control current outputs for groups of four controllers by ID. 

* **0x200** controls ID **1–4**
* **0x1FF** controls ID **5–8** 

Command range: **-16384 ~ 16384**, corresponding output torque current **-20A ~ 20A**. 

### 5) Feedback identifiers & payload (what you receive)

C620 sends feedback to CAN bus:

* Feedback identifier: **0x200 + speed controller ID** (e.g., ID=1 → 0x201). 
* Data fields include: rotor mechanical angle, rotational speed, actual torque current, motor temperature. 
* Default sending frequency: **1 kHz** (can be changed in RoboMaster Assistant). 

### 6) LED quick diagnosis (debugging CAN issues)

* Normal: green blinks **1–8 times** per second = current controller ID. 
* Warning: orange blinks **twice** per second = **duplicate ID on CAN bus**, and output may be cut off.  
