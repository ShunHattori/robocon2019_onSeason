# Control Panel Comunnicating Sheets

## Send Data

### Mode Enum
```
typedef enum {
  RED_FRONT_PRE_bathTowelLeft,
  RED_MIDDLE_PRE_bathTowelRight,
  RED_MIDDLE_PRE_bathTowelLeft,
  RED_MIDDLE_PRE_bathTowelboth,
  RED_MIDDLE_FINAL_bathTowelRight,
  RED_MIDDLE_FINAL_bathTowelLeft,
  RED_MIDDLE_FINAL_bathTowelboth,
  RED_BACK_Sheets,
  BLUE_FRONT_PRE_bathTowelLeft,
  BLUE_MIDDLE_PRE_bathTowelRight,
  BLUE_MIDDLE_PRE_bathTowelLeft,
  BLUE_MIDDLE_PRE_bathTowelboth,
  BLUE_MIDDLE_FINAL_bathTowelRight,
  BLUE_MIDDLE_FINAL_bathTowelLeft,
  BLUE_MIDDLE_FINAL_bathTowelboth,
  BLUE_BACK_Sheets
} GameMode;
```

```
uint8_t sendData;
```

---

## Receive Data

## PREFIX PRESETS

- LED TAPE  'L'
- DEBUGGING 'D'

### MDD Packets
- Drive PWM x6
- Mechanism Right x6
- Mechanism Left x6
    
---
 total 18 bytes

### Robot Parameter
- Location [Target(4byte) x3, Current(4byte) x3] 
- Drive Vector [wheel(4byte) x3] 

---
 total 36 bytes

```
 float Target.x 
 1byte: (x << 24) & 0xFF000000
 2byte: (x << 16) & 0xFF0000
 3byte: (x <<  8) & 0xFF00
 4byte: (x      ) & 0xFF
```