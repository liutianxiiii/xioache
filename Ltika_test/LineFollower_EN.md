# STM32F303K8 Four-Sensor Line Following Robot

## Hardware Pin Mapping

### Light Sensors (ADC1 + DMA, 12-bit, 0–4095)

| Buffer Index | Pin | ADC Channel | Position |
|-------------|-----|-------------|----------|
| `adc_buf[0]` | PA0 | IN1 | Leftmost |
| `adc_buf[1]` | PA1 | IN2 | Mid-left |
| `adc_buf[2]` | PA3 | IN4 | Mid-right |
| `adc_buf[3]` | PB0 | IN11 | Rightmost |

DMA runs in circular mode and continuously refreshes the buffer. No manual ADC trigger is needed in the main loop.

### Motor Driver (TB6612 or equivalent)

| Motor | Direction Pin 1 | Direction Pin 2 | PWM Pin | Timer Channel |
|-------|----------------|----------------|---------|---------------|
| Left wheel — Motor A | AIN1 = PF0 | AIN2 = PF1 | PA8 | TIM1_CH1 |
| Right wheel — Motor B | BIN1 = PB1 | BIN2 = PA10 | PA9 | TIM1_CH2 |

**PWM parameters**: TIM1, prescaler 32, period 1000 → frequency = 8 MHz / 32 / 1000 = **250 Hz**. CCR range 0–1000 maps to 0–100% duty cycle.

---

## Initialization Order (`USER CODE BEGIN 2` in `main`)

```
HAL_ADCEx_Calibration_Start()   ← must run before DMA starts
HAL_ADC_Start_DMA()             ← start continuous acquisition
HAL_TIM_PWM_Start(CH1)          ← enable left wheel PWM output
HAL_TIM_PWM_Start(CH2)          ← enable right wheel PWM output
```

> **Note**: Three bugs existed in the CubeMX-generated code and have been fixed in `main.c`:
> 1. `adc_buffer` and `adc_value` were two separate buffers; DMA was started twice pointing to different addresses
> 2. ADC calibration was called after `HAL_ADC_Start_DMA` (wrong order)
> 3. `HAL_ADC_Start_DMA` was called twice

---

## Line-Following State Machine

### Sensor Thresholding

```c
uint8_t s = (adc_buf[i] > BLACK_THRESHOLD) ? 1 : 0;
// 1 = black line detected, 0 = white surface
```

The four sensor readings are encoded as a 4-bit value `[sL  sML  sMR  sR]`:

```
sL   sML   sMR   sR
PA0  PA1   PA3   PB0
Left               Right
```

### Decision Table

| State (binary) | Meaning | Left PWM | Right PWM | Action |
|---------------|---------|---------|----------|--------|
| `0110` `0111` `1110` `1111` | Centered | 600 | 600 | Go straight |
| `0010` | Slight right drift (sMR only) | 680 | 520 | Gentle left correction |
| `0011` | Noticeable right drift (sMR+sR) | 750 | 200 | Moderate left turn |
| `0001` | Hard right drift (sR only) | 750 | −450 | Sharp left spin |
| `0100` | Slight left drift (sML only) | 520 | 680 | Gentle right correction |
| `1100` | Noticeable left drift (sL+sML) | 200 | 750 | Moderate right turn |
| `1000` | Hard left drift (sL only) | −450 | 750 | Sharp right spin |
| `1001` | T-junction / end of line | 600 | 600 | Continue straight |
| `0000` | Line lost | 300 | 300 | Slow forward — search for line |

---

## Tunable Parameters (macros at the top of `main.c`)

| Macro | Default | Description |
|-------|---------|-------------|
| `BLACK_THRESHOLD` | 2000 | ADC threshold (0–4095). Above this value = black line |
| `SPEED_BASE` | 600 | Straight-line speed |
| `SPEED_TRIM` | 80 | Gentle correction magnitude; actual speed = `base ± TRIM`. Reduce if oscillation occurs on straight lines |
| `SPEED_FAST` | 750 | Outer wheel speed during a moderate turn |
| `SPEED_SLOW` | 200 | Inner wheel speed during a moderate turn |
| `SPEED_SPIN` | 450 | Reverse speed of inner wheel during a sharp spin turn |

---

## Troubleshooting

### Robot turns in the wrong direction
Swap the two arguments in the `Motor_Drive` call, or invert the `xIN1`/`xIN2` logic for the affected motor inside `Motor_Drive`.

### Sensor polarity is inverted (white surface returns a high ADC value)
Change the comparison operator in `main.c` from `>` to `<`:

```c
// Before
uint8_t sL = (adc_buf[0] > BLACK_THRESHOLD) ? 1 : 0;
// After
uint8_t sL = (adc_buf[0] < BLACK_THRESHOLD) ? 1 : 0;
```

### Robot overshoots and leaves the track at speed
Lower `SPEED_BASE` and reduce `SPEED_FAST` proportionally.

### Robot cannot recover after losing the line
Increase the slow-forward speed in the `case 0x00` branch, or add logic to remember the last known turn direction (store the previous `st` value in a variable and repeat that motor command when `st == 0`).

---

## Further Improvements

- **PID controller**: Replace the state machine with a weighted error term `e = −3·sL − 1·sML + 1·sMR + 3·sR`. Use P and D terms to improve tracking accuracy and reduce oscillation.
- **Dynamic thresholding**: At startup, scan sensor min/max values and compute a midpoint threshold automatically to adapt to different lighting conditions.
- **TIM2** (already initialized, 1 Hz period): Available for periodic debug output or watchdog feeding.
