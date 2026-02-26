# Physical Robot Parameter Specification Form

Please provide values for ALL of the following parameters. This will ensure your simulation matches your physical robot as precisely as possible.

---

## 🤖 CHASSIS & DIMENSIONS

### Overall Robot
- [ ] **Robot mass** (kg) 0.245
- [ ] **Total chassis length** (mm) 170
- [ ] **Total chassis width** (mm) 140
- [ ] **Total chassis height** (mm) 10
- [ ] **Center of mass location** (mm from front wheel axle) 55

### Wheels & Drivetrain
- [ ] **Wheel radius** (mm) 17
- [ ] **Wheel base** (mm, distance between wheels) - 165
- [ ] **Wheel mass** (grams each) 14
- [ ] **Wheel material** (rubber, foam, silicone, etc.) sponge-rubber
- [ ] **Tire hardness/grip coefficient** (estimate 0-1) 1.14, questionable calculations
- [ ] **Wheel tread pattern** (smooth, grooved, knobby, etc.) smooth

---

## ⚡ MOTOR SPECIFICATIONS

### Motor Details
- [ ] **Motor type** brushed dc
- [ ] **Motor manufacturer and model** Pololu 30:1 Micro Metal Gearmotor HP 6V
- [ ] **Supply voltage** 7.4v
- [ ] **No-load speed** 1000rpm
- [ ] **Stall torque** 0.57 kg·cm
- [ ] **Rated power** 1.5W

### Motor Current
- [ ] **No-load current** 0.1A
- [ ] **Stall current** 1.6A
- [ ] **Typical operating current** 0.3A

### Motor Dynamics
- [ ] **Motor time constant (tau)** (ms) - Current: `MOTOR_TAU = 0.05` (50ms)
  - If unknown: How many milliseconds to reach 63% speed from rest?
- [ ] **Electrical time constant** (ms)
- [ ] **Dead zone voltage** (V, minimum voltage to start moving)
- [ ] **Back EMF coefficient** (V·s/rad, if known)

### Gearbox (if present)
- [ ] **Gear ratio** (motor turns per wheel turn)
- [ ] **Gearbox efficiency** (%)
- [ ] **Gearbox backlash** (degrees)
- [ ] **Gearbox material** (metal, plastic)

---

## 🔌 POWER & ELECTRONICS

### Battery
- [ ] **Battery type** lipo
- [ ] **Battery voltage** 7.6V
- [ ] **Battery capacity** 660 mAh
- [ ] **Battery weight** 35
- [ ] **Battery C-rating** 90c
- [ ] **Voltage sag under load** (estimated V drop at full current)

### Motor Driver
- [ ] **Driver type** arduino motor driver
- [ ] **Driver model/part number**
- [ ] **Voltage drop** across driver at nominal current (V)
- [ ] **Maximum continuous current** (A)
- [ ] **PWM frequency** (Hz, if applicable)
- [ ] **Response time** (ms from command to motor action)

### Microcontroller
- [ ] **Processor type** esp32
- [ ] **Processor model** esp32-32d n4
- [ ] **CPU clock speed** (MHz)
- [ ] **Number of PWM pins available**

---

## 👁️ SENSOR SPECIFICATIONS (Line Follower Array)

### Array Configuration
- [ ] **Sensor array model** QTRX-HD-25RC
- [ ] **Total number of sensors** - Current: `QTR_CHANNELS = 8`
- [ ] **Sensor spacing** (mm between adjacent sensors) - Current: `QTR_SPACING_M = 0.004` (4mm)
- [ ] **Array total length** 101mm
- [ ] **Mounting height above ground** 5.5mm
- [ ] **Sensor mounting angle** nose down, parallel to ground, maybe 10° tilt?

### Individual Sensor Technology
- [ ] **Sensor type** IR reflectance
- [ ] **Operating voltage per sensor** 2.9-5.5v
- [ ] **LED/light source wavelength** 940nm
- [ ] **LED current** 3.5mA
- [ ] **Receiver type** (photodiode, phototransistor, etc.)

### Sensor Timing & Response
- [ ] **Charge/decay time** (µs)
- [ ] **Sampling time per sensor** (µs)
- [ ] **Total read time for array** (µs)
- [ ] **Response time** (ms to 90% output)

### Sensor Calibration
- [ ] **Dark reading on black line** 0-1023 i think
- [ ] **Light reading on white surface** (ADC units)
- [ ] **Noise level** (±ADC units at rest)
- [ ] **Sensor offset position from robot center** 97mm
- [ ] **Individual sensor lateral positions** (mm from center, if custom)

### Line Detection
- [ ] **Current line detection threshold** - Current: `LINE_THRESH = 0.08`
- [ ] **How is threshold calibrated?** (auto-range, fixed value, etc.)

---

## 🛞 TRACK & FRICTION PROPERTIES

### Track Characteristics
- [ ] **Line color** black
- [ ] **Background color** white
- [ ] **Line width** 15mm
- [ ] **Track surface material** vinyl flooring
- [ ] **Surface finish** matte
- [ ] **Surface condition** a bit dusty

### Friction & Traction
- [ ] **Estimated friction coefficient** μ 1.14
- [ ] **Does wheel slip on your track?** yes, due to bad center of mass, light weight and high torque
- [ ] **Does traction vary with speed?** i think so, but not sure how to measure it
- [ ] **Does traction vary with direction?** no

---

## 🎮 CONTROL LOOP & TIMING

### Sensor-to-Motor Path
- [ ] **Sensor read time** (ms)
- [ ] **Processing/control time** (ms)
- [ ] **Motor command time** (ms)
- [ ] **Motor response lag** (ms from command to measurable speed change)
- [ ] **Total loop latency** (ms from sensor read to wheel speed change)

### Actual Update Rate
- [ ] **Measured/actual control loop frequency** (Hz)
- [ ] **Intended control loop frequency** (Hz)
- [ ] **Current DT in simulation** = `0.005` (200 Hz)
- [ ] **Does your robot match 200 Hz?** (yes/no, what's the actual rate?)

### Motor Speed
- [ ] **Maximum wheel speed on your track** (m/s or RPM)
- [ ] **Minimum usable wheel speed** (m/s)
- [ ] **Speed measurement method** (encoder, dead reckoning, visual estimation)
- [ ] **Speed control accuracy** (±% of commanded speed)
- [ ] **Speed response time** (ms to reach target speed)

---

## 📊 CURRENT TUNING (for reference)

Your current working parameters:
```
PID_KP              = 122.0
PID_KI              = 4.07
PID_KD              = 19.3
PID_LIMIT           = 18.0
SC_STRAIGHT_SPEED  = 1.097
SC_TURN_SPEED      = 0.960
SC_ERROR_THRESHOLD = 0.0020
SC_SMOOTHING       = 0.300
MAX_LINE_LOSS_TIME  = 0.3
```

- [ ] **Are these parameters tuned for your physical robot?** (yes/no)
- [ ] **How long did tuning take?** (hours/days)
- [ ] **What track was it tuned on?** (bane_fase2, suzuka, custom)

---

## 🔄 ROBOT BEHAVIOR OBSERVATIONS

### Speed & Performance
- [ ] **Best lap time achieved on your track** (seconds)
- [ ] **Typical speed on straights** (m/s)
- [ ] **Typical speed in corners** (m/s)
- [ ] **Tightest turn radius possible** (mm)
- [ ] **Steering response lag** (ms from error to heading change)
- [ ] **Oscillation frequency when cornering** (Hz, if noticeable)

### Line Following Quality
- [ ] **Typical lateral error from center** (mm)
- [ ] **Maximum error before losing line** (mm)
- [ ] **Recovery time from minor drift** (ms)
- [ ] **Main source of errors** (sensor noise, motor lag, inertia, etc.)

### Robustness
- [ ] **Line loss duration before restart** (seconds) - Current: `MAX_LINE_LOSS_TIME = 0.3`
- [ ] **How often does it lose the line?** (never, rarely, occasionally, frequently)
- [ ] **What causes line loss?** (sharp turns, noise, shadows, etc.)

### Physical Characteristics
- [ ] **Does robot vibrate?** (yes/no, at what frequencies?)
- [ ] **Power consumption** (mA typical, mA peak)
- [ ] **Operating time on battery** (minutes)
- [ ] **Temperature stability** (does behavior change with temperature?)
- [ ] **Weight distribution** (balanced, front-heavy, rear-heavy)

---

## 📸 ENVIRONMENTAL

### Lighting
- [ ] **Ambient light level** (lux, or "bright", "normal", "dim")
- [ ] **Light source** (natural sunlight, LED, fluorescent, etc.)
- [ ] **Lighting uniformity** (consistent across track, or variable)
- [ ] **Does lighting affect sensor readings?** (yes/no)

### Track Variations
- [ ] **Is track flat or have slopes?** (degrees of tilt, if any)
- [ ] **Surface consistency** (uniform, variable texture)
- [ ] **Turn types** (sharp 90°, gentle curves, S-curves)
- [ ] **Typical straightaway length** (mm)
- [ ] **Typical turn radius** (mm)

---

## 🔍 MEASUREMENT DATA (If Available)

Please provide any of these if you have them:

- [ ] **Logged sensor readings** from actual robot runs (CSV/text format)
- [ ] **Motor speed measurements** (RPM vs time graphs)
- [ ] **Actual lap times** from multiple runs
- [ ] **Error traces** (lateral error vs time from real runs)
- [ ] **Speed profiles** (wheel speed vs distance)
- [ ] **Motor current measurements** during operation
- [ ] **Video of robot in action** (can extract dynamics from this)
- [ ] **Acceleration/deceleration measurements**

---

## 📋 HOW TO FILL THIS OUT

**Format:** Use checkboxes ☑️ for items you're providing, and ⬜ for unknown items.

**For each item:**
1. Check the box if you know the value
2. Provide the measured/actual value
3. If unknown, leave unchecked or write "unknown" or "estimate: X"
4. Include units always

**Priority levels:**
- 🔴 **CRITICAL** (big impact on simulation accuracy):
  - Motor time constant (MOTOR_TAU)
  - Wheel base
  - Sensor spacing & offset
  - Friction coefficient
  - Actual control loop frequency
  - Current PID gains (if tuned)

- 🟡 **IMPORTANT** (medium impact):
  - Motor speed limits
  - Sensor sensitivity (dark/light readings)
  - Track properties
  - Motor response lag
  - Battery voltage sag

- 🟢 **NICE TO HAVE** (refinement):
  - Exact motor model specs
  - Component masses
  - Temperature effects
  - Detailed measurement data

---

## NEXT STEPS

Once you provide these parameters, I will:

1. ✅ Update `config.py` with your exact specifications
2. ✅ Adjust friction model for your wheel/track combination
3. ✅ Fine-tune motor dynamics model
4. ✅ Calibrate sensor noise characteristics
5. ✅ Match control loop timing to your actual hardware
6. ✅ Re-optimize PID gains if needed
7. ✅ Validate against your measured data

---

**Please fill out this form with as many parameters as you can provide. Even estimates are better than generic defaults!**

**You can respond with:**
- Filled-out version of this form
- A simple list like "Motor time constant: 50ms, Wheel base: 120mm, ..."
- A table format
- Whatever is easiest for you!

