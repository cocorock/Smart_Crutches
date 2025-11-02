# IMPORTANT CLARIFICATION - Button Behavior & Stop Conditions

## ❗ Corrected Understanding

### What B3 Button Actually Does

**B3 Button Behavior:**
- **During normal operation:** Prints current analog value to console
- **Does NOT stop logging**
- **Does NOT send "AN" command to stop the script**

**Format when B3 is pressed:**
```
[timestamp],AN,[analog_value]
```

**Example:**
```
520000,AN,2048
```

**What the Python script does:**
- Displays: `ℹ️  Analog value from CrutchHMI-BT-7036: 2048`
- Continues logging normally

---

## 🛑 How to Stop Logging

### Method 1: Analog Input Reaches Emergency Stop Threshold (ES)

**When analog input > 3276:**
- Crutch firmware sends: `[timestamp],ES,AN:[value]`
- Example: `520000,ES,AN:3500`
- Python script detects ES command
- **Logging stops automatically**

**This is the intended method for stopping during operation!**

### Method 2: Close Plot Window

- User closes the matplotlib plot window
- Python script detects window close
- Logging stops gracefully

### Method 3: Ctrl+C / Keyboard Interrupt

- User presses Ctrl+C in console
- Script catches interrupt
- Logging stops and saves data

---

## 📊 Complete Command Reference

### Commands During Logging:

| Source | Command | Format | Python Action | Logging |
|--------|---------|--------|---------------|---------|
| **B1 press** | MC | `timestamp,MC,AN:value` | Confirms motor calibration | N/A (before logging) |
| **B3 press** | AN | `timestamp,AN,value` | Prints analog value | ✅ Continues |
| **Analog > 3276** | ES | `timestamp,ES,AN:value` | **Stops logging** | 🛑 Stops |
| **Analog < 819** | W data | `timestamp,W,force,qw,qx,qy,qz` | Logs as "Walking" | ✅ Continues |
| **Analog 819-3276** | S data | `timestamp,S,force,qw,qx,qy,qz` | Logs as "Standing" | ✅ Continues |

---

## 🎯 Typical Session Flow

### Complete Workflow:

```
1. Power on crutches
   ↓
2. Run Python script
   ↓
3. Script finds and connects to both devices
   ↓
4. Press B1 on Crutch 1 → MC command sent
   ↓
5. Press B1 on Crutch 2 → MC command sent
   ↓
6. Wait 5 seconds (automatic)
   ↓
7. 🚀 LOGGING STARTS
   ↓
8. Normal data streaming:
   - timestamp,S,force,qw,qx,qy,qz
   - timestamp,W,force,qw,qx,qy,qz
   ↓
9. (Optional) Press B3 to check analog value
   - Console shows: "ℹ️  Analog value: 2048"
   - Logging continues
   ↓
10. Move analog input to ES threshold (>3276)
    OR close plot window
    ↓
11. ES command sent: timestamp,ES,AN:3500
    ↓
12. 🛑 LOGGING STOPS
    ↓
13. CSV file saved
    ↓
14. Session complete ✅
```

---

## 🔧 Analog Input Thresholds

| Range | State | Command | Behavior |
|-------|-------|---------|----------|
| < 819 | Walking | W | Normal logging |
| 819 - 3276 | Standing | S | Normal logging |
| > 3276 | Emergency Stop | ES | **STOPS LOGGING** |

**Important:** The ES threshold is used as the stop condition!

---

## 📝 What Changed from Original Understanding

### ❌ WRONG (Original):
- "Press B3 to stop logging"
- "B3 sends AN command that stops the script"

### ✅ CORRECT (Updated):
- "Press B3 to print analog value (informational only)"
- "Analog reaching ES threshold stops the script"
- "ES command (timestamp,ES,AN:value) triggers stop"

---

## 🎓 Why This Makes Sense

**B3 as Information Button:**
- During operation, you might want to check the current analog value
- B3 provides this info without interrupting logging
- Useful for calibration and debugging

**ES as Stop Condition:**
- Emergency Stop is a definitive event
- Indicates the user has moved the joystick to maximum
- Natural endpoint for a measurement session
- Safety feature (emergency situations)

**Alternative Stop Methods:**
- Closing plot window (graceful exit)
- Ctrl+C (user interruption)
- Not triggered by button presses during normal operation

---

## 🔍 Message Examples

### Normal Operation:
```
520000,S,894.88,0.575,-0.418,0.570,0.408
520050,S,897.23,0.574,-0.419,0.571,0.409
520100,W,125.45,0.573,-0.420,0.572,0.410
```

### B3 Pressed (Info Only):
```
520150,AN,2048
```
Console: `ℹ️  Analog value from CrutchHMI-BT-7036: 2048`

### Emergency Stop (Stops Logging):
```
520200,ES,AN:3500
```
Console: 
```
🛑 ES (Emergency Stop) received from CrutchHMI-BT-7036
   Analog value: 3500 (threshold: 3276)
   Stopping logging...
```

---

## 💡 Pro Tips

### Testing the Stop Condition:

1. **Manual ES trigger:**
   - Move the analog joystick to maximum position
   - Value should exceed 3276
   - Logging stops automatically

2. **Check threshold:**
   - During logging, press B3
   - Console shows current analog value
   - Compare to threshold (3276)

3. **Verify behavior:**
   - B3 multiple times → logging continues
   - Analog > 3276 → logging stops immediately

### For Development/Testing:

If you want to stop logging without triggering ES:
- **Option 1:** Close the plot window
- **Option 2:** Press Ctrl+C in console
- **Option 3:** Modify analog threshold in firmware (not recommended)

---

## 📋 Quick Reference Card

### To Check Analog Value:
**Press B3** → See value in console → Logging continues

### To Stop Logging:
**Move analog joystick to max** → Analog > 3276 → ES sent → Logging stops

### Alternative Stop:
**Close plot window** → Logging stops gracefully

---

## 🔄 Updated Script Behavior

The Python script now correctly:

✅ Recognizes `ES` command as stop trigger  
✅ Treats `AN` command as informational only  
✅ Displays analog value when B3 is pressed  
✅ Continues logging after B3 press  
✅ Stops logging when ES is detected  
✅ Shows ES threshold value in console  

---

## ✅ Summary

**B3 Button:**
- Purpose: Print analog value for information
- Command: `timestamp,AN,value`
- Effect: Console message only
- Logging: Continues

**ES Condition:**
- Purpose: Stop logging session
- Trigger: Analog input > 3276
- Command: `timestamp,ES,AN:value`
- Effect: Stops logging, saves CSV
- Logging: Stops

**Now you understand the correct behavior!** 🎯
