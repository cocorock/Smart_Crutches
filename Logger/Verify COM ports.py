"""
Quick COM Port Verification Script
Tests if the crutch devices can be found on COM10 and COM11
"""

import serial.tools.list_ports

print("=" * 60)
print("COM PORT VERIFICATION")
print("=" * 60)
print()

print("Scanning all available COM ports...")
print()

ports = serial.tools.list_ports.comports()

if not ports:
    print("❌ No COM ports found!")
    print("   Make sure devices are paired and connected")
else:
    print(f"✅ Found {len(ports)} COM port(s):")
    print()
    
    found_7036 = False
    found_74d3 = False
    
    for port in ports:
        print(f"Port: {port.device}")
        print(f"  Description: {port.description}")
        print(f"  Hardware ID: {port.hwid}")
        
        # Check for our devices
        if "7036" in str(port) or "7036" in port.description:
            print("  ✅ This is CrutchHMI-BT-7036")
            found_7036 = True
        
        if "74d3" in str(port) or "74d3" in port.description:
            print("  ✅ This is CrutchHMI-BT-74d3")
            found_74d3 = True
        
        print()
    
    print("=" * 60)
    print("VERIFICATION RESULTS:")
    print("=" * 60)
    
    if found_7036:
        print("✅ CrutchHMI-BT-7036 detected")
    else:
        print("⚠️  CrutchHMI-BT-7036 not detected")
        print("   Expected on COM10")
    
    if found_74d3:
        print("✅ CrutchHMI-BT-74d3 detected")
    else:
        print("⚠️  CrutchHMI-BT-74d3 not detected")
        print("   Expected on COM11")
    
    print()
    
    if found_7036 and found_74d3:
        print("🎉 Both devices found! You're ready to run the logger!")
    else:
        print("⚠️  Some devices not detected by search.")
        print("   They may be on COM10 and COM11 but not showing in description.")
        print("   The main script will try direct COM port connection.")

print()
print("=" * 60)
print("MANUAL COM PORT CHECK:")
print("=" * 60)
print("According to your Bluetooth settings:")
print("  CrutchHMI-BT-7036 → COM10")
print("  CrutchHMI-BT-74d3 → COM11")
print()
print("The main logger script will attempt to connect to these ports.")
print()

input("Press Enter to exit...")