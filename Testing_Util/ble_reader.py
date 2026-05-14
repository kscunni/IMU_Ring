import asyncio
from bleak import BleakScanner, BleakClient

# --- Configuration ---
# The name your CC2340R5 is broadcasting. 
# If you didn't change it in SysConfig, it usually defaults to "Basic BLE" or "Project Zero"
DEVICE_NAME = "BLE Ring" 

# The 128-bit UUID for Characteristic 4 (0xFFF4)
CHAR_UUID = "0000fff4-0000-1000-8000-00805f9b34fb"

def notification_handler(sender, data):
    """This function is called instantly whenever the CC2340R5 sends a notification."""
    print(f"--- New Packet Received ---")
    print(f"Length: {len(data)} bytes")
    
    # Print the first 16 bytes in HEX format to verify your hardcoded data
    # hex_data = " ".join(f"{b:02X}" for b in data[:16])
    # print(f"Data (First 16 bytes): {hex_data} ...\n")
    print(data.decode())

async def main():
    print(f"Scanning for {DEVICE_NAME}...")
    
    # Scan for the device by its broadcast name
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)

    if not device:
        print(f"\n Could not find '{DEVICE_NAME}'.")
        print("Make sure the board is powered on, advertising, and not connected to your phone!")
        return

    print(f"\n Found {DEVICE_NAME} ({device.address}). Connecting...")

    # Connect to the device
    async with BleakClient(device) as client:
        print(" Connected!")
        
        # Subscribe to the notifications on Characteristic 4
        print(f"Subscribing to {CHAR_UUID}...")
        await client.start_notify(CHAR_UUID, notification_handler)
        print(" Subscribed! Waiting for data...\n")
        
        # Keep the script running for 60 seconds to listen for incoming data
        await asyncio.sleep(60)
        
        # Clean up and disconnect
        await client.stop_notify(CHAR_UUID)
        print("Finished listening. Disconnecting.")

if __name__ == "__main__":
    # Windows requires this specific event loop policy for Bleak sometimes
    import sys
    if sys.platform.startswith('win'):
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
        
    asyncio.run(main())