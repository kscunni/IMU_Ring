import asyncio
import struct
import time

from bleak import BleakClient, BleakScanner

# --- Configuration ---
DEVICE_NAME = "IMU_Ring"
CHAR_UUID = "0000fff4-0000-1000-8000-00805f9b34fb"
MAX_SAMPLES = 3000

# Global state tracking
sample_count = 0
start_time = None  # Holds the baseline timestamp of the first data point
stop_event = asyncio.Event()


def notification_handler(sender, data):
    global sample_count, start_time

    # Ensure we have the full 12-byte payload before unpacking
    if len(data) == 12:
        # Unpack 6 signed 16-bit integers (Little Endian)
        ax, ay, az, gx, gy, gz = struct.unpack("<6h", data)

        current_time = time.time()

        # Lock in the start time on the very first packet
        if start_time is None:
            start_time = current_time

        # Calculate milliseconds elapsed since the first data point
        elapsed_ms = int((current_time - start_time) * 1000)

        # Print directly to the terminal with fixed widths for neat columns
        print(f"Accel: {ax:6d}, {ay:6d}, {az:6d} | Gyro: {gx:6d}, {gy:6d}, {gz:6d}")

        sample_count += 1

        # Trigger the main loop to exit once we hit the target
        if sample_count >= MAX_SAMPLES:
            stop_event.set()
    else:
        print(f"Warning: Received malformed packet of length: {len(data)} bytes")


async def main():
    print(f"Scanning for {DEVICE_NAME}...")

    # Scan for the device by its broadcast name
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)

    if not device:
        print(f"\nCould not find '{DEVICE_NAME}'.")
        print(
            "Make sure the board is powered on, advertising, and not connected to your phone!"
        )
        return

    print(f"\nFound {DEVICE_NAME} ({device.address}). Connecting...")

    async with BleakClient(device) as client:
        print("Connected!")

        # Subscribe to the notifications on Characteristic 4
        print(f"Subscribing to {CHAR_UUID}...")
        await client.start_notify(CHAR_UUID, notification_handler)
        print(f"Subscribed! Waiting to collect {MAX_SAMPLES} samples...\n")


        # Pause this async block until stop_event.set() is called in the handler
        await stop_event.wait()

        # Clean up and disconnect
        await client.stop_notify(CHAR_UUID)
        print(f"\nFinished listening. Collected {MAX_SAMPLES} samples. Disconnecting.")


if __name__ == "__main__":
    import sys

    if sys.platform.startswith("win"):
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

    asyncio.run(main())