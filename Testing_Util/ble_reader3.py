import asyncio
import struct
import time

from bleak import BleakClient, BleakScanner

# --- Configuration ---
DEVICE_NAME = "IMU_Ring"
CHAR_UUID = "0000fff4-0000-1000-8000-00805f9b34fb"
MAX_SAMPLES = 3000
OUTPUT_FILENAME = "imu_data.csv"

# Global state tracking
sample_count = 0
start_time = None  # Holds the baseline timestamp of the first data point
stop_event = asyncio.Event()
file_handle = None


def notification_handler(sender, data):
    global sample_count, start_time

    # Ensure we have the full 96-byte payload (8 samples * 12 bytes)
    if len(data) == 96:
        current_time = time.time()

        # Lock in the start time on the very first packet
        if start_time is None:
            start_time = current_time

        # Calculate milliseconds elapsed since the first data point
        # Note: All 8 samples in this packet will share the same arrival timestamp
        elapsed_ms = (current_time - start_time) * 1000

        # Loop through the 96 bytes in 12-byte chunks
        for i in range(8):
            # Slice out the current 12-byte chunk
            chunk = data[i * 12 : (i + 1) * 12]

            # Unpack 6 signed 16-bit integers (Little Endian)
            ax, ay, az, gx, gy, gz = struct.unpack("<6h", chunk)

            # Write directly to the CSV file
            file_handle.write(f"{int(elapsed_ms)},{ax},{ay},{az},{gx},{gy},{gz}\n")

            sample_count += 1

            # Provide a terminal update every 100 samples
            if sample_count % 100 == 0:
                print(f"Collected {sample_count}/{MAX_SAMPLES} samples...")

            # Trigger the main loop to exit once we hit the target
            if sample_count >= MAX_SAMPLES:
                stop_event.set()
                break  # Stop processing mid-packet if we hit the limit

    else:
        print(f"Warning: Received malformed packet of length: {len(data)} bytes")


async def main():
    global file_handle
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

    # Open the file in write mode
    with open(OUTPUT_FILENAME, "w") as f:
        file_handle = f
        # Write CSV Headers (timestamp is now ms_since_start)
        file_handle.write(
            "ms_since_start,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n"
        )

        async with BleakClient(device) as client:
            print("Connected!")

            # Subscribe to the notifications on the Characteristic
            print(f"Subscribing to {CHAR_UUID}...")
            await client.start_notify(CHAR_UUID, notification_handler)
            print(f"Subscribed! Waiting to collect {MAX_SAMPLES} samples...\n")

            # Pause this async block until stop_event.set() is called in the handler
            await stop_event.wait()

            # Clean up and disconnect
            await client.stop_notify(CHAR_UUID)
            print(
                f"\nFinished listening. Data saved to {OUTPUT_FILENAME}. Disconnecting."
            )


if __name__ == "__main__":
    import sys

    if sys.platform.startswith("win"):
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

    asyncio.run(main())
