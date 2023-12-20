import can
import random
import time

def generate_random_can_message():
    can_id = random.randint(0, 0x7FF)
    data = [random.randint(0, 0xFF) for _ in range(8)]
    return can.Message(arbitration_id=can_id, data=data, is_extended_id=False)

def publish_random_can_message(bus, interval=1):
    while True:
        msg = generate_random_can_message()
        print(f"Publishing CAN message: {msg}")
        bus.send(msg)
        time.sleep(interval)

if __name__ == "__main__":
    # Create a virtual CAN bus instance
    bus = can.interface.Bus(channel='vcan0', bustype='socketcan')

    try:
        # Publish random CAN messages with a 1-second interval
        publish_random_can_message(bus, interval=1)
    except KeyboardInterrupt:
        print("Script terminated by user.")
    finally:
        # Close the CAN bus when done
        bus.shutdown()
