import serial
import time

PORT = 'COM5' 
BAUD_RATE = 2400
DATA = """Finance Minister Arun Jaitley Tuesday hit out at former RBI governor Raghuram Rajan for predicting that the next banking crisis would be triggered by MSME lending, saying postmortem is easier than taking action when it was required. Rajan, who had as the chief economist at IMF warned of impending financial crisis of 2008, in a note to a parliamentary committee warned against ambitious credit targets and loan waivers, saying that they could be the sources of next banking crisis. Government should focus on sources of the next crisis, not just the last one.

In particular, government should refrain from setting ambitious credit targets or waiving loans. Credit targets are sometimes achieved by abandoning appropriate due diligence, creating the environment for future NPAs," Rajan said in the note." Both MUDRA loans as well as the Kisan Credit Card, while popular, have to be examined more closely for potential credit risk. Rajan, who was RBI governor for three years till September 2016, is currently."""

try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {PORT} at {BAUD_RATE} baud.")
except Exception as e:
    print(f"Error connecting to serial port: {e}")
    exit()

# Function to send data
def send_data(data):
    total_bits = 0
    start_time = time.time()
    
    for char in data:
        ser.write(char.encode())
        total_bits += 8 
        time.sleep(0.0042)

        elapsed_time = time.time() - start_time
        if elapsed_time > 0:
            speed = total_bits / elapsed_time
            print(f"Transmitting: {char} | Speed: {speed:.2f} bits/sec")

    print("Data transmission complete.")

def receive_data():
    received_bits = 0
    received_data = ""
    start_time = time.time()

    while True:
        char = ser.read().decode()
        if char:
            received_data += char
            received_bits += 8
            elapsed_time = time.time() - start_time
            if elapsed_time > 0:
                speed = received_bits / elapsed_time
                print(f"Receiving: {char} | Speed: {speed:.2f} bits/sec")
        
        if len(received_data) >= len(DATA):
            break
    
    print("Data reception complete.")
    return received_data

try:
    print("Sending data...")
    send_data(DATA)

    print("\nReceiving data...")
    received_text = receive_data()
    print("\nReceived Data:")
    print(received_text)

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    ser.close()
    print("Serial connection closed.")