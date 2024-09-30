import redis
import os
import time
from datetime import datetime

# Redis configuration
REDIS_HOST = '127.0.0.1'
REDIS_PORT = 6379
OUTPUT_FILE = '../recordings/NEW_full_rehearsal2.txt'

# List of specific IDs to monitor
IDS_TO_MONITOR = [5, 23, 2, 47, 11, 27, 49, 8, 45, 50, 32]

# Generate keys for both position and orientation
ALL_KEYS = [f"1::{id_num}::pos" for id_num in IDS_TO_MONITOR] + [f"1::{id_num}::ori" for id_num in IDS_TO_MONITOR]

# Connect to Redis
redis_client = redis.StrictRedis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)

def initialize_output_file():
    """
    Initialize the output file with a header row containing the keys.
    """
    with open(OUTPUT_FILE, 'w') as file:
        header = ['timestamp'] + ALL_KEYS
        file.write('\t'.join(header) + '\n')

def append_to_output_file(data):
    """
    Append a row of data to the output file.
    Each row includes a timestamp followed by the values for each key.
    """
    with open(OUTPUT_FILE, 'a') as file:
        row = [datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]] + [data.get(key, 'None') for key in ALL_KEYS]
        file.write('\t'.join(row) + '\n')

def read_and_append_keys():
    """
    Continuously read from the specified Redis keys and append their values to the output file.
    """
    initialize_output_file()
    while True:
        data = {}
        for key in ALL_KEYS:
            try:
                value = redis_client.get(key)
                if value is not None:
                    data[key] = value
            except redis.ConnectionError as e:
                print(f"Redis connection error: {e}")
                return
        append_to_output_file(data)
        time.sleep(1.0 / 120)  # Maintain a rate of 120 Hz

if __name__ == "__main__":
    read_and_append_keys()
