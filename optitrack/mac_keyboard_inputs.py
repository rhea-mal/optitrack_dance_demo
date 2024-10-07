from pynput.keyboard import Key, Listener
import redis 

r = redis.Redis(host='localhost', port=6379, decode_responses=True)

def on_press(key):
    # print(f'{key} pressed')
    if hasattr(key, "char") and key.char == "r":
        print("Robot Reset")
        r.set("sai2::sim::reset", 1)
    elif hasattr(key, "char") and key.char == "1":
        print("User 1 Ready")
        r.set("sai2::optitrack::user_1_ready", 1)
    elif hasattr(key, "char") and key.char == "2":
        print("User 2 Ready")
        r.set("sai2::optitrack::user_2_ready", 1)
    elif hasattr(key, "char") and key.char == "0":
        print("Single User Ready")
        r.set("sai2::optitrack::user_ready", 1)

def on_release(key):
    if key == Key.esc:
        # Stop listener
        return False

# Collect events until released
with Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
