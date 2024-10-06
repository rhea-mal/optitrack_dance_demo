import keyboard
import redis 

r = redis.Redis(host='localhost', port=6379, decode_responses=True)

def on_key_event(keyboard_event):
    # print(f"Key {keyboard_event.name} {keyboard_event.event_type}")
    if (keyboard_event.name == "r" and keyboard_event.event_type == "up"):
        print("Robot Reset")
        r.set("sai2::sim::reset", 1)
    elif (keyboard_event.name == "1" and keyboard_event.event_type == "up"):
        print("User 1 Ready")
        r.set("sai2::optitrack::user_1_ready", 1)
    elif (keyboard_event.name == "2" and keyboard_event.event_type == "up"):
        print("User 2 Ready")
        r.set("sai2::optitrack::user_2_ready", 1)
    elif (keyboard_event.name == "0" and keyboard_event.event_type == "up"):
        print("Single User Ready")
        r.set("sai2::optitrack::user_ready", 1)

# Set up a listener for all key events
keyboard.hook(on_key_event)

# Block the program from exiting immediately
keyboard.wait("esc")
