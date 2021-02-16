import time


def sleep(time_to_sleep, stop_event):
    start_time = time.time()
    while not stop_event.isSet():
        now = time.time()
        if now - start_time >= time_to_sleep:
            break
        else:
            time.sleep(0.1)

