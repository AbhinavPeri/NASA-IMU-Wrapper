import time
from threading import Thread


wake_up_errors = 0
duration_errors = 0

def busy_wait(t):
    start = time.monotonic()
    while time.monotonic() - start < t:
        pass


def simulated_task_loop(task_time, frequency, t_num):
    global wake_up_errors, duration_errors
    expected_wake_time = time.monotonic()
    prev_offset = None
    while True:
        start = time.monotonic()
        offset_time = start - expected_wake_time
        log_t_num = 1
        if t_num == log_t_num:
            print("=" * 20)
            print(offset_time)
        if prev_offset and abs(offset_time - prev_offset) > 0.002:
            wake_up_errors += 1
            print("Thread %d is not waking up consistently. Unable to meet specified frequency" % t_num)
        prev_offset = offset_time
        busy_wait(task_time)
        sleep_time = max(1/frequency - (time.monotonic() - max(start, 0)) - offset_time, 0)
        expected_wake_time += 1/frequency
        if sleep_time == 0:
            duration_errors += 1
            print("Thread %d taking longer than expected. Unable to meet specified frequency" % t_num)
        if t_num == log_t_num:
            print(sleep_time)
        time.sleep(max(sleep_time - 0.001, 0))


if __name__ == "__main__":

    num_threads = 5
    task_time = 0.015
    frequency = 10
    
    t = None
    for i in range(num_threads):
        t = Thread(target=simulated_task_loop, args=(task_time, frequency, i), daemon=True)
        t.start()
    
    time.sleep(30)
    print("Wake Up Errors: " + str(wake_up_errors) + " Duration Errors: " + str(duration_errors))
