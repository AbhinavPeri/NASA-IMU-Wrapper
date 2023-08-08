import asyncio
import time
import threading

async def simulate_async_task(t):
    await asyncio.sleep(t)
    # busy_wait(t)
    return "Hello World"


def busy_wait(t):
    start = time.time()
    while time.time() - start < t:
        pass


async def simulated_task_loop(task_time, async_task_time, frequency):
    expected_wake_time = time.time()
    prev_offset = None
     
    task = asyncio.create_task(simulate_async_task(async_task_time))
    while True:
        start = time.time()
        offset_time = start - expected_wake_time
        
        print("=" * 20)
        print(offset_time)
        
        if prev_offset and abs(offset_time - prev_offset) > 0.002:
            print("Task is not waking up consistently. Unable to meet specified frequency")
        prev_offset = offset_time
        
        # Manage asynchronous task
        if task.done():
            if task.cancelled():
                print("Async task cancelled")
            else:
                print(task.result())
            task = asyncio.create_task(simulate_async_task(async_task_time))

        # Perform main task
        busy_wait(task_time)
        
        sleep_time = max(1/frequency - (time.time() - max(start, 0)) - offset_time, 0)
        expected_wake_time += 1/frequency
        if sleep_time == 0:
            print("Task taking longer than expected. Unable to meet specified frequency")
        
        print(sleep_time)
        
        await asyncio.sleep(max(sleep_time - 0.001, 0))


def run_main_loop():
    task_time = 0.005
    async_task_time = 0.01
    frequency = 50

    asyncio.run(simulated_task_loop(task_time, async_task_time, frequency))


if __name__ == "__main__":
    thread = threading.Thread(target=run_main_loop, daemon=True)
    thread.start()

    while True:
         busy_wait(0.005)
         time.sleep(0.015)

