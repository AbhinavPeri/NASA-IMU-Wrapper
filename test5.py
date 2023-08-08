import asyncio
import time


async def simulate_async_task(t):
    await asyncio.sleep(t)
    return "Hello World"


def busy_wait(t):
    start = time.monotonic()
    while time.monotonic() - start < t:
        pass


async def simulated_task_loop(task_time, async_task_time, frequency):
    expected_wake_time = time.monotonic()
    prev_offset = None
     
    task = asyncio.create_task(simulate_async_task(async_task_time))
    while True:
        start = time.monotonic()
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
        
        sleep_time = max(1/frequency - (time.monotonic() - max(start, 0)) - offset_time, 0)
        expected_wake_time += 1/frequency
        if sleep_time == 0:
            print("Task taking longer than expected. Unable to meet specified frequency")
        
        print(sleep_time)
        
        await asyncio.sleep(max(sleep_time - 0.001, 0))


if __name__ == "__main__":
    
    task_time = 0.001
    async_task_time = 0.01
    frequency = 100

    asyncio.run(simulated_task_loop(task_time, async_task_time, frequency))
