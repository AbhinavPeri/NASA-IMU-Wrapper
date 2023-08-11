import time
from multiprocessing import Manager, Process, Queue

class Tester:

    def __init__(self):
        self.p = Process(target=self.update, daemon=True)
        self.p2 = Process(target=self.update2, daemon=True)
        self.manager = Manager()
        self.shared_list = self.manager.list([0, 1, 2])

        self.queue = Queue(1)

    def update(self):
        while True:
            i = 4
            if not self.queue.empty():
                print(self.queue.get())
            self.shared_list[:] = [i, i + 1, i + 2]

    def update2(self):
        while True:
            i = 0
            self.shared_list[:] = [i, i + 1, i + 2, i + 3]

    def get_list(self):
        return self.shared_list

    def run_process(self):
        self.p.start()
        self.p2.start()

    def send_stuff(self):
        self.queue.put("HI")

if __name__ == '__main__':
    tester = Tester()
    tester.run_process()
    while True:
        print(tester.get_list())
        tester.send_stuff()
        time.sleep(0.1)

