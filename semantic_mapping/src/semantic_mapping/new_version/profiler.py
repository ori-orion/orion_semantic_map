import time
from typing import Dict, List, Optional


class Profile:
    def __init__(self, name: str):
        self.name = name
        self.times: List[float] = []
        self.started_at: Optional[float] = None

    def start(self):
        if self.started_at is not None:
            raise ValueError("Profiler started twice: " + self.name)
        self.started_at = time.time()

    def record(self):
        end = time.time()
        if self.started_at is None:
            raise ValueError("Profiler recorded without being started: " + self.name)
        self.times.append(end - self.started_at)
        self.started_at = None

    def avg(self) -> float:
        if len(self.times) == 0:
            return 0
        return sum(self.times) / len(self.times)


timers: Dict[str, Profile] = dict()


def start(name: str):
    if name in timers:
        timers[name].start()
    else:
        profiler = Profile(name)
        timers[name] = profiler
        profiler.start()


def record(name: str):
    timers[name].record()


def print_description():
    print("-------------------------------")
    print("------ Profiling Results ------")
    for name in timers:
        print(f"{name} : {timers[name].avg()} ({len(timers[name].times)})")
    print("-------------------------------")
