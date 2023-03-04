import constants
import sim
import visual
import time

print("Compiling functions...")
start = time.perf_counter()
sim.setup
end = time.perf_counter()
print(f"Compilation completed in {end - start}s")
