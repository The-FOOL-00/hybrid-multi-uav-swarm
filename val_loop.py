import os
import subprocess
import time
import sys

def main():
    print("Starting validation loop (5 runs)...")
    for i in range(1, 6):
        print(f"=== RUN {i} ===")
        start_time = time.time()
        env = os.environ.copy()
        env["WEBOTS_HEADLESS"] = "true"
        env["PYTHONUNBUFFERED"] = "1"
        
        proc = subprocess.Popen(["python", "run_simulation.py", "--scenario", "single_drone", "--headless"], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        stdout, stderr = "", ""
        try:
            stdout, stderr = proc.communicate(timeout=180)
            print(f"Run {i} finished naturally in {time.time() - start_time:.1f}s")
        except subprocess.TimeoutExpired as e:
            print(f"Run {i} TIMED OUT after 180s")
            proc.kill()
            stdout, stderr = proc.communicate()
            
        with open(f"val_run_{i}.log", "w", encoding="utf-8") as f:
            f.write(stdout if stdout else "")
            f.write("\n--- STDERR ---\n")
            f.write(stderr if stderr else "")
            
        print(f"Log for run {i} written to val_run_{i}.log")
        
        # Kill any zombie webots
        try:
            subprocess.run(["taskkill", "/F", "/IM", "webotsw.exe", "/T"], capture_output=True)
        except: pass
        
    print("All 5 runs completed.")

if __name__ == "__main__":
    main()
