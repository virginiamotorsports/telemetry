import threading
import subprocess

def run_script(script_name):
    subprocess.run(["python", script_name])

if __name__ == "__main__":
    recieve_thread = threading.Thread(target=run_script, args=("recieve.py",))
    send_thread = threading.Thread(target=run_script, args=("send.py",))

    recieve_thread.start()
    send_thread.start()

    recieve_thread.join()
    send_thread.join()

    print("Both scripts have finished executing.")