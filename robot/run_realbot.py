from realbot import example_automatic_gait

#Road Balance 키보드 제어
from multiprocessing import Process
from simulation.multiprocess_kb import KeyInterrupt

if __name__ == "__main__":
    try:
        # Keyboard input Process
        KeyInputs = KeyInterrupt()
        KeyProcess = Process(target=KeyInputs.keyInterrupt, args=(1, KeyInputs.key_status, KeyInputs.command_status))
        KeyProcess.start()

        # Main Process 
        example_automatic_gait.main(2, KeyInputs.command_status)
        
        print("terminate KeyBoard Input process")
        if KeyProcess.is_alive():
            KeyProcess.terminate()

    except Exception as e:
        print(e)
    finally:
        print("Done... :)")