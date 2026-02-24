#!/usr/bin/env python3
import json
import os
import queue
import signal
import sys


RUNNING = True


def handle_stop(_sig, _frame):
    global RUNNING
    RUNNING = False


def main():
    signal.signal(signal.SIGTERM, handle_stop)
    signal.signal(signal.SIGINT, handle_stop)

    try:
        import sounddevice as sd
    except Exception:
        print("ERROR: missing python package 'sounddevice' (pip install sounddevice)", flush=True)
        return 1

    try:
        from vosk import KaldiRecognizer, Model
    except Exception:
        print("ERROR: missing python package 'vosk' (pip install vosk)", flush=True)
        return 1

    model_path = os.environ.get("VOSK_MODEL_PATH", "models/vosk-model-en-us-0.22")
    if not os.path.isdir(model_path):
        print(
            "ERROR: Vosk model not found. Set VOSK_MODEL_PATH or place model at "
            + model_path,
            flush=True,
        )
        return 1

    try:
        model = Model(model_path=model_path)
    except Exception as exc:
        print(f"ERROR: failed to load model: {exc}", flush=True)
        return 1

    sample_rate = 16000
    audio_queue = queue.Queue()

    def audio_callback(indata, _frames, _time, status):
        if status:
            return
        audio_queue.put(bytes(indata))

    recognizer = KaldiRecognizer(model, sample_rate)
    recognizer.SetWords(False)

    try:
        with sd.RawInputStream(
            samplerate=sample_rate,
            blocksize=8000,
            dtype="int16",
            channels=1,
            callback=audio_callback,
        ):
            print("voice listener ready", flush=True)
            while RUNNING:
                try:
                    data = audio_queue.get(timeout=0.2)
                except queue.Empty:
                    continue

                if recognizer.AcceptWaveform(data):
                    result = json.loads(recognizer.Result())
                    text = result.get("text", "").strip()
                else:
                    partial = json.loads(recognizer.PartialResult())
                    text = partial.get("partial", "").strip()

                if text:
                    print(text, flush=True)
    except Exception as exc:
        print(f"ERROR: microphone stream failed: {exc}", flush=True)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
