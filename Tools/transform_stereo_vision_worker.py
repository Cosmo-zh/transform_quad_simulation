"""Keep image decoding, detection and PNG writes outside the Offboard process."""
import json
import multiprocessing as mp
from pathlib import Path
import queue
import time


def _put_latest(output, message):
    try:
        output.put_nowait(message)
    except queue.Full:
        try:
            output.get_nowait()
        except queue.Empty:
            pass
        try:
            output.put_nowait(message)
        except queue.Full:
            pass


def _work(output, clipped, stop, directory, image_interval):
    from transform_stereo_perception import StereoStream, GateDetector
    import cv2
    stream, detector = StereoStream(), GateDetector()
    directory = Path(directory)
    last_saved = -float('inf')
    stats = dict(frames=0, image_pairs=0, opencv_threads=cv2.getNumThreads(),
                 max_processing_seconds=0, max_png_seconds=0)
    try:
        while not stop.is_set():
            pair = stream.next_pair()
            if pair is None:
                stop.wait(.005)
                continue
            started = time.monotonic()
            stamp, left, right = pair
            gate = detector.detect(left, right, stamp, allow_clipped=clipped.is_set())
            _put_latest(output, dict(gate=gate, produced=time.monotonic()))
            saved = time.monotonic()
            if image_interval > 0 and saved-last_saved >= image_interval:
                frames = directory / 'frames'
                frames.mkdir(exist_ok=True)
                for side, rgb in [('left', left), ('right', right)]:
                    if not cv2.imwrite(str(frames / f'{stamp:016.6f}_{side}.png'), rgb[:, :, ::-1]):
                        raise RuntimeError('Could not save stereo image')
                stats['image_pairs'] += 1
                last_saved = saved
            if gate and not (directory / 'left.png').exists():
                for side, rgb in [('left', left), ('right', right)]:
                    if not cv2.imwrite(str(directory / f'{side}.png'), rgb[:, :, ::-1]):
                        raise RuntimeError('Could not save first stereo image')
            stats['max_png_seconds'] = max(stats['max_png_seconds'], time.monotonic()-saved)
            stats['max_processing_seconds'] = max(stats['max_processing_seconds'], time.monotonic()-started)
            stats['frames'] += 1
    except BaseException as exc:
        _put_latest(output, dict(error=str(exc) or type(exc).__name__))
        raise
    finally:
        (directory / 'vision.json').write_text(json.dumps(stats, indent=2)+'\n')
        # Do not wait for the parent to drain the final queued measurement.
        output.cancel_join_thread()


class VisionWorker:
    def __init__(self, directory, image_interval):
        context = mp.get_context('spawn')
        self.output = context.Queue(maxsize=2)
        self.clipped = context.Event()
        self.stop_event = context.Event()
        self.process = context.Process(target=_work, args=(self.output, self.clipped, self.stop_event,
                                                         str(directory), image_interval))
        self.process.start()

    def next_gate(self, allow_clipped=False):
        self.clipped.set() if allow_clipped else self.clipped.clear()
        newest = None
        while True:
            try:
                message = self.output.get_nowait()
            except queue.Empty:
                break
            if 'error' in message:
                raise RuntimeError('Vision worker: '+message['error'])
            newest = message
        if not self.process.is_alive():
            raise RuntimeError(f'Vision worker exited: {self.process.exitcode}')
        if newest and time.monotonic()-newest['produced'] < .5:
            return newest['gate']
        return None

    def close(self):
        self.stop_event.set()
        self.process.join(timeout=5)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=3)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=3)
        self.output.close()
