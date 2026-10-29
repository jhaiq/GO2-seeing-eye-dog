#!/usr/bin/env python3
"""
GO2 Voice Commander Node
Wake-word detection followed by faster-whisper transcription.
音频来源通过参数 audio_source 控制:
  - "local" (默认): 使用本地 PyAudio 设备
  - "ros":          订阅 /audio/raw 话题 (来自 go2-audio-node 的 WebRTC 音频)
"""

import queue
import threading

import numpy as np
import pyaudio
import rclpy
from rclpy.node import Node
from scipy.signal import resample_poly, butter, filtfilt
from std_msgs.msg import Int16MultiArray, String
from faster_whisper import WhisperModel
import noisereduce as nr

WAKE_WORDS = {"hey robot", "hey go2", "excuse me", "come here", "come"}
COMMAND_MAP = {
    "come": "come here", "come here": "come here", "over here": "come here",
    "follow me": "follow", "follow": "follow",
    "stop": "stop", "wait": "stop", "stay": "stop",
    "help": "help", "help me": "help",
}


def compute_chunk_energy(chunk: np.ndarray) -> float:
    chunk_f32 = chunk.astype(np.float32, copy=False)
    return float(np.mean(np.square(chunk_f32)))


def parse_command(transcript: str):
    t = transcript.strip().lower()
    for trigger, cmd in COMMAND_MAP.items():
        if trigger in t:
            return cmd
    for wake in WAKE_WORDS:
        if wake in t:
            return "come here"
    return None


class VoiceCommanderNode(Node):
    def __init__(self):
        super().__init__("voice_commander_node")

        self.declare_parameter("whisper_model_path", "./models/faster-whisper-base.en")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("listen_duration_sec", 3.0)
        self.declare_parameter("energy_threshold", 800.0)
        self.declare_parameter("audio_source", "local")
        self.declare_parameter("audio_topic", "/audio/raw")
        self.declare_parameter("ros_sample_rate", 48000)
        self.declare_parameter("ros_chunk_sec", 2.0)   # 累积多少秒处理一次

        model_path = self.get_parameter("whisper_model_path").value
        self.fs = self.get_parameter("sample_rate").value
        self.listen_duration = self.get_parameter("listen_duration_sec").value
        self.energy_threshold = self.get_parameter("energy_threshold").value
        self.audio_source = self.get_parameter("audio_source").value
        self.audio_topic = self.get_parameter("audio_topic").value
        self.ros_fs = self.get_parameter("ros_sample_rate").value
        self.ros_chunk_sec = self.get_parameter("ros_chunk_sec").value

        self.get_logger().info(f"Loading faster-whisper model '{model_path}' ...")
        self.whisper_model = WhisperModel(model_path, device="cpu", compute_type="int8")
        self.get_logger().info("faster-whisper model loaded.")

        self.cmd_pub = self.create_publisher(String, "/go2/voice_command", 10)
        self.raw_pub = self.create_publisher(String, "/go2/voice_raw_transcript", 10)

        self.audio_queue = queue.Queue()
        self.running = True
        self.audio_stream = None
        self.pa = None

        if self.audio_source == "ros":
            self.get_logger().info(
                f"Audio source = ROS topic '{self.audio_topic}'. "
                f"Expecting {self.ros_fs}Hz mono frames."
            )
            self.create_subscription(
                Int16MultiArray, self.audio_topic, self.audio_topic_callback, 10)
            # ROS 模式下不需要独立的处理线程, 回调里直接处理
        else:
            self.get_logger().info("Audio source = local PyAudio device.")
            self.pa = pyaudio.PyAudio()
            self.listen_thread = threading.Thread(target=self._audio_listener, daemon=True)
            self.process_thread = threading.Thread(target=self._command_processor, daemon=True)
            self.listen_thread.start()
            self.process_thread.start()

        self.get_logger().info("VoiceCommanderNode ready. Listening for wake word...")

    # ---------- ROS 模式 ----------
    def audio_topic_callback(self, msg: Int16MultiArray):
        chunk = np.asarray(msg.data, dtype=np.int16)
        if chunk.size == 0:
            return

        if not hasattr(self, "_ros_buffer"):
            self._ros_buffer = []
            self._frames_per_chunk = max(1, int(self.ros_fs * self.ros_chunk_sec / chunk.size))

        self._ros_buffer.append(chunk)
        if len(self._ros_buffer) < self._frames_per_chunk:
            return

        merged = np.concatenate(self._ros_buffer)
        self._ros_buffer = []

        # 48kHz -> 16kHz
        audio_48k = merged.astype(np.float32) / 32768.0
        audio_16k = resample_poly(audio_48k, 1, 3)

        # 降噪
        cleaned = self._preprocess(audio_16k, fs=16000)

        # faster-whisper VAD + 识别
        segments, info = self.whisper_model.transcribe(
            cleaned,
            language="en",
            vad_filter=True,
            vad_parameters=dict(
                threshold=0.35,
                min_speech_duration_ms=250,
                min_silence_duration_ms=400,
                speech_pad_ms=200,
            ),
            beam_size=5,
            initial_prompt="hello, hey go2, come here, hello robot",
        )

        for seg in segments:
            text = seg.text.strip().lower()
            if text:
                self.get_logger().info(f"Transcript: '{text}'")
                raw_msg = String()
                raw_msg.data = text
                self.raw_pub.publish(raw_msg)
                command = parse_command(text)
                if command:
                    cmd_msg = String()
                    cmd_msg.data = command
                    self.cmd_pub.publish(cmd_msg)
                    self.get_logger().info(f"Command published: '{command}'")

    def _preprocess(self, audio, fs=16000):
        """带通滤波 + noisereduce + 归一化。"""
        nyq = fs / 2.0
        b, a = butter(4, [400/nyq, 3400/nyq], btype='band')
        filtered = filtfilt(b, a, audio).astype(np.float32)

        noise_sample = filtered[:int(0.3*fs)]
        if len(noise_sample) < 100:
            noise_sample = filtered[:100]
        reduced = nr.reduce_noise(
            y=filtered, sr=fs, y_noise=noise_sample,
            stationary=True, prop_decrease=0.85)

        peak = np.max(np.abs(reduced))
        if peak > 1e-6:
            reduced = reduced / peak * 0.9
        return reduced

    # ---------- 本地 PyAudio 模式 ----------
    def _audio_listener(self):
        chunk_size = int(self.fs * 0.1)
        self.audio_stream = self.pa.open(
            format=pyaudio.paInt16, channels=1, rate=self.fs,
            input=True, frames_per_buffer=chunk_size)

        buffer = []
        collecting = False
        collect_frames = int(self.listen_duration / 0.1)
        collected = 0

        while self.running:
            try:
                raw = self.audio_stream.read(chunk_size, exception_on_overflow=False)
                chunk = np.frombuffer(raw, dtype=np.int16)
                energy = compute_chunk_energy(chunk)

                if energy > self.energy_threshold and not collecting:
                    collecting = True
                    collected = 0
                    buffer = [chunk]
                elif collecting:
                    buffer.append(chunk)
                    collected += 1
                    if collected >= collect_frames:
                        audio_segment = np.concatenate(buffer).astype(np.float32) / 32768.0
                        self.audio_queue.put(audio_segment)
                        collecting = False
                        buffer = []
            except Exception as e:
                self.get_logger().warn(f"Audio listener error: {e}")

        if self.audio_stream is not None:
            self.audio_stream.stop_stream()
            self.audio_stream.close()
            self.audio_stream = None

    def _command_processor(self):
        while self.running:
            try:
                audio = self.audio_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            try:
                cleaned = self._preprocess(audio, fs=16000)
                segments, info = self.whisper_model.transcribe(
                    cleaned,
                    language="en",
                    vad_filter=True,
                    vad_parameters=dict(
                        threshold=0.35,
                        min_speech_duration_ms=250,
                        min_silence_duration_ms=400,
                        speech_pad_ms=200,
                    ),
                    beam_size=5,
                    initial_prompt="hello, hey go2, come here, hello robot",
                )
                for seg in segments:
                    text = seg.text.strip().lower()
                    if text:
                        self.get_logger().info(f"Transcript: '{text}'")
                        raw_msg = String()
                        raw_msg.data = text
                        self.raw_pub.publish(raw_msg)
                        command = parse_command(text)
                        if command:
                            cmd_msg = String()
                            cmd_msg.data = command
                            self.cmd_pub.publish(cmd_msg)
                            self.get_logger().info(f"Command published: '{command}'")
            except Exception as e:
                self.get_logger().error(f"Transcription error: {e}")

    def destroy_node(self):
        self.running = False
        if self.audio_source == "local":
            self.listen_thread.join(timeout=2.0)
            self.process_thread.join(timeout=2.0)
        if self.audio_stream is not None:
            self.audio_stream.stop_stream()
            self.audio_stream.close()
            self.audio_stream = None
        if self.pa is not None:
            self.pa.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
