#!/usr/bin/env python3
"""
GO2 Audio Perception Node
Captures audio from microphone array and estimates sound-source azimuth
using GCC-PHAT (Generalized Cross-Correlation with Phase Transform).
Publishes bearing angle for the robot to orient toward the caller.

音频来源通过参数 audio_source 控制:
  - "local" (默认): 使用本地 PyAudio 设备 (4 通道麦克风阵列)
  - "ros":          订阅 /audio/raw 话题 (来自 go2-audio-node 的 WebRTC 单声道音频)
"""

import numpy as np
import pyaudio
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Float32, Int16MultiArray

# Physical microphone array geometry (meters)
# Assumes a 4-mic linear array mounted on the GO2's head
MIC_SPACING = 0.05  # 5 cm between adjacent mics
SPEED_OF_SOUND = 343.0  # m/s at ~20°C


def compute_channel_energy(samples: np.ndarray) -> float:
    """Return mean-square energy without integer overflow."""
    samples_f32 = samples.astype(np.float32, copy=False)
    return float(np.mean(np.square(samples_f32)))


def gcc_phat(sig1: np.ndarray, sig2: np.ndarray, fs: int, max_tau: float) -> float:
    """
    Generalized Cross-Correlation with Phase Transform.
    Returns the estimated time delay (seconds) between sig1 and sig2.
    """
    n = sig1.shape[0] + sig2.shape[0] - 1
    # Pad to next power of 2 for FFT efficiency
    n_fft = 1
    while n_fft < n:
        n_fft <<= 1

    S1 = np.fft.rfft(sig1, n=n_fft)
    S2 = np.fft.rfft(sig2, n=n_fft)

    # Cross-power spectrum
    R = S1 * np.conj(S2)

    # Phase transform (whiten the spectrum)
    denom = np.abs(R)
    denom[denom < 1e-10] = 1e-10
    R_phat = R / denom

    # Inverse FFT -> correlation
    cc = np.fft.irfft(R_phat, n=n_fft)
    cc = np.concatenate((cc[-(n_fft // 2):], cc[: n_fft // 2 + 1]))

    max_shift = int(np.ceil(max_tau * fs))
    center = n_fft // 2
    cc_slice = cc[center - max_shift : center + max_shift + 1]

    peak_idx = np.argmax(np.abs(cc_slice))
    tau = (peak_idx - max_shift) / float(fs)
    return tau


class AudioPerceptionNode(Node):
    def __init__(self):
        super().__init__("audio_perception_node")

        # Parameters
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("chunk_duration_ms", 200)
        self.declare_parameter("n_mics", 4)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("energy_threshold", 500.0)
        # 新增参数: "local" 使用 PyAudio 本地设备; "ros" 订阅 /audio/raw
        self.declare_parameter("audio_source", "ros")
        self.declare_parameter("audio_topic", "/audio/raw")

        self.fs = self.get_parameter("sample_rate").value
        self.chunk_ms = self.get_parameter("chunk_duration_ms").value
        self.n_mics = self.get_parameter("n_mics").value
        self.energy_threshold = self.get_parameter("energy_threshold").value
        self.audio_source = self.get_parameter("audio_source").value
        self.audio_topic = self.get_parameter("audio_topic").value

        self.chunk_size = int(self.fs * self.chunk_ms / 1000)

        # Publishers
        self.bearing_pub = self.create_publisher(Float32, "/go2/audio/bearing_deg", 10)
        self.raw_pub = self.create_publisher(Vector3Stamped, "/go2/audio/sound_source", 10)
        self.audio_pub = self.create_publisher(Int16MultiArray, "/go2/audio/mono_raw", 10)

        # 最近一次从 ROS 话题收到的音频帧缓存
        self._latest_ros_frame = None

        # 根据 audio_source 选择音频输入方式
        self.pa = None
        self.stream = None
        if self.audio_source == "ros":
            self.get_logger().info(
                f"Audio source = ROS topic '{self.audio_topic}'. "
                "Expecting Int16MultiArray with 48kHz mono frames."
            )
            self.create_subscription(
                Int16MultiArray,
                self.audio_topic,
                self.audio_topic_callback,
                10,
            )
        else:
            self.get_logger().info("Audio source = local PyAudio device.")
            self.pa = pyaudio.PyAudio()
            self.stream = self.pa.open(
                format=pyaudio.paInt16,
                channels=self.n_mics,
                rate=self.fs,
                input=True,
                frames_per_buffer=self.chunk_size,
            )

        # Processing timer
        rate = self.get_parameter("publish_rate_hz").value
        self.timer = self.create_timer(1.0 / rate, self.process_audio)

        self.get_logger().info("AudioPerceptionNode started. Listening for sound sources.")

    def audio_topic_callback(self, msg: Int16MultiArray):
        """接收 /audio/raw 的单声道 int16 音频帧并缓存。"""
        if msg.data:
            self._latest_ros_frame = np.asarray(msg.data, dtype=np.int16)

    def _read_local_audio(self) -> np.ndarray:
        """从本地 PyAudio 设备读取多通道音频。"""
        raw = self.stream.read(self.chunk_size, exception_on_overflow=False)
        return np.frombuffer(raw, dtype=np.int16).reshape(-1, self.n_mics)

    def _read_ros_audio(self) -> np.ndarray:
        """
        从 /audio/raw 缓存中取出单声道音频，并 reshape 成 (N, 1) 以复用下游逻辑。
        注意: WebRTC 音频是 48kHz 单声道，与本地 4 通道阵列不兼容，
        GCC-PHAT 方位估计在单声道下不可用，仅做能量检测和转发。
        """
        if self._latest_ros_frame is None:
            return None
        return self._latest_ros_frame.reshape(-1, 1)

    def process_audio(self):
        try:
            if self.audio_source == "ros":
                audio = self._read_ros_audio()
                if audio is None:
                    return  # 还没收到话题数据
            else:
                audio = self._read_local_audio()
        except Exception as e:
            self.get_logger().warn(f"Audio read error: {e}")
            return

        # Energy gate — only process if signal is loud enough
        energy = compute_channel_energy(audio[:, 0])
        if energy < self.energy_threshold:
            return

        # Publish mono audio (first channel)
        audio_msg = Int16MultiArray()
        audio_msg.data = audio[:, 0].tolist()
        self.audio_pub.publish(audio_msg)

        # 只有多通道 (n_mics >= 2) 时才做 GCC-PHAT 方位估计
        if audio.shape[1] < 2:
            self.get_logger().debug(
                "Single-channel audio; skipping GCC-PHAT bearing estimation."
            )
            return

        # Use first two mics for primary TDOA estimate
        mic0 = audio[:, 0].astype(np.float32)
        mic1 = audio[:, 1].astype(np.float32)

        max_tau = MIC_SPACING / SPEED_OF_SOUND
        tau = gcc_phat(mic0, mic1, self.fs, max_tau)

        # Clamp to physical range
        tau = np.clip(tau, -max_tau, max_tau)

        # Convert TDOA to azimuth angle (radians)
        sin_theta = (tau * SPEED_OF_SOUND) / MIC_SPACING
        sin_theta = np.clip(sin_theta, -1.0, 1.0)
        azimuth_rad = np.arcsin(sin_theta)
        azimuth_deg = np.degrees(azimuth_rad)

        # Publish bearing
        bearing_msg = Float32()
        bearing_msg.data = float(azimuth_deg)
        self.bearing_pub.publish(bearing_msg)

        # Publish 3D unit vector on horizontal plane
        source_msg = Vector3Stamped()
        source_msg.header.stamp = self.get_clock().now().to_msg()
        source_msg.header.frame_id = "base_link"
        source_msg.vector.x = float(np.cos(azimuth_rad))
        source_msg.vector.y = float(np.sin(azimuth_rad))
        source_msg.vector.z = 0.0
        self.raw_pub.publish(source_msg)

        self.get_logger().debug(f"Sound source bearing: {azimuth_deg:.1f} deg")

    def destroy_node(self):
        if self.stream is not None:
            self.stream.stop_stream()
            self.stream.close()
        if self.pa is not None:
            self.pa.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
