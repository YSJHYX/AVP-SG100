from threading import Lock

import numpy as np

from avp_stream.grpc_msg import handtracking_pb2
from avp_stream.streamer import VisionProStreamer, YUP2ZUP


def matrix_message(matrix):
    message = handtracking_pb2.Matrix4x4()
    for row in range(4):
        for column in range(4):
            setattr(message, f"m{row}{column}", float(matrix[row, column]))
    return message


def hand_update(source_sequence=8, right_valid=True, right_wrist=None):
    update = handtracking_pb2.HandUpdate(
        source_frame_sequence=source_sequence,
        left_hand_tracked=False,
        right_hand_tracked=right_valid,
    )
    identity = np.eye(4)
    update.Head.CopyFrom(matrix_message(identity))
    update.left_hand.wristMatrix.CopyFrom(matrix_message(identity))
    update.right_hand.wristMatrix.CopyFrom(matrix_message(right_wrist if right_wrist is not None else identity))
    for _ in range(27):
        update.left_hand.skeleton.jointMatrices.append(matrix_message(identity))
        update.right_hand.skeleton.jointMatrices.append(matrix_message(identity))
    return update


def bare_streamer():
    streamer = VisionProStreamer.__new__(VisionProStreamer)
    streamer.axis_transform = YUP2ZUP
    streamer.origin = "avp"
    streamer.ht_backend = "grpc"
    streamer._webrtc_hand_ready = False
    streamer._latest_lock = Lock()
    streamer._markers_lock = Lock()
    streamer._stylus_lock = Lock()
    streamer._detected_markers = {}
    streamer._tracked_images = {}
    streamer._stylus_data = None
    streamer._transport_receive_sequence = 0
    streamer._cross_network_mode = False
    streamer.record = False
    streamer.recording = []
    streamer.latest = None
    streamer.verbose = False
    return streamer


def test_source_sequence_stays_stable_across_cached_queries():
    streamer = bare_streamer()
    streamer._process_hand_update(hand_update(source_sequence=8))

    first = streamer.get_latest()
    for _ in range(1000):
        cached = streamer.get_latest()
        assert cached.frame_sequence == first.frame_sequence == 8
        assert cached.received_monotonic_ns == first.received_monotonic_ns
        assert cached.right_hand_valid is True


def test_transport_sequence_increments_only_when_packet_is_processed():
    streamer = bare_streamer()
    update = hand_update(source_sequence=0, right_valid=False)

    streamer._process_hand_update(update)
    first = streamer.get_latest()
    streamer._process_hand_update(update)
    second = streamer.get_latest()

    assert first.frame_sequence == 1
    assert second.frame_sequence == 2
    assert first.right_hand_valid is False
    assert second.right_hand_valid is False
    assert second.received_monotonic_ns >= first.received_monotonic_ns


def test_repeated_transport_packet_with_same_source_frame_keeps_source_identity():
    streamer = bare_streamer()
    update = hand_update(source_sequence=91)

    streamer._process_hand_update(update)
    first = streamer.get_latest()
    streamer._process_hand_update(update)
    second = streamer.get_latest()

    assert first.frame_sequence == second.frame_sequence == 91
    assert first["transport_receive_sequence"] == 1
    assert second["transport_receive_sequence"] == 2


def test_legacy_app_infers_validity_from_wrist_matrix():
    # Old app builds send neither tracked flags nor source_frame_sequence.
    # Validity must fall back to the wrist matrix: non-identity -> tracked.
    streamer = bare_streamer()
    posed_wrist = np.eye(4)
    posed_wrist[:3, 3] = [0.1, 0.2, 0.9]
    streamer._process_hand_update(
        hand_update(source_sequence=0, right_valid=False, right_wrist=posed_wrist)
    )

    assert streamer.get_latest().right_hand_valid is True

    streamer._process_hand_update(hand_update(source_sequence=0, right_valid=False))
    assert streamer.get_latest().right_hand_valid is False


def main():
    test_source_sequence_stays_stable_across_cached_queries()
    test_transport_sequence_increments_only_when_packet_is_processed()
    test_repeated_transport_packet_with_same_source_frame_keeps_source_identity()
    test_legacy_app_infers_validity_from_wrist_matrix()
    print("AVP STREAMER FRAME METADATA: PASS")


if __name__ == "__main__":
    main()
