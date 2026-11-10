profile version 1;
endpoint { envelope = native_packet; }
send PeerArmState { delivery = unreliable; }
latest PeerArmState { delivery = unreliable; }
