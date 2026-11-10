// Standalone StreamHeader(id1) is a state-only probe handled by runtime dispatch.
profile version 1;
endpoint { envelope = native_packet; }
send ArmCommand { delivery = unreliable; }
send HandCommand { delivery = unreliable; }
latest ArmState { delivery = unreliable; }
latest HandState { delivery = unreliable; }
latest FaultSnapshot { delivery = unreliable; }
latest Diagnostics { delivery = unreliable; }
