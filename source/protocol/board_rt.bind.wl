// Standalone StreamHeader(id1) is a state-only probe handled by runtime dispatch.
profile version 1;
endpoint { envelope = native_packet; }
latest ArmCommand { delivery = unreliable; }
latest HandCommand { delivery = unreliable; }
send ArmState { delivery = unreliable; }
send HandState { delivery = unreliable; }
send FaultSnapshot { delivery = unreliable; }
send Diagnostics { delivery = unreliable; }
