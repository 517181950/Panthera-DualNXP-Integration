profile version 1;
endpoint { envelope = native_packet; rpc_role = server; }
rpc Operation {
  request = OperationRequest;
  response = OperationResult;
  request_operation_id = operation_id;
  response_operation_id = operation_id;
  response_status = status;
  request_delivery = reliable;
  response_delivery = reliable;
}
rpc Configure {
  request = ConfigureRequest;
  response = ConfigureResult;
  request_operation_id = operation_id;
  response_operation_id = operation_id;
  response_status = status;
  request_delivery = reliable;
  response_delivery = reliable;
}
send FaultEvent { delivery = reliable; }
