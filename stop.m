message = rosmessage(pub);
message.Data = [0, 0];
send(pub, message);