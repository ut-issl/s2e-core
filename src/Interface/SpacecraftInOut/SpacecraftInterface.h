#pragma once

int SendToSimWrapper(int port_id, unsigned char* buffer, int offset, int count);
int ReceiveFromSimWrapper(int port_id, unsigned char* buffer, int offset, int count);
int SendToScWrapper(int port_id, unsigned char* buffer, int offset, int count);
int ReceiveFromScWrapper(int port_id, unsigned char* buffer, int offset, int count);
int DigtalReadWrapper(int port_id);
int DigtalWriteWrapper(int port_id, bool isHigh);

int send_to_sim(int port_id, unsigned char* buffer, int offset, int count);
int receive_from_sim(int port_id, unsigned char* buffer, int offset, int count);
int send_to_sc(int port_id, unsigned char* buffer, int offset, int count);
int receive_from_sc(int port_id, unsigned char* buffer, int offset, int count);
int digital_read(int port_id);
int digital_write(int port_id, bool isHigh);
