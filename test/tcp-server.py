
import socket
import struct
s = socket.socket()         
 
s.bind(('192.168.43.129', 8090 ))
s.listen(0)                 
print("Starting server")
while True:
 
    client, addr = s.accept()
    print(f"Connection accepted from {client}")
    while True:
        content = client.recv(4096*2)
        if len(content) > 0:
            try:
                data_struct = struct.Struct('>8192B')
                read_recover = list(data_struct.unpack(received))

                for x in read_recover:
                    print(x)
            except Exception as e:
                print(e)
