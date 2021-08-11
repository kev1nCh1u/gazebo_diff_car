
# py3
import socket

UDP_IP = "127.0.0.1"
# UDP_IP = "192.168.72.152"
# UDP_IP = "221.120.83.108"

UDP_PORT = 9930
MESSAGE = b"Mr;1,0, 0.68, -0.11, 0.98,diff,0,0.5,qq;2,1,1.68, 3.00, -0.99,diff,0,0.5,qq;4,3, 6.43,1.65,0.96,diff,0,0.5,qq,2;E" #switch_test
# MESSAGE = b"Mr;1,0, -1.93,-0.44,-3.06,diff,0,0.5,ivam_3F;2,3, -1.94, -0.39,0.09,diff,0,0.5,ivam_3F,2;E"


print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))