
# py3
import socket

UDP_IP = "127.0.0.1"
# UDP_IP = "192.168.72.152"
# UDP_IP = "221.120.83.108"

UDP_PORT = 9930
# MESSAGE = b"Hello, World!"
# MESSAGE = b"Mr;1,0,1.73,0.43,1.74,diff,0,0.5,ivam_3F;2,3,2.48,-5.01,1.74,diff,0,0.5,ivam_3F,2;E" #line
# MESSAGE = b"Mr;1,0,-0.17,-0.44,0.10,diff,1,0.5,ivam_3F;2,1,3.61,0.35,1.68,diff,0,0.5,ivam_3F;3,3,4.01,-3.44,1.68,diff,0,0.5,ivam_3F,2;E" #one turn 
# MESSAGE = b"Mr;1,0,-0.17,-0.44,0.10,diff,1,0.5,ivam_3F;2,1,3.61,0.35,1.68,diff,1,0.5,ivam_3F;3,1,4.01,-3.44,1.68,diff,0,0.5,ivam_3F;4,3,1.31,-3.73,0.12,diff,0,0.5,ivam_3F,2;E" #two turn
# MESSAGE = b"Mr;1,0,-0.17,-0.44,0.10,diff,1,0.5,ivam_3F;2,1,3.61,0.35,1.68,diff,0,0.5,ivam_3F;3,1,3.77,-0.85,-1.46,diff,1,0.5,ivam_3F;4,1,4.01,-3.44,1.68,diff,1,0.5,ivam_3F;5,1,1.31,-3.73,0.12,diff,0,0.5,ivam_3F;6,3,0.72,-0.358,-1.45,diff,0,0.5,ivam_3F,2;E" #o turn
MESSAGE = b"Mr;1,0,2.05, -2.39,1.22,diff,0,0.5,ivam_3F;2,3,2.65, 0.23, 0.74,diff,0,0.5,ivam_3F;4,3, 1.01,-2.98,1.69,diff,0,0.5,ivam_3F,2;E" #switch_test

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))