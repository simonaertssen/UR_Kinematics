import socket
import time
import binascii

HOST = "192.168.111.2"  # The Robot IP address
PORT_502 = 502  # The MODBUS Port on the robot

print
"Starting Program"
count = 0
program_run = 0

while (True):
    if 1 == 1:
        if program_run == 0:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(10)
                s.connect((HOST, PORT_502))
                time.sleep(0.05)
                print("")
                print("Modbus Read inputs")
                print("Sending: x00 x01 x00 x00 x00 x06 x00 x03 x01 x02 x00 x01")

                #s.send(b"\x00\x01\x00\x00\x00\x06\x00\x03\x00\x00\x00\x01")
                s.send(b"\x00\x01\x00\x00\x00\x06\x00\x03\x01\x02\x00\x01")

                msg = s.recv(1024)
                print("Raw received data = ", msg)
                print("Type: ", type(msg))
                print("Received data as x format = ", repr(msg))  # Print the receive data in \x hex format (notice that data above 32 hex will be represented by the ascii code e.g. 50 will show P
                print("")
                msg = msg.hex()  # Convert the data from \x hex format to plain hex
                print("Hex data received = ", msg)

                print("Strip downto last two bytes")
                byte_1_in = msg[21]
                byte_2_in = msg[20]
                print("byte_1_in = ", byte_1_in)
                print("byte_2_in = ", byte_2_in)

                byte_1_bin_in = bin(int(byte_1_in, 16))  # convert hex to bin
                print("Binary value of input byte 1 = ", byte_1_bin_in)
                byte_1_bin_in = byte_1_bin_in[2:].zfill(4)  # get rid of the 0b and fill with zeros so byte take 4 positions
                print("Remove 0b and make byte occupie 4 positions = ", byte_1_bin_in)
                byte_2_bin_in = bin(int(byte_2_in, 16))  # convert hex to bin
                print("Binary value of input byte 2 = ", byte_2_bin_in)
                byte_2_bin_in = byte_2_bin_in[2:].zfill(4)  # get rid of the 0b and fill with zeros so byte take 4 positions
                print("Remove 0b and make byte occupie 4 positions = ", byte_2_bin_in)
                print("")
                print("Input 0 = ", byte_1_bin_in[3])
                print("Input 1 = ", byte_1_bin_in[2])
                print("Input 2 = ", byte_1_bin_in[1])
                print("Input 3 = ", byte_1_bin_in[0])
                print("Input 4 = ", byte_2_bin_in[3])
                print("Input 5 = ", byte_2_bin_in[2])
                print("Input 6 = ", byte_2_bin_in[1])
                print("Input 7 = ", byte_2_bin_in[0])


                s.close()
                program_run = 0
                break
            except socket.error as socketerror:
                print("Error: ", socketerror)
print("Program finish")

# # Echo client program
# import socket
# import time
#
# HOST = "192.168.0.9" # The remote host
# PORT_502 = 502
#
# print "Starting Program"
#
# count = 0
# home_status = 0
# program_run = 0
#
# while (True):
#  if 1 == 1:
#  if program_run == 0:
#  try:
#  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#  s.settimeout(10)
#  s.connect((HOST, PORT_502))
#  time.sleep(0.05)
#  print ""
#  print "Modbus set reg 128 to 01"
#  print "Sending: x00 x01 x00 x00 x00 x06 x00 x06 x00 x80 x00 x01"
#  s.send ("\x00\x01\x00\x00\x00\x06\x00\x06\x00\x80\x00\x01") #sets register 128 to 1
#  time.sleep(1.00)
#  print ""
#  print "Modbus set reg 128 to 00"
#  print "Sending: x00 x01 x00 x00 x00 x06 x00 x06 x00 x80 x00 x00"
#  s.send ("\x00\x01\x00\x00\x00\x06\x00\x06\x00\x80\x00\x00") #sets register 128 to 0
#  time.sleep(1.00)
#  s.close() #clear buffer
#  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#  s.settimeout(10)
#  s.connect((HOST, PORT_502))
#  time.sleep(0.05)
#  print ""
#  print "Request reg 129"
#  print "Sending: x00 x04 x00 x00 x00 x06 x00 x03 x00 x81 x00 x01"
#  msg = ""
#  s.send ("\x00\x04\x00\x00\x00\x06\x00\x03\x00\x81\x00\x01") #request data from register 128-133 (cartisian data)
#  print "Modbus command send to read reg 129"
#  time.sleep(0.75)
#  print "Received"
#  msg = s.recv(1024)
#  print repr(msg) #Print the receive data in \x hex notation (notice that data above 32 hex will berepresented at the ascii code e.g. 50 will show P
#  print ""
#  msg = msg.encode("hex") #convert the data from \x hex notation to plain hex
#  print msg
#  print ""
#  home_status = 1
#  program_run = 0
#  s.close()
#  except socket.error as socketerror:
#  print("Error: ", socketerror)
# print "Program finish"
