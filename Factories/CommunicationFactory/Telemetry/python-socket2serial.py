# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# implementation of virtual serial over udp from https://github.com/GuLinux/python-socket2serial/tree/master
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
import os
import pty
import socket
import select
import argparse

parser = argparse.ArgumentParser(description='Creates a virtual pty for a remote tcp/udp socket')
parser.add_argument('rhost', type=str)
parser.add_argument('rport', type=int)
parser.add_argument('lport', type=int)
parser.add_argument('-u', '--udp', action='store_true')
args = parser.parse_args()

rhost = args.rhost
rport = args.rport
lport = args.lport
lhost = ""
def main():
    s_local = socket.socket(socket.AF_INET, socket.SOCK_DGRAM if args.udp else socket.SOCK_STREAM)
    s_local.bind((lhost, lport))
    s_remote = socket.socket(socket.AF_INET, socket.SOCK_DGRAM if args.udp else socket.SOCK_STREAM)
    s_remote.connect((rhost, rport))
    master, slave = pty.openpty()
#    tty.setraw(master, termios.TCSANOW)
    print('PTY: Opened {} for {}:{}'.format(os.ttyname(slave), lhost, lport))
    mypoll = select.poll()
    mypoll.register(s_local, select.POLLIN)
    mypoll.register(master, select.POLLIN)

    try:
        while True:
            fdlist = mypoll.poll(1000)
            for fd,event in fdlist:
                data = os.read(fd, 4096)
                print(data)
                write_fd = s_remote.fileno() if fd == master else master
                try:
                    os.write(write_fd, data)
                except:
                    print("Couldnt write data")
    finally:
        s_local.close()
        s_remote.close()
        os.close(master)
        os.close(slave)
    
if __name__ == "__main__":
    main()
