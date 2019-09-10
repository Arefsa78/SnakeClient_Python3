import socket
import random
from argparse import ArgumentParser
from src.base.Message import *
from src.World import *
import src.ClientGreedy as c_greedy
import src.ClientRandom as c_random
import src.ClientBest as c_best
import src.YourClient as c_your
import signal

is_run = True


def signal_handler(sig, frame):
    global is_run
    print('You pressed Ctrl+C!')
    is_run = False


signal.signal(signal.SIGINT, signal_handler)


def run():
    parser = ArgumentParser()
    parser.add_argument("-n", "--name", dest="name", type=str, default='team_name' + str(random.randint(0, 10000)),
                        help="Client Name", metavar="NAME")
    parser.add_argument("-c", "--client", dest="client_type", type=str, default='auto',
                        help="greedy, random, hand, best, your", metavar="ClientType")
    parser.add_argument("-p", "--port", dest="server_port", type=int, default=20002,
                        help="server port", metavar="ServerPort")
    parser.add_argument("-s", "--server", dest="server_address", type=str, default='localhost',
                        help="server address", metavar="ServerAddress")
    args = parser.parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1)
    server_address = (args.server_address, args.server_port)
    world = World()
    message_snd = MessageClientConnectRequest(args.name).build()

    while is_run:
        sock.sendto(message_snd, server_address)
        try:
            message_rcv = sock.recvfrom(4096)
        except:
            continue
        message = parse(message_rcv[0])
        if message.type == 'MessageClientConnectResponse':
            if message.id == -1:
                print('your name is duplicated!!!')
                exit(-1)
            print('my id is ' + str(message.id))
            world.set_id(message.id, message.ground_config['goal_id'])
            break

    while is_run:
        try:
            r = sock.recvfrom(4096)
        except:
            continue
        message = parse(r[0])
        if message.type == 'MessageClientDisconnect':
            break
        elif message.type == 'MessageClientWorld':
            world.update(message)
            world.print()

            action = ""
            if args.client_type == 'greedy' or (args.client_type == 'auto' and world.self_id == 1):
                action = c_greedy.get_action(world)
            elif args.client_type == 'random' or (args.client_type == 'auto' and world.self_id >= 2):
                action = c_random.get_action(world)
            elif args.client_type == 'best':
                action = c_best.get_action(world)
            elif args.client_type == 'your':
                action = c_your.get_action(world)
            elif args.client_type == 'hand':
                action = input('enter action (u or d or l or r:')

            sock.sendto(MessageClientAction(string_action=action).build(), server_address)
