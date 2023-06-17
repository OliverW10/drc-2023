# client
import socket
import time
import pygame
import signal
import sys
import struct



SERVER_IP = "127.0.0.1"  # The server's hostname or IP address
PORT = 5000  # The port used by the server

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    s.close()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


def clamp(x, _lo, _hi):
    lo = min(_lo, _hi)
    hi = max(_lo, _hi)
    return min(max(x, lo), hi)

WIDTH = 640
HEIGHT = 480

## initialize pygame and create window
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Manual controller")
clock = pygame.time.Clock()

speed = 0
turn = 0
MAX_SPEED = 1.5
MAX_TURN = 1.5

running = True
while running:

    clock.tick(60)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    keys = pygame.key.get_pressed()
    right = (keys[pygame.K_RIGHT] or keys[pygame.K_d])
    left = keys[pygame.K_LEFT] or keys[pygame.K_a]
    up = keys[pygame.K_UP] or keys[pygame.K_w]
    down = keys[pygame.K_DOWN] or keys[pygame.K_s]

    turn_rate = right - left
    turn_rate *= 2.5 * 1/60

    speed_rate = down - up
    speed_rate *= 1 * 1/60

    # if turn_rate == 0:
    #     decay = 0.5 * 1/60
    #     turn = clamp(0, turn - decay, turn + decay)
    # else:
    turn = clamp(turn + turn_rate, -MAX_TURN, MAX_TURN)

    speed = clamp(speed + speed_rate, -MAX_SPEED, MAX_SPEED)

    if pygame.mouse.get_pressed()[0]:
        pos = pygame.mouse.get_pos()
        turn = MAX_TURN*2*((pos[0] / WIDTH)-0.5)
        speed = MAX_SPEED*2*((pos[1] / HEIGHT)-0.5)

    screen.fill((0, 0, 0))
    pygame.draw.line(screen, (255, 255, 255), (WIDTH//2, 0), (WIDTH//2, HEIGHT))
    pygame.draw.line(screen, (255, 255, 255), (0, HEIGHT//2), (WIDTH, HEIGHT//2))

    _speed = speed if abs(speed) > 0.1 else 0
    circle_pos = (
        round(WIDTH * (0.5 + turn/MAX_TURN/2)),
        round(HEIGHT * (0.5 + _speed/MAX_SPEED/2)),
    )
    pygame.draw.circle(screen, (255, 0, 0), circle_pos, 20, 5)

    s.sendto(struct.pack("dd?", _speed, turn, False), (SERVER_IP, PORT))

    pygame.display.flip()       

s.close()
pygame.quit()