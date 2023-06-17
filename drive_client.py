# client
import socket
import time
import pygame
import signal
import sys
import struct

MAX_INT = 1<<32

SERVER_IP = "127.0.0.1"
SERVER_PORT = 5000
CLIENT_PORT = 5001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind(("", CLIENT_PORT))
sock.settimeout(20/1000)

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sock.close()
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
font = pygame.font.SysFont(None, 24)

speed = 0
turn = 0
MAX_SPEED = 1.5
MAX_TURN = 1.5
loop_i = 0

enabled = False
connected = False
latency = 0

fps = 30

running = True
while running:

    clock.tick(fps)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                enabled = not enabled
    
    keys = pygame.key.get_pressed()
    right = (keys[pygame.K_RIGHT] or keys[pygame.K_d])
    left = keys[pygame.K_LEFT] or keys[pygame.K_a]
    up = keys[pygame.K_UP] or keys[pygame.K_w]
    down = keys[pygame.K_DOWN] or keys[pygame.K_s]

    turn_rate = right - left
    turn_rate *= 3 * 1/fps

    speed_rate = down - up
    speed_rate *= 2 * 1/fps

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
    line_col = (255, 255, 255) if connected else (255, 0, 0)
    pygame.draw.line(screen, line_col, (WIDTH//2, 0), (WIDTH//2, HEIGHT))
    pygame.draw.line(screen, line_col, (0, HEIGHT//2), (WIDTH, HEIGHT//2))

    _speed = speed if abs(speed) > 0.1 else 0
    circle_pos = (
        round(WIDTH * (0.5 + turn/MAX_TURN/2)),
        round(HEIGHT * (0.5 + _speed/MAX_SPEED/2)),
    )
    circle_col = (0, 255, 0) if enabled else (255, 0, 0)
    pygame.draw.circle(screen, circle_col, circle_pos, 20, 5)



    t1 = round(time.time_ns()/1000)
    sock.sendto(struct.pack("dd?I", _speed, turn, enabled, t1%MAX_INT), (SERVER_IP, SERVER_PORT))
    try:
        msg, _ = sock.recvfrom(4)
        t2 = round(time.time_ns()/1000)
        latency = (t2-t1)/1000
        connected = True
        if loop_i % 30 == 0:
            print(f"latency: {latency}ms")
    except socket.timeout:
        if loop_i % 10 == 0:
            print("didint get response")
        connected = False
        time.sleep(0.1)

    if connected:
        text = f"connected. {latency}ms"
    else:
        text = "not connected"
    screen.blit(font.render(text, False, line_col), (0, 0))

    loop_i += 1
    pygame.display.flip()       

sock.close()
pygame.quit()