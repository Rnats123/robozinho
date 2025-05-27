import pybullet as p
import pybullet_data
import time
import json
import random
import paho.mqtt.client as mqtt

# --- Configurações MQTT --- #
broker = "localhost"
control_topic = "robot/control"
status_topic = "robot/position"
marker_topic = "robot/markers"
client = mqtt.Client()

# Variáveis globais
current_cmd = None
target_markers = []
current_speed = 0.0
current_angular_speed = 0.0

# Função para publicar o status do robô
def publish_status():
    pos, _ = p.getBasePositionAndOrientation(robot)
    payload = {
        "posicao": [round(pos[0], 3), round(pos[1], 3)],
        "command": current_cmd
    }
    client.publish(status_topic, json.dumps(payload))

# Função para adicionar marcador na posição atual
def drop_marker():
    pos, _ = p.getBasePositionAndOrientation(robot)
    x, y = round(pos[0], 3), round(pos[1], 3)
    # Carrega um cubo pequeno para marcar (não consegui fazer funcionar)
    marker_id = p.loadURDF(
        "cube_small.urdf",
        [pos[0], pos[1], 0.1],
        p.getQuaternionFromEuler([0, 0, 0]),
        globalScaling=0.2
    )
    target_markers.append((x, y))
    # Publica posição do marcador (era pra marcar)
    client.publish(marker_topic, json.dumps({"marker": [x, y]}))
    print(f"📍 Marcador adicionado em ({x}, {y})")

# Controle de movimentação do robo
def on_control(client, userdata, msg):
    global current_cmd, current_speed, current_angular_speed
    cmd = msg.payload.decode('utf-8')
    if cmd in ('fwd', 're', 'left', 'right', 'stop', 'drop'):
        current_cmd = cmd
        if cmd == 'fwd':
            current_speed = 2.0
            current_angular_speed = 0.0
        elif cmd == 're':
            current_speed = -1.2
            current_angular_speed = 0.0
        elif cmd == 'left':
            current_speed = 0.0
            current_angular_speed = 1.0
        elif cmd == 'right':
            current_speed = 0.0
            current_angular_speed = -1.0
        elif cmd == 'stop':
            current_speed = 0.0
            current_angular_speed = 0.0
        elif cmd == 'drop':
            current_speed = 0.0
            current_angular_speed = 0.0
            drop_marker()
        publish_status()

# Inicializa MQTT
def setup_mqtt():
    client.on_message = on_control
    client.connect(broker, 1883, 60)
    client.subscribe(control_topic)
    client.loop_start()

# Iniciar do pyBullet e cena com blocos em locais aleatórios em um epaço de 20x20
def setup_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")
    num_obstacles = 20
    x_bounds = (-10, 10)
    y_bounds = (-10, 10)
    for _ in range(num_obstacles):
        x = random.uniform(*x_bounds)
        y = random.uniform(*y_bounds)
        p.loadURDF("cube.urdf", [x, y, 0.1], p.getQuaternionFromEuler([0,0,0]), globalScaling=0.5)

# Carrega o modelo do R2D2
robot = None

def load_robot():
    global robot
    robot = p.loadURDF("r2d2.urdf", [0, 0, 0.1], p.getQuaternionFromEuler([0, 0, 0]))

# Alerta de obstáculo no chão até 1 metro à frente
def check_warning_obstacle():
    pos, ori = p.getBasePositionAndOrientation(robot)
    rot = p.getMatrixFromQuaternion(ori)
    dir_x = rot[1]
    dir_y = rot[4]
    ray_length = 1.0
    ray_start = [pos[0], pos[1], 0.1]  
    ray_end = [
        pos[0] + dir_x * ray_length,
        pos[1] + dir_y * ray_length,
        0.1  
    ]
    result = p.rayTest(ray_start, ray_end)[0]
    hit_object = result[0]
    hit_fraction = result[2]
    if hit_object >= 0:
        distance = hit_fraction * ray_length
        print(f" Obstáculo no chão a {distance:.2f} metros à frente.")

# Quando aperta ele anda até apertar em parar
def apply_control():
    check_warning_obstacle()
    _, ori = p.getBasePositionAndOrientation(robot)
    rot = p.getMatrixFromQuaternion(ori)
    dir_x = rot[1]
    dir_y = rot[4]
    vx = current_speed * dir_x
    vy = current_speed * dir_y
    wz = current_angular_speed
    p.resetBaseVelocity(robot, linearVelocity=[vx, vy, 0], angularVelocity=[0, 0, wz])

# Loop principal
def main():
    setup_mqtt()
    setup_simulation()
    load_robot()
    print("Simulação iniciada. Use comandos MQTT e 'drop' para marcar.")
    try:
        while True:
            apply_control()
            p.stepSimulation()
            time.sleep(1/240)
    except KeyboardInterrupt:
        print("Encerrando...")
    finally:
        client.loop_stop()
        p.disconnect()

if __name__ == '__main__':
    main()
