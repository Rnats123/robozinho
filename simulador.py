import pybullet as p  # Simulador de física para robótica
import pybullet_data  # Dados padrão para PyBullet
import time  # Controle de tempo para a simulação
import json  # Manipulação de dados em formato JSON
import random  # Geração de números aleatórios
import paho.mqtt.client as mqtt  # Comunicação MQTT para controle remoto

# Configuração do cliente MQTT
broker = "localhost"
control_topic = "robot/control"
status_topic = "robot/position"
client = mqtt.Client()

# Variáveis globais para controle
current_cmd = None  # Comando atual recebido
current_speed = 0.0  # Velocidade linear atual
current_angular_speed = 0.0  # Velocidade angular atual

# Função para publicar o status do robô
def publish_status():
    pos, _ = p.getBasePositionAndOrientation(robot)
    payload = {
        "posicao": [round(pos[0], 3), round(pos[1], 3)],
        "command": current_cmd
    }
    client.publish(status_topic, json.dumps(payload))

# Callback para tratamento de mensagens MQTT
def on_control(client, userdata, msg):
    global current_cmd, current_speed, current_angular_speed
    cmd = msg.payload.decode('utf-8')
    if cmd in ('fwd', 're', 'left', 'right', 'stop'):
        current_cmd = cmd
        if cmd == 'fwd':
            current_speed = 2.0  # Velocidade para frente
            current_angular_speed = 0.0
        elif cmd == 're':
            current_speed = -1.2  # Velocidade para trás
            current_angular_speed = 0.0
        elif cmd == 'left':
            current_speed = 0.0
            current_angular_speed = 1.0  # Velocidade angular para esquerda
        elif cmd == 'right':
            current_speed = 0.0
            current_angular_speed = -1.0  # Velocidade angular para direita
        elif cmd == 'stop':
            current_speed = 0.0
            current_angular_speed = 0.0
        publish_status()

# Inicialização do cliente MQTT
client.on_message = on_control
client.connect(broker, 1883, 60)
client.subscribe(control_topic)
client.loop_start()

# Inicialização do PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Carregamento do plano de chão
p.loadURDF("plane.urdf")

# Adição de obstáculos
obstacles = []
for _ in range(4):
    x = random.uniform(2.0, 4.0)
    y = random.uniform(-1.5, 1.5)
    obst = p.loadURDF("cube.urdf", [x, y, 0.1], p.getQuaternionFromEuler([0, 0, 0]), globalScaling=1.0)
    obstacles.append(obst)

# Carregamento do modelo R2D2
robot = p.loadURDF("r2d2.urdf", [0, 0, 0.1], p.getQuaternionFromEuler([0, 0, 0]))

# Função para aplicar o controle baseado no comando atual
def apply_control():
    # Obter a orientação atual do robô
    _, orientation = p.getBasePositionAndOrientation(robot)
    
    # Converter quaternion para matriz de rotação
    rot_matrix = p.getMatrixFromQuaternion(orientation)
    
    # Extrair o vetor de direção (eixo Y da matriz de rotação)
    dir_x = rot_matrix[1]
    dir_y = rot_matrix[4]
    
    # Inicializar velocidades
    vx = 0.0
    vy = 0.0
    wz = 0.0
    
    # Determinar velocidades com base no comando atual
    if current_cmd == 'fwd':
        vx = current_speed * dir_x
        vy = current_speed * dir_y
    elif current_cmd == 're':
        vx = current_speed * dir_x
        vy = current_speed * dir_y
    elif current_cmd == 'left':
        wz = current_angular_speed
    elif current_cmd == 'right':
        wz = current_angular_speed  # Correção: valor positivo para girar à direita
    elif current_cmd == 'stop' or current_cmd is None:
        vx = 0.0
        vy = 0.0
        wz = 0.0
    
    # Aplicar velocidades linear e angular
    p.resetBaseVelocity(robot, linearVelocity=[vx, vy, 0], angularVelocity=[0, 0, wz])

# Loop principal da simulação
print("Simulação R2D2 em andamento. Use os comandos MQTT para controlar o robô.")
try:
    while True:
        apply_control()
        p.stepSimulation()
        time.sleep(1/240)
except KeyboardInterrupt:
    print("Encerrando simulação...")
finally:
    client.loop_stop()
    p.disconnect()
