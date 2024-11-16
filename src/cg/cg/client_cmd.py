import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd
from collections import deque

class ReactiveNavigator(Node):
    def __init__(self):
        # Inicializa o nó ROS2 com o nome 'reactive_navigator'
        super().__init__('reactive_navigator')
        
        # Cria um cliente para o serviço 'move_command', que será usado para enviar comandos de movimento ao robô
        self.cli = self.create_client(MoveCmd, 'move_command')
        
        # Espera até que o serviço 'move_command' esteja disponível
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service move_command not available, waiting again...')
        
        # Cria uma requisição para o serviço 'MoveCmd'
        self.req = MoveCmd.Request()
        
        # Inicializa um conjunto para armazenar posições já visitadas, evitando que o robô revisite os mesmos pontos
        self.visited_positions = set()
        
        # Define a posição inicial do robô como (0, 0)
        self.current_position = (0, 0)

    def move_robot(self, direction):
        # Define a direção de movimento na requisição
        self.req.direction = direction
        
        # Envia a requisição ao serviço de movimento de forma assíncrona e espera pela resposta
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        # Retorna o resultado do movimento (success, posição, visão ao redor, etc.)
        return self.future.result()

    def navigate(self):
        # Mapeamento das direções com os respectivos deslocamentos de coordenadas
        directions_map = {
            'up': (0, 1),
            'down': (0, -1),
            'left': (-1, 0),
            'right': (1, 0)
        }
        
        # Cria uma fila (usando BFS) para explorar o mapa a partir da posição inicial do robô
        # Cada item da fila armazena uma posição e a direção usada para chegar nela
        queue = deque([(self.current_position, None)])  # Começa na posição inicial, sem direção de chegada
        
        while queue:
            # Remove o primeiro item da fila (posição atual e direção para chegar nela)
            current_position, direction_to_get_here = queue.popleft()
            
            # Marca a posição atual como visitada
            self.visited_positions.add(current_position)
            
            # Se existe uma direção para chegar à posição atual, executa o movimento
            if direction_to_get_here:
                response = self.move_robot(direction_to_get_here)
                
                # Verifica se a posição atual do robô é a mesma do alvo
                if response and all(a == b for a, b in zip(response.target_pos, response.robot_pos)):
                    self.get_logger().info("Target reached!")
                    return  # Encerra a navegação se o alvo foi encontrado
                
                # Se o movimento falhou, ignora a posição atual e passa para a próxima iteração
                if not response or not response.success:
                    continue

                # Loga informações sobre o movimento bem-sucedido e a visão do robô
                self.get_logger().info(f"Move successful to position {current_position}")
                self.get_logger().info(f"Robot sees: left={response.left}, down={response.down}, up={response.up}, right={response.right}")
            
            # Explora todas as direções possíveis (up, down, left, right) a partir da posição atual
            for direction, (dx, dy) in directions_map.items():
                # Calcula a nova posição baseada na direção
                new_position = (current_position[0] + dx, current_position[1] + dy)
                
                # Se a nova posição não foi visitada antes, adiciona à fila para exploração
                if new_position not in self.visited_positions:
                    queue.append((new_position, direction))

        # Loga uma mensagem caso a exploração do mapa termine sem encontrar o alvo
        self.get_logger().info("Exploration complete, target not found.")

def main(args=None):
    # Inicializa o sistema ROS2
    rclpy.init(args=args)
    
    # Cria uma instância da classe ReactiveNavigator
    navigator = ReactiveNavigator()
    
    # Inicia o processo de navegação e exploração
    navigator.get_logger().info("Starting navigation...")
    navigator.navigate()
    
    # Destrói o nó ao final da execução
    navigator.destroy_node()
    rclpy.shutdown()

# Ponto de entrada do programa
if __name__ == '__main__':
    main()

# Função dois para a navegação reativa
import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd

class ReactiveNavigation(Node):
    def __init__(self):
        super().__init__('reactive_navigation')
        self.move_client = self.create_client(MoveCmd, '/move_command')
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço /move_command...')

        # Histórico completo de posições visitadas e nós de decisão
        self.visited_positions = set()
        self.backtrack_stack = []  # Posições que atuam como "nós" com múltiplas opções

    def send_move_request(self, direction):
        request = MoveCmd.Request()
        request.direction = direction
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def get_unvisited_directions(self, robot_pos, sensors):
        """Retorna uma lista de direções que ainda não foram visitadas."""
        directions = []
        for direction, sensor in sensors.items():
            if sensor in ['f', 't']:  # Espaço livre ou alvo
                # Calcular a posição nova
                new_pos = list(robot_pos)
                if direction == 'left':
                    new_pos[1] -= 1
                elif direction == 'right':
                    new_pos[1] += 1
                elif direction == 'up':
                    new_pos[0] -= 1
                elif direction == 'down':
                    new_pos[0] += 1
                
                # Adiciona a direção se a nova posição ainda não foi visitada
                if tuple(new_pos) not in self.visited_positions:
                    directions.append((direction, tuple(new_pos)))
        return directions

    def prioritize_directions_toward_target(self, robot_pos, target_pos, sensors):
        """Retorna as direções priorizadas para se aproximar do alvo."""
        directions = []
        if robot_pos[0] < target_pos[0] and sensors['down'] in ['f', 't']:
            directions.append(('down', (robot_pos[0] + 1, robot_pos[1])))
        if robot_pos[0] > target_pos[0] and sensors['up'] in ['f', 't']:
            directions.append(('up', (robot_pos[0] - 1, robot_pos[1])))
        if robot_pos[1] < target_pos[1] and sensors['right'] in ['f', 't']:
            directions.append(('right', (robot_pos[0], robot_pos[1] + 1)))
        if robot_pos[1] > target_pos[1] and sensors['left'] in ['f', 't']:
            directions.append(('left', (robot_pos[0], robot_pos[1] - 1)))
        return directions

    def is_near_target(self, robot_pos, target_pos, distance=2):
        """Verifica se o robô está próximo do alvo (a até `distance` unidades de distância)."""
        return abs(robot_pos[0] - target_pos[0]) + abs(robot_pos[1] - target_pos[1]) <= distance

    def reactive_navigation(self):
        while rclpy.ok():
            # Obter informações de sensores e posições
            result = self.send_move_request('')  # Requisição sem direção para ler sensores
            robot_pos = tuple(result.robot_pos)
            target_pos = tuple(result.target_pos)
            left, down, up, right = result.left, result.down, result.up, result.right
            sensors = {'left': left, 'down': down, 'up': up, 'right': right}

            # Verificar se o robô chegou ao alvo
            if robot_pos == target_pos:
                self.get_logger().info('Alvo alcançado!')
                break

            # Registrar posição atual como visitada
            self.visited_positions.add(robot_pos)

            # Priorizar direções que se aproximam do alvo se estiver próximo
            if self.is_near_target(robot_pos, target_pos, distance=3):
                priority_directions = self.prioritize_directions_toward_target(robot_pos, target_pos, sensors)
                if priority_directions:
                    direction, new_pos = priority_directions[0]
                    move_result = self.send_move_request(direction)
                    if move_result.success:
                        self.get_logger().info(f"Movimento bem-sucedido para {direction}. Nova posição: {move_result.robot_pos}")
                        continue

            # Caso contrário, continuar com a lógica de exploração
            unvisited_directions = self.get_unvisited_directions(robot_pos, sensors)
            if len(unvisited_directions) > 1:
                self.backtrack_stack.append(robot_pos)

            if unvisited_directions:
                direction, new_pos = unvisited_directions[0]
                move_result = self.send_move_request(direction)
                if move_result.success:
                    self.get_logger().info(f"Movimento bem-sucedido para {direction}. Nova posição: {move_result.robot_pos}")
            else:
                if self.backtrack_stack:
                    self.get_logger().info('Nenhuma direção nova, voltando para último nó com opções...')
                    backtrack_pos = self.backtrack_stack.pop()
                    self.move_to_position(backtrack_pos)
                else:
                    self.get_logger().info('Sem movimentos válidos disponíveis e sem mais nós para voltar. Robô preso.')
                    break

    def move_to_position(self, target_pos):
        """Navega até a posição alvo (backtracking)."""
        current_result = self.send_move_request('')
        robot_pos = tuple(current_result.robot_pos)

        while robot_pos != target_pos:
            direction = None
            if robot_pos[0] < target_pos[0]:
                direction = 'down'
            elif robot_pos[0] > target_pos[0]:
                direction = 'up'
            elif robot_pos[1] < target_pos[1]:
                direction = 'right'
            elif robot_pos[1] > target_pos[1]:
                direction = 'left'

            move_result = self.send_move_request(direction)
            if move_result.success:
                robot_pos = tuple(move_result.robot_pos)
                self.get_logger().info(f"Backtracking para {direction}. Nova posição: {robot_pos}")
            else:
                self.get_logger().info(f"Falha ao voltar para {direction}. Tentando outra direção.")
                break

def main():
    rclpy.init()
    navigator = ReactiveNavigation()
    navigator.reactive_navigation()
    rclpy.shutdown()

if __name__ == '__main__':
    main()