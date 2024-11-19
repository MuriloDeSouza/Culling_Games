import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd, GetMap
import heapq
class MapNavigation(Node):
    def __init__(self):
        super().__init__('map_navigation')
        self.move_client = self.create_client(MoveCmd, '/move_command')
        self.map_client = self.create_client(GetMap, '/get_map')
        # Aguarda até que os serviços estejam prontos
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço /move_command...')
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço /get_map...')
        # Obtenha o mapa do labirinto
        self.map = self.get_map()
        self.start_pos, self.target_pos = self.find_positions(self.map)
        # Planeje o caminho
        self.path = self.plan_path(self.start_pos, self.target_pos)
    def get_map(self):
        """Requisita o mapa usando o serviço /get_map e reconstrói o grid."""
        request = GetMap.Request()
        future = self.map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        map_data = future.result().occupancy_grid_flattened
        map_shape = future.result().occupancy_grid_shape
        # Reconstrua o mapa 2D a partir do grid planificado
        map_2d = [map_data[i:i + map_shape[1]] for i in range(0, len(map_data), map_shape[1])]
        return map_2d
    def find_positions(self, map_2d):
        """Encontra a posição inicial do robô e o alvo no mapa."""
        start_pos = None
        target_pos = None
        for i, row in enumerate(map_2d):
            for j, cell in enumerate(row):
                if cell == 'r':
                    start_pos = (i, j)
                elif cell == 't':
                    target_pos = (i, j)
        return start_pos, target_pos
    def plan_path(self, start, target):
        """Implementa o algoritmo A* para planejar o caminho."""
        # Configurações para A*
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, target)}
        while open_list:
            _, current = heapq.heappop(open_list)
            # Verifica se chegamos ao alvo
            if current == target:
                return self.reconstruct_path(came_from, current)
            # Expandir nós vizinhos
            for direction, neighbor in self.get_neighbors(current):
                if self.map[neighbor[0]][neighbor[1]] == 'b':
                    continue  # Ignora posições bloqueadas
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = (current, direction)
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, target)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
        # Retorna uma lista vazia se não encontrar caminho
        return []
    def heuristic(self, pos, target):
        """Calcula a heurística de Manhattan para A*."""
        return abs(pos[0] - target[0]) + abs(pos[1] - target[1])
    def get_neighbors(self, pos):
        """Retorna os vizinhos de uma posição com suas direções."""
        directions = {
            'up': (-1, 0),
            'down': (1, 0),
            'left': (0, -1),
            'right': (0, 1)
        }
        neighbors = []
        for direction, (di, dj) in directions.items():
            neighbor = (pos[0] + di, pos[1] + dj)
            if 0 <= neighbor[0] < len(self.map) and 0 <= neighbor[1] < len(self.map[0]):
                neighbors.append((direction, neighbor))
        return neighbors
    def reconstruct_path(self, came_from, current):
        """Reconstrói o caminho de volta usando o dicionário came_from."""
        path = []
        while current in came_from:
            current, direction = came_from[current]
            path.append(direction)
        path.reverse()
        return path
    def navigate(self):
        """Navega pelo caminho planejado."""
        for direction in self.path:
            result = self.send_move_request(direction)
            if result.success:
                self.get_logger().info(f"Movendo para {direction}. Posição atual: {result.robot_pos}")
            else:
                self.get_logger().warning("Falha ao mover.")
    def send_move_request(self, direction):
        """Envia o comando de movimento para o robô."""
        request = MoveCmd.Request()
        request.direction = direction
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
def main():
    rclpy.init()
    navigator = MapNavigation()
    navigator.navigate()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

    import rclpy  # Importa o pacote para lidar com ROS 2 em Python
from rclpy.node import Node  # Importa a classe base para criar nós em ROS 2
from cg_interfaces.srv import MoveCmd, GetMap  # Importa serviços personalizados MoveCmd e GetMap
import heapq  # Biblioteca para implementar a fila de prioridade usada no algoritmo A*

# Define a classe principal do nó ROS para navegação em mapas
class MapNavigation(Node):
    def __init__(self):
        super().__init__('map_navigation')  # Inicializa o nó com o nome 'map_navigation'
        
        # Criação dos clientes para os serviços de movimento e obtenção do mapa
        self.move_client = self.create_client(MoveCmd, '/move_command')
        self.map_client = self.create_client(GetMap, '/get_map')
        
        # Aguarda até que os serviços estejam disponíveis
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço /move_command...')
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço /get_map...')
        
        # Obtemos o mapa do ambiente
        self.map = self.get_map()
        self.start_pos, self.target_pos = self.find_positions(self.map)  # Determina posições inicial e alvo
        self.path = self.plan_path(self.start_pos, self.target_pos)  # Planeja o caminho usando A*

    def get_map(self):
        """Requisita o mapa do serviço /get_map e reconstrói a matriz do mapa."""
        request = GetMap.Request()
        future = self.map_client.call_async(request)  # Faz chamada assíncrona ao serviço
        rclpy.spin_until_future_complete(self, future)  # Espera a resposta do serviço
        map_data = future.result().occupancy_grid_flattened  # Obtém o mapa como lista linear
        map_shape = future.result().occupancy_grid_shape  # Obtém as dimensões do mapa
        # Reconstrói a matriz 2D a partir da lista linear
        map_2d = [map_data[i:i + map_shape[1]] for i in range(0, len(map_data), map_shape[1])]
        return map_2d

    def find_positions(self, map_2d):
        """Identifica as posições inicial ('r') e alvo ('t') no mapa."""
        start_pos = None
        target_pos = None
        for i, row in enumerate(map_2d):
            for j, cell in enumerate(row):
                if cell == 'r':  # Posição do robô
                    start_pos = (i, j)
                elif cell == 't':  # Posição do alvo
                    target_pos = (i, j)
        return start_pos, target_pos

    def plan_path(self, start, target):
        """Planeja o caminho do robô ao alvo usando o algoritmo A*."""
        # Inicializa a fila de prioridade
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}  # Para rastrear o caminho de volta
        g_score = {start: 0}  # Custo do início ao nó atual
        f_score = {start: self.heuristic(start, target)}  # Heurística + custo acumulado

        while open_list:
            _, current = heapq.heappop(open_list)  # Retira o nó com menor custo
            if current == target:  # Chegou ao alvo
                return self.reconstruct_path(came_from, current)
            
            for direction, neighbor in self.get_neighbors(current):  # Examina vizinhos
                if self.map[neighbor[0]][neighbor[1]] == 'b':  # Ignora blocos
                    continue
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = (current, direction)
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, target)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
        return []  # Retorna caminho vazio se não houver solução

    def heuristic(self, pos, target):
        """Calcula a distância de Manhattan para a heurística A*."""
        return abs(pos[0] - target[0]) + abs(pos[1] - target[1])

    def get_neighbors(self, pos):
        """Retorna vizinhos válidos e suas direções."""
        directions = {
            'up': (-1, 0), 'down': (1, 0), 'left': (0, -1), 'right': (0, 1)
        }
        neighbors = []
        for direction, (di, dj) in directions.items():
            neighbor = (pos[0] + di, pos[1] + dj)
            if 0 <= neighbor[0] < len(self.map) and 0 <= neighbor[1] < len(self.map[0]):
                neighbors.append((direction, neighbor))
        return neighbors

    def reconstruct_path(self, came_from, current):
        """Reconstrói o caminho a partir do dicionário 'came_from'."""
        path = []
        while current in came_from:
            current, direction = came_from[current]
            path.append(direction)
        path.reverse()  # Inverte para obter na ordem correta
        return path

    def navigate(self):
        """Executa a navegação pelo caminho planejado."""
        for direction in self.path:
            result = self.send_move_request(direction)
            if result.success:
                self.get_logger().info(f"Movendo para {direction}. Posição atual: {result.robot_pos}")
            else:
                self.get_logger().warning("Falha ao mover.")

    def send_move_request(self, direction):
        """Envia um comando de movimento ao robô."""
        request = MoveCmd.Request()
        request.direction = direction
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# Função principal do nó ROS
def main():
    rclpy.init()  # Inicializa o cliente ROS 2
    navigator = MapNavigation()  # Instancia a classe de navegação
    navigator.navigate()  # Inicia a navegação
    rclpy.shutdown()  # Encerra o cliente ROS 2

if __name__ == '__main__':
    main()  # Ponto de entrada do script
