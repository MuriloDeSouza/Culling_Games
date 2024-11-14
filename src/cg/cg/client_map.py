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