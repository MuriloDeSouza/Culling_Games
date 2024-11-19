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
