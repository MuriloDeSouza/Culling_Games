import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd
import time

class ReactiveNavigation(Node):
    def __init__(self):
        super().__init__('reactive_navigation')  # Inicializa o nó ROS2 chamado 'reactive_navigation'
        
        # Cria um cliente para o serviço '/move_command' responsável pelos comandos de movimento do robô
        self.move_client = self.create_client(MoveCmd, '/move_command')
        
        # Aguarda o serviço '/move_command' ficar disponível
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço /move_command...')
        
        # Conjuntos e estruturas para armazenar o estado do robô
        self.visited_positions = set()  # Posições já visitadas
        self.backtrack_stack = []  # Posições que possuem múltiplas opções (para voltar caso necessário)

    def send_move_request(self, direction):
        """
        Envia uma requisição de movimento para o robô.
        :param direction: Direção para onde o robô deve se mover ('up', 'down', 'left', 'right').
        :return: Resultado da requisição (contendo informações de sucesso, posição atual e sensores).
        """
        request = MoveCmd.Request()  # Cria a requisição
        request.direction = direction  # Define a direção de movimento
        future = self.move_client.call_async(request)  # Envia a requisição de forma assíncrona
        rclpy.spin_until_future_complete(self, future)  # Espera pela resposta
        return future.result()  # Retorna o resultado do movimento

    def get_unvisited_directions(self, robot_pos, sensors):
        """
        Identifica as direções não visitadas com base na posição atual e nos sensores.
        :param robot_pos: Posição atual do robô.
        :param sensors: Dados dos sensores nas direções ('left', 'right', 'up', 'down').
        :return: Lista de direções não visitadas e suas posições correspondentes.
        """
        directions = []
        for direction, sensor in sensors.items():
            if sensor in ['f', 't']:  # Espaço livre ('f') ou alvo ('t')
                # Calcula a nova posição com base na direção
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
        """
        Prioriza direções que aproximam o robô do alvo com base na posição atual e nos sensores.
        :param robot_pos: Posição atual do robô.
        :param target_pos: Posição do alvo.
        :param sensors: Dados dos sensores nas direções ('left', 'right', 'up', 'down').
        :return: Lista de direções priorizadas e suas posições correspondentes.
        """
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
        """
        Verifica se o robô está próximo do alvo (a uma distância definida).
        :param robot_pos: Posição atual do robô.
        :param target_pos: Posição do alvo.
        :param distance: Distância limite para considerar "próximo" (default: 2).
        :return: True se o robô estiver próximo, False caso contrário.
        """
        return abs(robot_pos[0] - target_pos[0]) + abs(robot_pos[1] - target_pos[1]) <= distance

    def reactive_navigation(self):
        """
        Lógica principal de navegação reativa do robô, com priorização para o alvo.
        """
        while rclpy.ok():
            # Obter informações de sensores e posições
            result = self.send_move_request('')  # Solicita informações sem mover o robô
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
                # Lógica de backtracking
                if self.backtrack_stack:
                    self.get_logger().info('Nenhuma direção nova, voltando para último nó com opções...')
                    backtrack_pos = self.backtrack_stack.pop()
                    self.move_to_position(backtrack_pos)
                else:
                    self.get_logger().info('Sem movimentos válidos disponíveis e sem mais nós para voltar. Robô preso.')
                    break

    def move_to_position(self, target_pos):
        """
        Realiza backtracking para retornar a uma posição anterior com opções de movimento.
        :param target_pos: Posição para onde o robô deve retornar.
        """
        current_result = self.send_move_request('')
        robot_pos = tuple(current_result.robot_pos)

        while robot_pos != target_pos:
            time.sleep(1)  # Pequeno delay para simular navegação
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
    rclpy.init()  # Inicializa o sistema ROS2
    navigator = ReactiveNavigation()  # Cria o nó de navegação
    navigator.reactive_navigation()  # Executa a navegação
    rclpy.shutdown()  # Encerra o sistema ROS2

if __name__ == '__main__':
    main()
