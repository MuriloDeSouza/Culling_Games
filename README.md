# Documentação do projeto 

&emsp;&emsp; Aqui temos a atividade que foi disponibilizada pelo professor Rodrigo Nicola para que nós fizéssemos uma programação para que o ponto azul, navegue pelo mapa e chegue no target (ponto vermelho)

# Barema da ponderada

&emsp;&emsp; Para esta ponderada, sua tarefa será a de implementar dois métodos distintos de navegação; a navegação reativa e a navegação com mapa.

## Como?

&emsp;&emsp; Na parte 1 do desafio, você precisará navegar o labirinto de forma reativa; isso significa que o seu algoritmo não deve ter acesso ao mapa e deve navegar utilizando somente as informações capturadas pelos sensores exteroceptivos do seu robô. Na prática, isso significa que você terá acesso a:

- A posição atual do robô;
    
- A posição do alvo;
    
- O que tem nos quadrados imediatamente à esquerda, abaixo, acima e à direita do robô, nessa ordem.

&emsp;&emsp; Na parte 2 você passa a ter acesso ao mapa do labirinto e, com esse mapa, precisa planejar a rota do robô antes mesmo de começar a movê-lo, garantindo assim uma rota otimizada.

## Ferramenta de simulação

&emsp;&emsp; A ferramenta de simulação oferece um sistema feito em pygame, com interface direta com o ROS2. Nele, há um mapa implementado em formato de grid - isto é, há uma separação do espaço total do labirinto em quadrados de mesmo tamanho - e, neste mapa, há os seguintes elementos:

- O robô, caracterizado por um quadrado pintado de azul;

- O alvo, caracterizado por um quadrado pintado de vermelho;

- Os espaços livres - onde o robô pode andar -, caracterizados por quadrados brancos; e
    
- Os espaços obstruidos - onde o robô não pode andar -, caracterizados por quadrados pretos.

# Por onde começamos para rodar o código

&emsp;&emsp; Primeiro, devemos clonar o repositório para começarmos a rodar o projeto

1 - Clonar o repositório:
```bash
git clone https://github.com/MuriloDeSouza/Culling_Games.git
```

2 -**Ter o ROS instalado no computador para rodar esses comando na pasta raiz, caso você não tenha o ROS instalado na sua máquina, eu cou deixar disponibilizado o material para a instalação no final da documentação**
```bash
colcon build
source install/local_setup.bash 
```

3 - Rodar o comando para abrir o mapa na pasta raiz do projeto
```bash
ros2 run cg maze
```

4 - Estar no caminho determinado "/src/cg/cg" e rodar os códios de busca
```bash
python3 busca_largura.py
python3 busca_largura_2.py
python3 busca_profundidade.py
python3 busca_mapa.py
```
&emsp;&emsp; Depois que rodou esse código é só esperar um pouco, você vai conseguir ver o pontinho se movimentando por todo o mapa, guardando todas as posições que ele ja fez para não repetir, até chegar no objetivo que é o pontinho vermelho.

# Entendendo a lógica do código "busca_largura.py"

&emsp;&emsp; Nesse código o objetivo é que o pontinho azul ache o caminho até o "target". Fazemos isso atraveś de um algoritmo de BFS (Breadth-First Search) que é um algoritmo de busca em largura. E o que significa um algoritmo de busca em largura ???
Significa que ele vai tentar todas as possibilidades de caminho dentro de um quadrante e quando não existir mais para onde ele ir ou pontos novos ele vai passar para um novo quadrante e vai continuar minando as possibilidades desse novo quadrante e assim por diante até chegar no objetivo.
&emsp;&emsp; Com base nessa explicação superficial, ja vemos que xhegar no alvo vai demorar um pouco por que ele vai "varrer" o mapa inteiro até encontrar o alvo e, talvez, não vá pelo caminho mais curto. Más, como o importante é chegar, ele cumpre o seu papel muito bem.

# Passos do código

## Conexão com o serviço

* - Quando o código está tentando estabelecer uma conexão, o nó cliente está tentando conectar-se ao nó do serviço.

No método __init__, o cliente self.cli espera até que o serviço move_command esteja disponível. Esse é o ponto onde o nó cliente (o ReactiveNavigator) se conecta ao serviço que controla o movimento do robô.

## Função move_robot

* - O método move_robot envia uma solicitação de movimento e recebe a resposta sobre o que o robô "vê" nos quatro cantos.

A função envia a direção desejada ao serviço move_command e espera pela resposta, que inclui informações como o status do movimento (success), a nova posição do robô (robot_pos) e os obstáculos detectados nas direções left, right, up, e down.

## Função navigate

* - A fila usa BFS (Breadth-First Search) para explorar o ambiente.

* - A fila deque começa com a posição inicial e, durante a exploração, o código adiciona novas posições para visitação.

* - Se o robô bate em um obstáculo, ele tenta outras direções até encontrar o alvo.

* - O robô evita revisitar posições já exploradas.

**Sobre BFS (Busca em Largura):** É um algoritmo de busca que explora todas as posições em um nível antes de passar para o próximo. Isso significa que o robô verifica todas as posições mais próximas antes de explorar posições mais distantes.

A função navigate usa uma fila para gerenciar as posições a serem exploradas. O robô tenta mover para uma nova posição (se possível), marca-a como visitada, e adiciona novas posições adjacentes à fila. Caso encontre o alvo, o processo termina.

## O que o robô está fazendo?

* - O robô vai até um ponto, verifica os arredores, volta pelo mesmo caminho e tenta outras direções. Ele evita bater no mesmo obstáculo novamente.

O robô não "volta" pelo mesmo caminho a menos que precise tentar outra direção. Ele se move conforme a direção dada pela fila, e a lógica do BFS garante que ele explore todas as possibilidades adjacentes antes de passar para posições mais distantes.

# Entendendo a lógica do código "busca_largura_2.py"

&emsp;&emsp; Neste código, o objetivo é criar um sistema de navegação reativo para um robô que precisa alcançar um alvo em um ambiente desconhecido. A estratégia utilizada combina a lógica de exploração com priorização de movimentos para atingir o objetivo de forma eficiente (isso significa que ele vai ser mais rapido que o outro código de busca em largura). O sistema toma decisões de movimento com base em informações sensoriais e utiliza estruturas de dados para registrar posições já visitadas e caminhos alternativos. Este modelo evita revisitar locais desnecessários e prioriza direções que aproximam o robô do alvo, quando possível.
Conexão com o serviço
Estabelecendo a conexão

## Descrição:

No método __init__, o cliente self.move_client tenta conectar-se ao serviço /move_command, responsável por interpretar comandos de movimento do robô.
**Funcionamento:**
    * - A conexão é estabelecida após o serviço /move_command estar disponível.
    * - Durante a inicialização, o sistema aguarda o serviço com mensagens de log informando o status de conexão.

**O que acontece aqui?**
Esta etapa garante que o robô consiga enviar comandos e receber informações antes de iniciar a navegação.

## Função send_move_request

### Comando de movimento

**Descrição:**
    * - Envia comandos de movimento para o robô e retorna a resposta sobre o status do movimento, posição atual e leituras sensoriais.

**Funcionamento:**
    * - Cria uma requisição com a direção desejada ('up', 'down', 'left', 'right').
    * - Envia a requisição de forma assíncrona ao serviço.
    * - Aguarda pela resposta e retorna o resultado.

**Por que essa função é importante?**
Ela permite a comunicação entre o nó de navegação e o serviço de movimento, possibilitando que o robô receba instruções e dados do ambiente.

## Função reactive_navigation

### Lógica principal de navegação

**Descrição:**
    * - Controla o fluxo de navegação do robô, combinando exploração e priorização do movimento em direção ao alvo.

**Funcionamento:**
    * - Obtém a posição atual e informações dos sensores.
    * - Verifica se o alvo foi alcançado.
    * - Se o robô estiver próximo do alvo, prioriza movimentos que o aproximem.
    * - Caso contrário, explora direções ainda não visitadas.
    * - Se não houver mais direções disponíveis, realiza backtracking para retornar a posições com opções de movimento.

### Estratégias usadas:

**Exploração:** Utiliza a função get_unvisited_directions para identificar áreas inexploradas.
**Prioridade:** Quando próximo do alvo, a função prioritize_directions_toward_target calcula as direções que mais aproximam o robô do objetivo.
**Backtracking:** Retorna a posições anteriores quando não há mais movimentos válidos.

**Por que isso funciona?**
Essa abordagem combina exploração completa com uma lógica de aproximação ao alvo, garantindo que o robô seja eficiente e consiga lidar com obstáculos no caminho.

## Funções auxiliares

***get_unvisited_directions*** 

**Descrição:**
    * - Identifica direções que ainda não foram visitadas com base nos sensores e posição atual.
**Por que é útil?**
    * - Ajuda a explorar novas áreas sem repetir caminhos.

***prioritize_directions_toward_target***

**Descrição:**
    * - Determina as direções que aproximam o robô do alvo, considerando posições adjacentes e dados sensoriais.
**Por que é útil?**
    * - Quando próximo do alvo, essa função garante que o robô foque em alcançar o objetivo rapidamente.

***is_near_target***

**Descrição:**
    * - Verifica se o robô está próximo do alvo com base em uma distância definida.
**Por que é útil?**
    * - Permite alternar entre exploração geral e priorização de movimentos.

***move_to_position***

**Descrição:**
    Realiza o movimento do robô para uma posição específica durante o backtracking.
**Funcionamento:**
    Determina a direção necessária para alcançar a posição alvo.
    Envia comandos de movimento até que o robô retorne ao local desejado.

## O que o robô está fazendo?

**Explorando:** O robô usa sensores para identificar caminhos livres e alvos próximos.
**Registrando posições:** Marca as posições visitadas para evitar revisitas desnecessárias.
**Priorizando:** Movimentos em direção ao alvo têm prioridade quando o robô está próximo.
**Voltando:** Em caso de becos sem saída, o robô retorna a posições anteriores para explorar alternativas.

**Por que isso é eficiente?**
Combina técnicas de exploração completa e busca orientada para reduzir o tempo necessário para alcançar o alvo, enquanto evita loops ou movimentos ineficazes.

## Conclusão

&emsp;&emsp; O código "reactive_navigation.py" é uma solução bem estruturada para navegação reativa. Ele utiliza conceitos de exploração, priorização e backtracking para navegar em ambientes desconhecidos, equilibrando eficiência e robustez.


# Link do material para baixar o ROS

&emsp;&emsp; Esse manual de instalação foi feito por um professor de minha instituição, se seguir os passos conforme o descrito, vai conseguir instalar o ROS. Pegue o link abaixo e cole na sua aba de pesquisa:
```bash
https://rmnicola.github.io/m6-ec-encontros/O01/ros
```

# Entendendo a lógica do código "busca_mapa.py"

## Classe MapNavigation

### Objetivo do Sistema

&emsp;&emsp; O código implementa um sistema de navegação autônoma para um robô em um ambiente representado por um mapa 2D (labirinto). Utiliza o framework ROS 2 para comunicação, com serviços de movimentação (/move_command) e obtenção de mapas (/get_map). O sistema encontra o caminho do robô até um alvo utilizando o algoritmo A* (A-estrela).
Descrição Geral

## O sistema tem os seguintes componentes principais:

**Obtenção do mapa:** Requisita o mapa do ambiente em forma de matriz 2D.
**Planejamento de Caminho:** Utiliza o algoritmo A* para calcular o caminho do ponto inicial ao alvo, considerando obstáculos.
**Movimentação do Robô:** Envia comandos de direção ao robô para segui-lo até o destino.

## Arquitetura do Código

**Importações**

**rclpy:** Biblioteca ROS 2 em Python.
**cg_interfaces.srv:** Define os serviços personalizados de movimentação (MoveCmd) e obtenção de mapa (GetMap).
**heapq:** Usada para implementar a fila de prioridade no algoritmo A*.

## Classe MapNavigation
__init__

* - Inicializa o nó e os clientes ROS 2.
* - Aguarda os serviços estarem disponíveis.
* - Obtém o mapa, identifica as posições inicial e alvo, e calcula o caminho.

## get_map()

* - Faz requisição ao serviço /get_map.
* - Reconstrói o mapa 2D a partir de uma lista linear recebida.

## find_positions(map_2d)

* - Procura as posições inicial (r) e alvo (t) no mapa.

## plan_path(start, target)

**Implementa o algoritmo A* para calcular o caminho mais curto.**
    * - G-Score: Custo acumulado.
    * - F-Score: Custo acumulado + heurística (distância Manhattan).
    * - Considera vizinhos válidos, ignorando obstáculos (b).

## heuristic(pos, target)

* - Calcula a distância Manhattan entre duas posições.

## get_neighbors(pos)

* - Retorna vizinhos válidos de uma posição no mapa, com as direções correspondentes.

## reconstruct_path(came_from, current)

* - Reconstrói o caminho da posição inicial ao alvo utilizando o dicionário came_from.

## navigate()

* - Percorre o caminho planejado enviando comandos de movimento ao robô.

## send_move_request(direction)

* - Envia requisições para mover o robô em uma direção específica.

## Função main()

* - Inicializa o nó, cria a instância de navegação e executa o plano.

## Fluxo do Sistema

**Inicialização:**
    * - O nó ROS inicia e estabelece conexões com os serviços.
**Obtenção do Mapa:**
    * - O sistema solicita o mapa 2D e reconstrói a matriz.
**Identificação das Posições:**
    * - Encontra as coordenadas do robô e do alvo.
**Planejamento do Caminho:**
    * - O algoritmo A* calcula o caminho mais eficiente até o alvo.
**Execução da Navegação:**
    * - O sistema envia comandos de movimentação ao robô, seguindo o caminho calculado.

## Tecnologias e Ferramentas

**ROS 2:** Para comunicação entre o sistema e os serviços.
**Python:** Para implementar lógica de controle e algoritmos.
**Algoritmo A*:** Para planejamento de caminhos.

## Melhorias e Considerações

**Tratamento de Erros:** Adicionar verificações para falhas nos serviços.
**Performance:** Analisar desempenho em mapas grandes.
**Extensibilidade:** Suporte a múltiplos robôs ou mapas dinâmicos.

# Referências

&emsp;&emsp; Essa documentação foi de grande valia para mim por que eu consegui entender um pouco mais sobre os "Nós" dos clientes e dos serviços (em inglês se fala Publisher que é uma forma de publicar as informações).
[1] ROS 2 Documentation: Iron. Escrever uma editora e assinante simples (Python) Disponível em: [https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html). Acesso em: 12 novembro. 2024.

[2] ROS 2 Documentation: Iron. Escrever um serviço simples e um cliente (Python) Disponível em: [https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html). Acesso em: 12 novembro. 2024.