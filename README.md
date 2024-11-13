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

2 - Ter o ROS instalado no computador para rodar esses comando na pasta raiz
```bash
colcon build
source install/local_setup.bash 
```

3 - Rodar o comando para abrir o mapa na pasta raiz do projeto
```bash
ros2 run cg maze
```

4 - Estar no caminho determinado "/src/cg/cg" e rodar o comando
```bash
python3 client_cmd.py
```