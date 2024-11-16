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

4 - Estar no caminho determinado "/src/cg/cg" e rodar o comando
```bash
python3 client_cmd.py
```
&emsp;&emsp; Depois que rodou esse código é só esperar um pouco, você vai conseguir ver o pontinho se movimentando por todo o mapa, guardando todas as posições que ele ja fez para não repetir, até chegar no objetivo que é o pontinho vermelho.


# Link do material para baixar o ROS

&emsp;&emsp; Esse manual de instalação foi feito por um professor de minha instituição, se seguir os passos conforme o descrito, vai conseguir instalar o ROS. Pegue o link abaixo e cole na sua aba de pesquisa:
```bash
https://rmnicola.github.io/m6-ec-encontros/O01/ros
```

# Referências

&emsp;&emsp; Essa documentação foi de grande valia para mim por que eu consegui entender um pouco mais sobre os "Nós" dos clientes e dos serviços (em inglês se fala Publisher que é uma forma de publicar as informações).
[1] ROS 2 Documentation: Iron. Escrever uma editora e assinante simples (Python) Disponível em: [https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html). Acesso em: 12 novembro. 2024.

[2] ROS 2 Documentation: Iron. Escrever um serviço simples e um cliente (Python) Disponível em: [https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html). Acesso em: 12 novembro. 2024.