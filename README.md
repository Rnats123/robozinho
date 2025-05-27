ALUNO: NATAN MARQUES MONTEIRO CAVALCANTI TURNO: NOITE Projeto: robozinho

Relatório Técnico do Projeto: Robô Simulado com Controle Remoto via MQTT

1.Descrição do Projeto Este projeto tem como objetivo simular um robô móvel em um ambiente 3D utilizando o PyBullet, com controle remoto via mensagens MQTT. O robô pode se mover em várias direções e executar a ação de "dropar" marcadores no ambiente(era o objetivo, não consegui implementar), representando pontos de interesse ou entrega de objetos. A simulação inclui obstáculos aleatórios e feedback visual sobre proximidade de colisões. A comunicação entre usuário e robô ocorre por meio de tópicos MQTT utilizando mosquito e uma comunicação com o NODE-RED.

2.Objetivos Criar um ambiente de simulação com obstáculos. Controlar um robô virtual via comandos MQTT (FRENTE, RE, ESQUERDA, DIREITA, PARAR, DROP). Registrar marcadores no ambiente ao receber o comando drop. (não consegui implementar, tentei, tentei muito) Publicar a posição atual do robô em tempo real via MQTT. Implementar um alerta de obstáculos no caminho do robô.

3.Abordagem Adotada A implementação do sistema foi feita de forma incremental: Inicialmente, foi criado o ambiente simulado com plano, obstáculos e o robô R2D2. Em seguida, configurou-se a comunicação via MQTT. Foi implementado o controle básico de movimento. Tentei colocar a lógica para dropar marcadores e publicá-los via MQTT. Por fim, foi incluído um sistema de alerta de obstáculos.

4.Arquitetura do Sistema Componentes Principais: Simulador PyBullet: Gera o ambiente 3D e simula física realista. MQTT Broker: Gerencia os canais de comunicação entre cliente e robô. Cliente MQTT: Recebe comandos e publica atualizações. Robô R2D2: Modelo simulado que se move.

5.Tecnologias Utilizadas Python 3.10+ PyBullet – Simulador físico de corpo rígido. Mosquitto – MQTT para comunicação em tempo real. NODE-RED – Backend no-code onde conseguimos fazer toda a comunicação. JSON – Formato para mensagens publicadas.

6.Principais Decisões de Implementação Uso do R2D2 como modelo de robô, foi por que eu tentei com outro e tava todo bugado, ai foi ele mesmo. Controle remoto via MQTT em vez de interface gráfica, permitindo automação e testes remotos. Criação de obstáculos aleatórios a cada execução, para testar a navegação em cenários variados. Ray Casting para detecção de obstáculos à frente do robô.

7.Resultados Obtidos O robô pode ser controlado com sucesso usando comandos via MQTT usando o dashboard do nodered e a ui dele. A posição atual do robô é publicada a cada comando executado. O sistema alerta quando há um obstáculo próximo à frente.

8.Dificuldades Encontradas Botão de "Drop" físico ou interface gráfica não implementado: A ideia inicial era que o robô possuísse um botão para dropar o marcador, mas a dificuldade de tempo levou à decisão de usar deixar essa implementação de lado.
Fazer outro robô sem ser o R2D2 funcionar: Mesmo fazendo de tudo para o outro robo funcionar ele ficava quebrando o eixo das rodas ou só andava para esquerda, então a dificuldade levoua escolha do r2d2