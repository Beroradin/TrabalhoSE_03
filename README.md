# ✨ Trabalho 03 - Semáforo Inteligente utilizando FreeRTOS

<p align="center"> Repositório dedicado ao Trabalho 03 - SE do processo de capacitação do EmbarcaTech que envolve a implementação de um sistema embarcado que funciona como um semáforo inteligente, fornecendo as intruções de trânsito de forma tanto visual quanto auditiva na placa Raspberry Pi Pico W por meio da Plataforma BitDogLab.</p>

## :clipboard: Apresentação da tarefa

Para o terceiro trabalho da trilha de Sistemas Embarcados (SE) da fase 2 do Embarcatech foi necessário a implementação de um semáforo inteligente com o uso do FreeRTOS. Foi utilizado as funções básicas do FreeRTOS (não era permitido semáforo, mutexes, queues), como também, os periféricos buzzer, LED RGB, Matriz de LEDs, Display OLED ssd1306 e botão.

## :dart: Objetivos

- Compreender o funcionamento e a aplicação de sistemas operacionais (FreeRTOS) em microcontroladores;

- Compreender a diferença entre bare-metal e SO;

- Implementar um semáforo inteligente com funções básicas da biblioteca FreeRTOS;

- Indicações visuais dos estados do semáforo (LED RGB, Matriz de LEDs, Display OLED);

- Indicação auditiva dos estados do semáforo por meio do buzzer;

- Dois modos de estado: 1 - Normal, 2 - Noturno;

- Utilizar tasks separadas para cada periférico e um para a alternar entre os modos;

- Utilização de vTaskDelay, Continue, Break e TickCount para mudanças rápidas entre estados e debounce do botão.

## :books: Descrição do Projeto

Utilizou-se a placa BitDogLab (que possui o microcontrolador RP2040) para a exibição audiovisual de um semáforo inteligente, que permite pessoas com ou sem deficiências visuais saberem os estados do semáforo (VERDE, AMARELO, VERMELHO, MODO NOTURNO). Ainda na parte do firmware, foi utilizado um SO (Sistema Operacional) chamado FreeRTOS, para o controle de tarefas e "memory management" do microcontrolador. Cada periférico utilizou uma tarefa distinta, existindo uma apenas para a mudança do estado "Normal" para "Noturno" e vice-versa.

## :walking: Integrantes do Projeto

- Matheus Pereira Alves

## :bookmark_tabs: Funcionamento do Projeto
- O firmware inicia ligando o taskScheduler() do freeRTOS no main(), comando as tarefas individualmente;
- As tarefas começam seus loops individuais, funcionando por meio de ciclos (Verde 4s - Amarelo 2s - Vermelho 8s / Modo noturno - Amarelo 2s - Off 2s), podendo ser alterado de um para o outro por meio da tarefa do botão e sua flag;
- As tarefas se sincronizam por meio do ticks do clock e variáveis globais que indicam os modos e estados atuais;
- As tarefas possuem prioridades de número igual;

## :eyes: Observações

## :camera: GIF mostrando o funcionamento do programa na placa BitDogLab
<p align="center">
  <img src="images/trabalho03.gif" alt="GIF" width="526px" />
</p>

## :arrow_forward: Vídeo no youtube mostrando o funcionamento do programa na placa BitDogLab

<p align="center">
    <a href="https://youtu.be/pvB4bP7rnCA">Clique aqui para acessar o vídeo</a>
</p>
